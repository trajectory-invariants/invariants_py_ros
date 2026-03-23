#!/usr/bin/env python3

import rospy
import json
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as Rot
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from invariants_py.kinematics.orientation_kinematics import quat2rot, rot2quat
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
import std_msgs.msg
from invariants_py.kinematics.rigidbody_kinematics import orthonormalize_rotation as orthonormalize
from invariants_py.ocp_initialization import initial_trajectory_movingframe_rotation as FSr_init
from invariants_py.reparameterization import interpR
from invariants_py.generate_trajectory import opti_generate_pose_traj_from_vector_invars as OCP_gen
import invariants_py.spline_handler as sh
from ros_spline_fitting_trajectory.msg import Trajectory
import rospkg
from invariants_py import data_handler as dh
import helper_funtions as hf

class OCP_results:

    def __init__(self,FSt_frames,FSr_frames,Obj_pos,Obj_frames,invariants):
        self.FSt_frames = FSt_frames
        self.FSr_frames = FSr_frames
        self.Obj_pos = Obj_pos
        self.Obj_frames = Obj_frames
        self.invariants = invariants

class invariants_traj_gen_node:

    def __init__(self, name = "talker") -> None:

        rospy.init_node(name, anonymous=True)
        
        #Initialise variables 
        self.tf = None # [x,y,z,qx,qy,qz,qw]
        self.progress = 0
        self.progress_fv = 0
        self.current_progress_offset = 0
        self.progress_sum = 0
        self.progress_offset = 0
        self.counter = 0
        self.condition = 0
        self.dist_ratio = 1
        self.enter_recovery_mode = 1
        self.recovery_target = 100 * np.ones(7)
        self.current_sample = 0
        self.factor_execution_speed = 1
        self.jointvel = np.zeros(7)
        self.R_w_tgt = None
        self.pos_w_tgt = None
        self.bottle_pos_real = np.array([100,100,100]) # to understand if use real position measurements or simulated
        self.bottle_pos_sim = np.array([100,100,100])
        self.previous_pos_w_bottle = np.array([100,100,100])
        self.home = np.zeros(3)
        self.trajectory = Trajectory()

        rospack = rospkg.RosPack()
        #%%
        rospy.Subscriber('sim_bottle_pos_pub', Float64MultiArray, self.callback_bottle_pos_sim)
        rospy.Subscriber('/pickit/objects_wrt_robot_frame', Float64MultiArray, self.callback_bottle_pos_real) # THIS IS EXAMPLE OF BOTTLE POSITION DETECTED BY SENSOR - NOT IMPLEMENTED CORRECTLY YET (update also in listener_node)
        rospy.Subscriber('tcp_pose', Float64MultiArray, self.callback_tf)
        rospy.Subscriber('progress', Float64MultiArray, self.callback_progress) # I need some kind of progress, representing the progress of the robot along the trajectory
        rospy.Subscriber('progress_fv', Float64MultiArray, self.callback_progress_fv) # This should be the value of the progress feature variable
        rospy.Subscriber('progress_offset_previous', Float64MultiArray, self.callback_progress_offset_previous) # It's the progress offset of the previous sample
        rospy.Subscriber('factor_execution_speed', Float64MultiArray, self.callback_factorexecspeed)
        rospy.Subscriber('jointvel', Float64MultiArray, self.callback_jointvel)
        rospy.Subscriber('home_position', Float64MultiArray, self.callback_home)
        self.pub_node_output = rospy.Publisher('/raw_inv_gen_node_output', Float64MultiArray, queue_size=10, latch=True)
        self.publisher_trajectory = rospy.Publisher('/trajectory_pub', Trajectory, queue_size=10)
        self.publisher_traj_gen_marker = rospy.Publisher('/trajectory_marker', Marker, queue_size=10)
        #%% User inputs
        self.number_samples = 50
        self.threshold_new_target_measurement = 0.005
        self.delay_sample = 1
        self.max_inv_err = 30
        self.debug_mode = True


        # Solver choice
        self.solver = "fatrop" # options "ipopt", "fatrop"

        # OCP params
        # Robot parameters
        self.robot_params = {
            "urdf_file_name": None, # use None if do not want to include robot model
            # "q_init": np.array([-np.pi, -2.27, 2.27, -np.pi/2, -np.pi/2, np.pi/4]), # Initial joint values for UR10
            "q_init": np.array([-0.06999619209628122, -0.9936042374309739, 0.04052275688381114, -3.040355360901146, 0.024184582866498106, 2.0659148485780445, 0.7930461100688347]), # Initial joint values for Franka Panda
            "tip": 'TCP_frame' # Name of the robot tip (if empty standard 'tool0' is used)
        }
        # Define OCP weights
        self.weights_params = {
            "w_invars": 0.1*np.array([0.1*1, 0.1*1, 0.1*1, 5, 1.0, 1.0]),
            # "w_invars": 0.1*np.array([0.1*1, 0.1*1, 0.1*1, 50, 10.0, 10.0]), # to use when include robot kin model
            "w_high_start": round(0.7*self.number_samples),
            "w_high_end": self.number_samples,
            "w_high_invars": 0.5*np.array([0.1*1/5, 0.1*1/5, 0.1*1/5, 5, 1, 1]),
            # "w_high_invars": 0.5*np.array([0.1*1/5, 0.1*1/5, 0.1*1/5, 50, 10, 10]), # to use when include robot kin model
            "w_high_active": 1
        }

        # Parameters for orientation defintion
        self.quat_demo = np.array([0.907466,-0.416623,0.0328278,-0.0430416]) # orientation quaternion (defined as [qw,qx,qy,qz]) of the final pose of the human demonstration
        self.radius = 0.0145 + 0.015 # radius of the bottle (stella :0.0145, FM: 0.021) + buffer of 0.015
        self.alpha = np.array([np.pi/2]) # angle of approach direction around z-axis in rad

        # Choice of demonstration
        # Crate filling
        model_filename = rospack.get_path("invariants_py_ros")+"/data/"+"cf_inv.csv"
        pos_filename = rospack.get_path("invariants_py_ros")+"/data/"+"cf_pobj.csv"
        R_filename = rospack.get_path("invariants_py_ros")+"/data/"+"cf_Robj.csv"
        FSt_filename = rospack.get_path("invariants_py_ros")+"/data/"+"cf_FSt.csv"
        FSr_filename = rospack.get_path("invariants_py_ros")+"/data/"+"cf_FSr.csv"

        # Water pouring
        # model_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_inv.csv"
        # pos_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_pobj.csv"
        # R_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_Robj.csv"
        # FSt_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_FSt.csv"
        # FSr_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_FSr.csv"

        # Bottle approach
        # model_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_inv.csv"
        # pos_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_pobj.csv"
        # R_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_Robj.csv"
        # FSt_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_FSt.csv"
        # FSr_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_FSr.csv"

        self.invariant_model = dh.read_invariants_from_csv(model_filename)
        self.demo_pos = np.loadtxt(pos_filename, delimiter=",")
        self.demo_pos -= self.demo_pos[0]
        loaded_Robj = np.loadtxt(R_filename, delimiter=",")
        self.demo_Robj = loaded_Robj.reshape(loaded_Robj.shape[0], loaded_Robj.shape[1] // 3, 3)
        loaded_FSt = np.loadtxt(FSt_filename, delimiter=",")
        self.demo_FSt = loaded_FSt.reshape(loaded_FSt.shape[0], loaded_FSt.shape[1] // 3, 3)
        loaded_FSr = np.loadtxt(FSr_filename, delimiter=",")
        self.demo_FSr = loaded_FSr.reshape(loaded_FSr.shape[0], loaded_FSr.shape[1] // 3, 3)

        # new constraints       
        alpha = 0
        rotate = R.from_euler('z', alpha, degrees=True)
        R_w_tgt =  orthonormalize(rotate.as_matrix() @ self.demo_Robj[-1])
        FSt_end = orthonormalize(rotate.as_matrix() @ self.demo_FSt[-1])

        # # Linear initialization
        R_obj_init = interpR(np.linspace(0, 1, self.number_samples), [0,1], np.array([np.eye(3), R_w_tgt]))

        R_r_init, R_r_init_array, invars_init = FSr_init(np.eye(3), R_w_tgt, N=self.number_samples)
        
        # Initialising the boundary constraints with zeros or I
        self.boundary_constraints = {
            "position": {
                "initial": np.zeros(3),
                "final": np.zeros(3)
            },
            "orientation": {
                "initial": np.eye(3),
                "final": np.eye(3)
            },
            "moving-frame": {
                "translational": {
                    "initial": np.eye(3),
                    "final": np.eye(3)
                },
                "rotational": {
                    "initial": np.eye(3),
                    "final": np.eye(3)
                }
            },
        }
        
        dummy = { "inv_sol": np.loadtxt(dh.find_data_path("dummy_sol.csv"),delimiter=","), 
          "inv_demo": np.loadtxt(dh.find_data_path("dummy_inv.csv"),delimiter=","), 
          "R_t": np.loadtxt(dh.find_data_path("dummy_R_t.csv"),delimiter=",").reshape(100,3,3), 
          "R_r": np.loadtxt(dh.find_data_path("dummy_R_r.csv"),delimiter=",").reshape(100,3,3)}
        
        self.new_traj = OCP_results(FSt_frames = [], FSr_frames = [], Obj_pos = 100*np.ones((self.number_samples,3)), Obj_frames = np.ones((self.number_samples,3,3))*np.eye(3), invariants = np.zeros((self.number_samples,6)))
        self.current_traj = OCP_results(FSt_frames = [], FSr_frames = [], Obj_pos = 100*np.ones((self.number_samples,3)), Obj_frames = np.ones((self.number_samples,3,3))*np.eye(3), invariants = np.zeros((self.number_samples,6)))

        self.FS_online_generation_problem = OCP_gen.OCP_gen_pose(self.boundary_constraints,self.number_samples,solver=self.solver,robot_params=self.robot_params, dummy=dummy)
        
        # Resample model invariants to desired number of self.number_samples samples
        progress_values,model_invariants,progress_step = hf.resample_invariants(self.invariant_model,self.progress,self.number_samples)

        self.initial_values = {
            "trajectory": {
                "position": np.array([np.interp(np.linspace(0,len(self.demo_pos)-1,num=self.number_samples),np.linspace(0,len(self.demo_pos)-1,num=len(self.demo_pos)),self.demo_pos[:,i]) for i in range(3)]).T, 
                "orientation": R_obj_init
            },
            "moving-frame": {
                "translational": interpR(np.linspace(0,1,self.number_samples),np.linspace(0,1,len(self.demo_pos)),self.demo_FSt),
                "rotational": R_r_init_array
            },
            "invariants": model_invariants,
            "joint-values": self.robot_params["q_init"] if self.robot_params["urdf_file_name"] is not None else {}
        }

        self.lookup_table,self.lookup_table_peak = hf.setup_heuristic_lookuptable(self.demo_pos)

        # Create the Marker message
        self.marker = Marker()
        self.marker.header = std_msgs.msg.Header()
        self.marker.header.frame_id = "world"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD

        # Set self.marker properties
        self.marker.scale.x = 0.01
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0

        if self.pos_w_tgt is None:
            print("Waiting for a target pose to be published on /sim_bottle_pos_pub topic...")
            rospy.wait_for_message('/sim_bottle_pos_pub', Pose)
        if self.tf is None:
            print("Waiting for a start pose to be published on /tcp_pose topic...")
            rospy.wait_for_message('/tcp_pose', Pose)
            
    def callback_bottle_pos_sim(self,data):

        self.bottle_pos_sim = data.data

    def callback_bottle_pos_real(self,data):

        self.bottle_pos_real = data.data

    def callback_progress(self,data):

        self.progress = data.data[0]

    def callback_progress_fv(self,data):

        self.progress_fv = data.data[0]
    
    def callback_progress_offset_previous(self,data):

        self.current_progress_offset = data.data[0]
        
    def callback_tf(self,data):

        self.tf = data.data

    def callback_jointvel(self,data):

        self.jointvel = data.data

    def callback_factorexecspeed(self,data):

        self.factor_execution_speed = data.data[0]

    def callback_home(self,data):

        self.home = data.data

    def check_condition(self):
        if not self.tf[0] == 0:
            # Automatically detect if there is a real bottle position measurement or if the simulated one should be used
            if all(i == 100 for i in self.bottle_pos_real):
                self.pos_w_bottle = self.bottle_pos_sim
            else:
                self.pos_w_bottle = self.bottle_pos_real

            if all(np.isclose(self.pos_w_bottle,self.previous_pos_w_bottle,0,self.threshold_new_target_measurement)) and self.enter_recovery_mode == 0:
                self.condition = 1
            else:
                if self.debug_mode:
                    print("New target detected, a new trajectory will be generated")
                self.condition = 0
                self.previous_pos_w_bottle = self.pos_w_bottle

    def generate_trajectory(self):
        if not self.tf[0] == 0 and not self.home[0] == 0:
            # Define the target pose, by adding the bottle radius distance to the bottle position measurement and a small offset to ensure no contact between opener and bottle
            # Rotate the target pose around the z-axis by the alpha_deg angle (currently fixed), defining the desired approach direction
            # FOR BOTTLE APPROACH
            # self.pos_w_tgt = np.array([self.pos_w_bottle[0]-self.radius*np.sin(self.alpha[0]),self.pos_w_bottle[1]+self.radius*np.cos(self.alpha[0]),self.pos_w_bottle[2]])
            # self.R_w_tgt = Rot.from_euler('z', self.alpha[0]).as_matrix() @ Rot.from_quat([self.quat_demo[1],self.quat_demo[2],self.quat_demo[3],self.quat_demo[0]]).as_matrix()
            # FOR CRATE FILLING
            self.pos_w_tgt = self.pos_w_bottle
            self.R_w_tgt = np.eye(3)
            quat_w_tgt = rot2quat(self.R_w_tgt.reshape(1,3,3)).reshape(4) # [qx,qy,qz,qw]
            self.pose_w_tgt = np.hstack([self.pos_w_tgt,np.array([quat_w_tgt[3],quat_w_tgt[0],quat_w_tgt[1],quat_w_tgt[2]])])
            
            if self.counter == 0:
                self.previous_target = self.pos_w_tgt

            self.current_sample = hf.find_current_sample(self.tf[:3],self.current_traj.Obj_pos,self.number_samples)

            # Predict the robot pose in 100ms (ros node rate) by taking the model pose in self.delay_sample sample(s)
            pos_w_tcp, R_w_tcp = hf.predict_robot_pose(self.delay_sample,self.jointvel,self.tf[:3],self.tf[3:],self.enter_recovery_mode,self.current_sample,self.current_traj.Obj_pos,self.current_traj.Obj_frames)

            # s_prior,self.progress_sum = hf.progress_heuristic(self.previous_target,pos_w_tcp,self.pos_w_tgt,self.progress_fv,self.current_progress_offset)
            s_prior,self.progress_sum = hf.progress_heuristic(self.pos_w_tgt,pos_w_tcp,self.progress_fv,self.current_progress_offset,self.lookup_table,self.lookup_table_peak,)

            # new constraints TODO  
            alpha = 0
            rotate = R.from_euler('z', alpha, degrees=True)
            R_w_tgt =  orthonormalize(rotate.as_matrix() @ self.demo_Robj[-1])
            FSt_end = orthonormalize(rotate.as_matrix() @ self.demo_FSt[-1])

            # Resample model invariants to desired number of self.number_samples samples
            progress_values,model_invariants,progress_step = hf.resample_invariants(self.invariant_model,s_prior,self.number_samples)

            # Specify the boundary constraints
            self.boundary_constraints["position"]["initial"] = pos_w_tcp
            self.boundary_constraints["position"]["final"] = self.pos_w_tgt
            self.boundary_constraints["orientation"]["initial"] = R_w_tcp
            self.boundary_constraints["orientation"]["final"] = R_w_tgt # Start simple, where R_w_tgt is constant and equal to demo_Robj (for cf), TODO add variability in orientation!
            if self.counter == 0:
                self.boundary_constraints["moving-frame"]["translational"]["initial"] = orthonormalize(self.demo_FSt[round((self.current_sample)*len(self.demo_pos)/self.number_samples)])
                self.boundary_constraints["moving-frame"]["rotational"]["initial"] = orthonormalize(self.demo_FSr[round((self.current_sample)*len(self.demo_pos)/self.number_samples)])
            else:
                if self.enter_recovery_mode == 0:
                    self.boundary_constraints["moving-frame"]["translational"]["initial"] = orthonormalize(self.current_traj.FSt_frames[self.current_sample+self.delay_sample])
                    self.boundary_constraints["moving-frame"]["rotational"]["initial"] = orthonormalize(self.current_traj.FSr_frames[self.current_sample+self.delay_sample])
                else:
                    self.boundary_constraints["moving-frame"]["translational"]["initial"] = orthonormalize(self.current_traj.FSt_frames[self.current_sample-self.delay_sample])
                    self.boundary_constraints["moving-frame"]["rotational"]["initial"] = orthonormalize(self.current_traj.FSr_frames[self.current_sample-self.delay_sample])
            self.boundary_constraints["moving-frame"]["translational"]["final"] = FSt_end
            self.boundary_constraints["moving-frame"]["rotational"]["final"] = self.demo_FSr[-1]

            self.initial_values["trajectory"]["position"] += self.home

            if self.enter_recovery_mode == 1 and self.counter > 0:
                self.initial_values["trajectory"]["position"] = np.array([np.interp(np.linspace(round(s_prior*len(self.demo_pos)),len(self.demo_pos)-1,num=self.number_samples),np.linspace(round(s_prior*len(self.demo_pos)),len(self.demo_pos)-1,num=len(self.demo_pos)-round(s_prior*len(self.demo_pos))),self.demo_pos[round(s_prior*len(self.demo_pos)):,i]) for i in range(3)]).T
                # self.initial_values["trajectory"]["orientation"] =
                self.initial_values["moving-frame"]["translational"] = interpR(np.linspace(s_prior,1,self.number_samples),np.linspace(s_prior,1,len(self.demo_pos)-round(s_prior*len(self.demo_pos))),self.demo_FSt[round(s_prior*len(self.demo_pos)):])
                # self.initial_values["moving-frame"]["rotational"] =
                self.initial_values["invariants"] = model_invariants
                self.initial_values["trajectory"]["position"] += self.home
                if self.debug_mode:
                    A = self.initial_values["trajectory"]["position"][0,:]
                    print(f"s_prior= {s_prior},init_pos start = {A}, first point old traj {self.new_traj.Obj_pos[0,:]}")
                    B = self.boundary_constraints["position"]["initial"]
                    print(f"current sample = {self.current_sample}, bound constr pos init {B},home pos {self.home}")
                # self.initial_values["joint-values"]

            # Generate trajectory
            if self.debug_mode:
                print("")
                print("Generating new traj")
            self.new_traj.invariants, self.new_traj.Obj_pos, self.new_traj.Obj_frames, self.new_traj.FSt_frames, self.new_traj.FSr_frames, joint_values, inv_err = self.FS_online_generation_problem.generate_trajectory(model_invariants,self.boundary_constraints,progress_step,self.weights_params,self.initial_values,output_inverr=True,recovery_mode=self.enter_recovery_mode)
            if self.debug_mode:
                print(f"Invariant error = {inv_err}")

            if np.linalg.norm(self.pos_w_tgt - self.new_traj.Obj_pos[-1]) < 0.01 and inv_err < self.max_inv_err and np.linalg.norm(self.new_traj.Obj_pos[0,:]-pos_w_tcp) < 0.02:
                if self.debug_mode:
                    print("The trajectory fits the requirements - sent to the controller")
                self.enter_recovery_mode = 0
                self.recovery_target = self.pose_w_tgt.copy()
                self.progress_offset = self.current_progress_offset + s_prior - self.progress_sum
                # Publish trajectory
                self.trajectory = Trajectory()
                quaternion = rot2quat(self.new_traj.Obj_frames)
                for i in range(self.new_traj.invariants.shape[0]):
                    pose = Pose()
                    pose.position.x = self.new_traj.Obj_pos[i,0]
                    pose.position.y = self.new_traj.Obj_pos[i,1]
                    pose.position.z = self.new_traj.Obj_pos[i,2]
                    pose.orientation.x = quaternion[i,0]
                    pose.orientation.y = quaternion[i,1]
                    pose.orientation.z = quaternion[i,2]
                    pose.orientation.w = quaternion[i,3]
                    self.trajectory.poses.append(pose)

                self.current_traj.Obj_pos = self.new_traj.Obj_pos.copy()
                self.current_traj.FSt_frames = self.new_traj.FSt_frames.copy()
                self.current_traj.FSr_frames = self.new_traj.FSr_frames.copy()
            else:
                if self.debug_mode:
                    print("ENTERING RECOVERY MODE because:")
                    if not np.linalg.norm(self.pos_w_tgt - self.new_traj.Obj_pos[-1]) < 0.01:
                        print(f"- New trajectory doesn't reach the target, dist = {np.linalg.norm(self.pos_w_tgt - self.new_traj.Obj_pos[-1])} > 0.01")
                    if inv_err >=30:
                        print(f"- Invariants error = {inv_err} >= {self.max_inv_err}")
                    if not np.linalg.norm(self.new_traj.Obj_pos[0,:]-pos_w_tcp) < 0.02:
                        print(f"- New trajectory doesn't start at pos_w_tcp, dist = {np.linalg.norm(self.new_traj.Obj_pos[0,:]-pos_w_tcp)} > 0.02")
                self.enter_recovery_mode = 1
                self.pose_w_tgt = self.recovery_target.copy()
            if self.debug_mode:
                print(f"s_prior = {s_prior}, dist initial point new traj to estimated tcp {np.linalg.norm(self.new_traj.Obj_pos[0,:]-pos_w_tcp)}")
                print(f"actual tcp pos = {self.tf[:3]},estimated tcp pos = {pos_w_tcp}, initial point new traj = {self.new_traj.Obj_pos[0,:]}")
                print("")

            # Plot invariants signature of first trajectory vs demonstration
            # if self.counter == 1:
            #     hf.plot_invariants(self.invariant_model,progress_values,self.new_traj.invariants)

            # Add points to the marker
            self.marker.points = []
            for point in self.new_traj.Obj_pos:  # Assuming traj is a numpy array of shape (self.number_samples, 3)
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = point[2]
                self.marker.points.append(p)

            # Publish the Marker
            self.marker.header.stamp = rospy.Time.now() # add timestamp
            self.publisher_traj_gen_marker.publish(self.marker)

            self.counter += 1

            self.previous_target = self.pos_w_tgt

            if self.counter > 0:
                self.publisher_trajectory.publish(self.trajectory)

            outputs = np.hstack((self.progress_offset, self.pose_w_tgt, self.enter_recovery_mode, self.alpha))

            self.pub_node_output.publish(Float64MultiArray(data=outputs))

    def reset_OCP(self):
        if not self.tf[0] == 0:
            if self.home[0] == 0 and self.counter > 0:
                if self.debug_mode:
                    print("RESETTING OCP",self.home)
                self.FS_online_generation_problem = OCP_gen.OCP_gen_pose(self.boundary_constraints,self.number_samples,solver=self.solver,robot_params=self.robot_params)
                self.counter = 0
                self.enter_recovery_mode = 1



if __name__ == '__main__':

    inv_node = invariants_traj_gen_node("talker")

    # This defines the rate at which the node should publish
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        inv_node.check_condition()
        if inv_node.condition == 0:
            inv_node.generate_trajectory()
        else:
            inv_node.reset_OCP()
        rate.sleep()