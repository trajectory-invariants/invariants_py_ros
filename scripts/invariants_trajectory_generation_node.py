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

class invariants_traj_gen_node:

    def __init__(self, name = "talker") -> None:

        rospy.init_node(name, anonymous=True)
        
        #Initialise variables 
        self.tf = np.zeros(7) # [x,y,z,qx,qy,qz,qw]
        self.progress = 0
        self.progress_fv = 0
        self.progress_offset_previous = 0
        self.progress_sum = 0
        self.progress_rot = 0
        self.progress_offset = 0
        self.cond = 1000
        self.counter = 0
        self.condition = 0
        self.dist_ratio = 1
        self.enter_recovery_mode = 0
        self.recovery_target = 100 * np.ones(7)
        self.R = np.eye(3)
        self.current_sample = 0
        self.factor_execution_speed = 1
        self.jointvel = np.zeros(7)
        self.R_w_tgt = np.eye(3)
        self.pos_w_tgt = np.zeros(3)
        self.bottle_pos_real = np.array([100,100,100]) # to understand if use real position measurements or simulated
        self.bottle_pos_sim = np.array([100,100,100])
        self.quat_demo = np.array([0.907466,-0.416623,0.0328278,-0.0430416]) # orientation quaternion (defined as [qw,qx,qy,qz]) of the final pose of the human demonstration
        self.radius = 0.0145 + 0.015 # radius of the bottle (stella :0.0145, FM: 0.021) + buffer of 0.015
        # self.alpha_deg = 0 # angle of approach direction around z-axis in degrees
        self.alpha = np.array([np.pi/2]) # angle of approach direction around z-axis in rad
        self.solver = "ipopt" # options "ipopt", "fatrop"

        rospy.Subscriber('sim_bottle_pos_pub', Float64MultiArray, self.callback_bottle_pos_sim)
        rospy.Subscriber('/pickit/objects_wrt_robot_frame', Float64MultiArray, self.callback_bottle_pos_real) # THIS IS EXAMPLE OF BOTTLE POSITION DETECTED BY SENSOR - NOT IMPLEMENTED CORRECTLY YET (update also in listener_node)
        rospy.Subscriber('tcp_pose', Float64MultiArray, self.callback_tf)
        rospy.Subscriber('progress', Float64MultiArray, self.callback_progress)
        rospy.Subscriber('progress_fv', Float64MultiArray, self.callback_progress_fv)
        rospy.Subscriber('progress_offset_previous', Float64MultiArray, self.callback_progress_offset_previous)
        rospy.Subscriber('condition_output', Float64MultiArray, self.callback_cond)
        rospy.Subscriber('tgt_frame_wrt_tgt', Float64MultiArray, self.callback_inst_tgt)
        rospy.Subscriber('jointvel', Float64MultiArray, self.callback_jointvel)
        rospy.Subscriber('factor_execution_speed', Float64MultiArray, self.callback_factorexecspeed)
        self.pub_node_output = rospy.Publisher('invgen_node_output', Float64MultiArray, queue_size=10, latch=True)

        if self.pos_w_tgt is None or self.R_w_tgt is None:
            print("Waiting for a target pose to be published on /target_pose_pub topic...")
            rospy.wait_for_message('/target_pose_pub', Pose)
        if self.pos_w_tcp is None or self.R_w_tcp is None:
            print("Waiting for a start pose to be published on /current_pose_pub topic...")
            rospy.wait_for_message('/current_pose_pub', Pose)
            rospy.wait_for_message('/progress_partial',std_msgs.msg.Float64)
           
        # current_progress = 0.6 # TAKE THIS FROM ETASLCORE
        self.number_samples = 100

        # new constraints       
        alpha = 0
        rotate = R.from_euler('z', alpha, degrees=True)
        # R_w_tgt =  orthonormalize(rotate.as_matrix() @ self.Robj[-1])
        FSt_end = orthonormalize(rotate.as_matrix() @ self.demo_FSt[-1])

        # # Linear initialization
        R_obj_init = interpR(np.linspace(0, 1, len(self.demo_pos)), [0,1], np.array([self.R_w_tcp, self.R_w_tgt]))
        # # R_r_init = interpR(np.linspace(0, 1, len(optim_calc_results.FSr_frames)), [0,1], np.array([FSr_start, FSr_end]))

        R_r_init, R_r_init_array, invars_init = FSr_init(self.R_w_tcp, self.R_w_tgt)

        self.boundary_constraints = {
            "position": {
                "initial": self.pos_w_tcp, #self.demo_pos[0], #np.array([0.3056, 0.0635, 0.441]), #
                "final": self.pos_w_tgt # self.demo_pos[-1] #self.p_obj[-1] #np.array([0.827,0.144,0.552]) 
            },
            "orientation": {
                "initial": self.demo_Robj[0], #self.R_w_tcp, # np.eye(3), # # THIS NEEDS TO BE TAKEN FROM THE DEMO BECAUSE THE STARTING ORIENTATION OF TCP IS NOT THE SAME AS DEMO
                "final": self.R_w_tgt # self.demo_Robj[-1] #rotate_x(np.pi/16) #
            },
            "moving-frame": {
                "translational": {
                    "initial": orthonormalize(self.demo_FSt[0]), #np.eye(3), #
                    "final": FSt_end #rotate_x(np.pi/8) #
                },
                "rotational": {
                    "initial": orthonormalize(self.demo_FSr[0]), #R_r_init, #np.eye(3), #
                    "final": orthonormalize(self.demo_FSr[-1]) # R_r_init #rotate_x(np.pi/8) #
                }
            },
        }

        robot_params = {
            "urdf_file_name": None, # use None if do not want to include robot model
            "q_init": np.array([-np.pi, -2.27, 2.27, -np.pi/2, -np.pi/2, np.pi/4]), # Initial joint values
            "tip": 'TCP_frame' # Name of the robot tip (if empty standard 'tool0' is used)
            # "joint_number": 6, # Number of joints (if empty it is automatically taken from urdf file)
            # "q_lim": [2*pi, 2*pi, pi, 2*pi, 2*pi, 2*pi], # Join limits (if empty it is automatically taken from urdf file)
            # "root": 'world', # Name of the robot root (if empty it is automatically taken from urdf file)
        }
        # Franka q_init: [-0.03594372660758203, -1.7589981400814376, -1.9254516473826875, -1.9436872720551073, -0.3177960551049983, 1.981110099809029,-1.0311961190588514]
        
        self.FS_online_generation_problem = OCP_gen.OCP_gen_pose(self.boundary_constraints,self.number_samples,solver=self.solver,robot_params=robot_params)
        
        # Resample model invariants to desired number of self.number_samples samples
        current_sample = round(self.progress*len(self.invariant_model))
        arclength_n = np.linspace(0,1,self.number_samples)
        spline_invariant_model = sh.create_spline_model(self.invariant_model[:,0], self.invariant_model[:,1:])
        progress_values = np.linspace(self.progress,self.invariant_model[-1,0],self.number_samples)
        model_invariants,progress_step = sh.interpolate_invariants(spline_invariant_model, progress_values)

        # Define OCP weights
        self.weights_params = {
            "w_invars": np.array([1, 1, 1, 5*10**1, 1.0, 1.0]),
            "w_high_start": 60,
            "w_high_end": self.number_samples,
            "w_high_invars": 10*np.array([1, 1, 1, 5*10**1, 1.0, 1.0]),
            "w_high_active": 0
        }

        self.initial_values = {
            "trajectory": {
                "position": self.demo_pos,
                "orientation": self.demo_Robj
            },
            "moving-frame": {
                "translational": self.demo_FSt,
                "rotational": self.demo_FSr #R_r_init_array,
            },
            "invariants": model_invariants,
            "joint-values": robot_params["q_init"] if robot_params["urdf_file_name"] is not None else {}
        }

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

    def callback_bottle_pos_sim(self,data):

        self.bottle_pos_sim = data.data

    def callback_bottle_pos_real(self,data):

        self.bottle_pos_real = data.data

    def callback_progress(self,data):

        self.progress = data.data[0]

    def callback_progress_fv(self,data):

        self.progress_fv = data.data[0]
    
    def callback_progress_offset_previous(self,data):

        self.progress_offset_previous = data.data[0]
        
    def callback_tf(self,data):

        self.tf = data.data
    
    def callback_cond(self,data):

        self.cond = data.data[0]

    def callback_inst_tgt(self,data):

        self.inst_tgt = data.data

    def callback_jointvel(self,data):

        self.jointvel = data.data

    def callback_factorexecspeed(self,data):

        self.factor_execution_speed = data.data[0]

    def check_condition(self):
        # This function checks that the robot position is not more than 0.01 m away from the model trajectory evaluated at the progress given by etasl.
        if not self.tf[0] == 0:

            # Automatically detect if there is a real bottle position measurement or if the simulated one should be used
            if all(i == 100 for i in self.bottle_pos_real):
                self.pos_w_bottle = self.bottle_pos_sim
            else:
                self.pos_w_bottle = self.bottle_pos_real
        
            if self.enter_recovery_mode == 0:
                self.previous_model_trajectory = np.array([(self.R_w_tgt @ self.model_trajectory[i,:3] + self.pos_w_tgt) for i in range(100)])

            # Define the target pose, by adding the bottle radius distance to the bottle position measurement and a small offset to ensure no contact between opener and bottle
            # Rotate the target pose around the z-axis by the alpha_deg angle (currently fixed), defining the desired approach direction
            # FOR BOTTLE APPROACH
            # self.pos_w_tgt = np.array([self.pos_w_bottle[0]-self.radius*np.sin(self.alpha[0]),self.pos_w_bottle[1]+self.radius*np.cos(self.alpha[0]),self.pos_w_bottle[2]])
            # self.R_w_tgt = Rot.from_euler('z', self.alpha[0]).as_matrix() @ Rot.from_quat([self.quat_demo[1],self.quat_demo[2],self.quat_demo[3],self.quat_demo[0]]).as_matrix()
            # FOR CRATE FILLING
            self.pos_w_tgt = self.pos_w_bottle # np.array([self.pos_w_bottle[0]-self.radius*np.sin(self.alpha[0]),self.pos_w_bottle[1]+self.radius*np.cos(self.alpha[0]),self.pos_w_bottle[2]])
            self.R_w_tgt = np.eye(3)
            quat_w_tgt = rot2quat(self.R_w_tgt.reshape(1,3,3)).reshape(4) # [qx,qy,qz,qw]
            self.pose_w_tgt = np.hstack([self.pos_w_tgt,np.array([quat_w_tgt[3],quat_w_tgt[0],quat_w_tgt[1],quat_w_tgt[2]])])

            # Interpolate the model trajectory at the current progress and check if the robot position is within 0.01 m of the model trajectory
            interp_samples = 100
            interp_model_trajectory = np.zeros((3,interp_samples))
            for i in range(3):
                interp_model_trajectory[i] = np.interp(np.linspace(0,1,num=interp_samples), np.array(self.m["dof"][0]["s"]), self.pos_w_model[:,i])
            dist_robot_to_model = np.array([np.linalg.norm(self.tf[:3] - interp_model_trajectory[i,:]) for i in range(interp_samples)])
            if any(dist_robot_to_model  < 0.01): # it checks if the robot is within 1cm away of the interp_samples of the model trajectory
                self.condition = 1
                self.current_sample = np.argmin(dist_robot_to_model)
            else:
                self.condition = 0

    def generate_trajectory(self):
        if not self.tf[0] == 0:# and self.condition == 0:

            # Take the current robot pose (coming from TF topic) and convert the quaternion orientation into a rotation matrix
            # tcp_pos = np.array([self.tf[0],self.tf[1],self.tf[2]])
            # R_tcp = quat2rot(np.hstack([self.tf[3], self.tf[4], self.tf[5],self.tf[6]]).reshape(1,4)).reshape(3,3)
            delay_sample = 2+int(self.factor_execution_speed/2)# int(2*self.factor_execution_speed) # (-self.enter_recovery_mode+1) # (-2*self.enter_recovery_mode+1) * 
            if all(self.jointvel[i] < 1e-4 for i in range(6)) or self.enter_recovery_mode == 1:
                tcp_pos = np.array([self.tf[0],self.tf[1],self.tf[2]])
                R_tcp = quat2rot(np.hstack([self.tf[3], self.tf[4], self.tf[5],self.tf[6]]).reshape(1,4)).reshape(3,3)
                tcp_pose = self.tf #np.array([0,0,0,0,0,0,1])
            else:
                if self.current_sample+ delay_sample>= len(self.previous_model_trajectory):
                    tcp_pos = self.previous_model_trajectory[self.current_sample] #np.array([self.inst_tgt[0],self.tf[1],self.inst_tgt[2]])
                else:
                    tcp_pos = self.previous_model_trajectory[self.current_sample+delay_sample] #np.array([self.inst_tgt[0],self.tf[1],self.inst_tgt[2]])
                R_tcp = quat2rot(np.hstack([self.tf[3], self.tf[4], self.tf[5],self.tf[6]]).reshape(1,4)).reshape(3,3)
                tcp_pose = np.hstack([tcp_pos,self.tf[3:]]) # QUICK FIX

            if self.counter == 0:
                self.previous_target = self.pos_w_tgt
            prev_dist_to_target = np.linalg.norm(self.previous_target)
            if self.condition == 0:
                # Calculate s_prior by comparing the distance of the current robot pose to the previous target and to the current target
                dist_to_target = np.linalg.norm(self.pos_w_tgt)
                self.progress_sum = self.progress_fv + self.progress_offset_previous
                self.dist_ratio = dist_to_target / prev_dist_to_target
                s_prior = self.progress_sum + self.progress_sum * (1 - self.dist_ratio)
                if s_prior < 0:
                    s_prior = 0
                elif s_prior > 0.75:
                    s_prior = 0.75
                
                # Resample model invariants to desired number of self.number_samples samples
                current_sample = round(self.progress*len(self.invariant_model))
                spline_invariant_model = sh.create_spline_model(self.invariant_model[:,0], self.invariant_model[:,1:])
                progress_values = np.linspace(self.progress,self.invariant_model[-1,0],self.number_samples)
                model_invariants,progress_step = sh.interpolate_invariants(spline_invariant_model, progress_values)
                
                # Specify the boundary constraints
                self.boundary_constraints["position"]["initial"] = self.pos_w_tcp # self.demo_pos[0] # np.array([0.3056, 0.0635, 0.441]) #
                self.boundary_constraints["position"]["final"] = self.pos_w_target # self.demo_pos[-1] #self.p_obj[-1] # np.array([0.827,0.144,0.552]) #np.array([0.69,0.244,0.4]) #
                self.boundary_constraints["orientation"]["initial"] = self.demo_Robj[current_sample]
                self.boundary_constraints["moving-frame"]["translational"]["initial"] = orthonormalize(self.demo_FSt[current_sample])
                self.boundary_constraints["moving-frame"]["rotational"]["initial"] = orthonormalize(self.demo_FSr[current_sample])

                # Generate trajectory
                invariants, pos, R_obj, R_t, R_r, joint_values = self.FS_online_generation_problem.generate_trajectory(model_invariants,self.boundary_constraints,progress_step,self.weights_params,self.initial_values)

                # Publish trajectory
                trajectory = Trajectory()
                quaternion = rot2quat(R_obj)
                for i in range(invariants.shape[0]):
                    pose = Pose()
                    pose.position.x = pos[i,0]
                    pose.position.y = pos[i,1]
                    pose.position.z = pos[i,2]
                    pose.orientation.x = quaternion[i,0]
                    pose.orientation.y = quaternion[i,1]
                    pose.orientation.z = quaternion[i,2]
                    pose.orientation.w = quaternion[i,3]
                    trajectory.poses.append(pose)
                self.publisher_trajectory.publish(trajectory)

                # Add points to the marker
                self.marker.points = []
                for point in pos:  # Assuming traj is a numpy array of shape (self.number_samples, 3)
                    p = Point()
                    p.x = point[0]
                    p.y = point[1]
                    p.z = point[2]
                    self.marker.points.append(p)

                # Publish the Marker
                self.marker.header.stamp = rospy.Time.now() # add timestamp
                self.publisher_traj_gen_marker.publish(self.marker)

                self.counter += 1

                # if dist_err > 0.0065:
                #     self.enter_recovery_mode = 1
                #     print("In recovery mode")
                # else:
                #     self.enter_recovery_mode = 0
                #     self.published_latent_variables = self.latent_variables.copy()
                #     # Calculate the new progress offset that results into the casadi_progress
                #     self.progress_offset = self.progress_offset_previous + s_prior - self.progress_sum

                # print(self.alpha,self.counter)

            if self.enter_recovery_mode == 0:
                self.recovery_target = self.pose_w_tgt.copy()
            else:
                self.pose_w_tgt = self.recovery_target.copy()

            self.previous_target = self.pos_w_tgt

    def publish(self):
        # This function publishes all the output topics of the node.
        # It was created to split publishing from calculations and reduce the risk of having etasl running in the middle of a node loop.
        if not self.tf[0] == 0:
            self.pub_ext_condition.publish(self.condition)

            # If the condition is 0 (meaning that the robot pose is not on the model trajectory), publish the latent variables and the progress offset.
            outputs = np.hstack((self.published_latent_variables, self.progress_offset, self.pose_w_tgt, self.dist_ratio, self.enter_recovery_mode,self.alpha))

            self.pub_modeltraj_markers.publish(self.marker_msg)
            
            self.pub_node_output.publish(Float64MultiArray(data=outputs))

if __name__ == '__main__':

    inv_node = invariants_traj_gen_node("talker")

    inv_node.init_OCP()

    # This defines the rate at which the node should publish
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        inv_node.check_condition()
        inv_node.generate_trajectory()
        inv_node.publish()
        rate.sleep()