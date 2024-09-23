#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Example of online invariants calculation
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
import invariants_py.calculate_invariants.rockit_calculate_vector_invariants_position_mj as invariants_calculation
import std_msgs.msg
import helper_functions_ros
from nav_msgs.msg import Path
import rospkg
from invariants_py import data_handler as rw
from invariants_py.generate_trajectory import rockit_generate_pose_traj_from_vector_invars as OCP_gen
import invariants_py.spline_handler as sh
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from invariants_py.kinematics.rigidbody_kinematics import orthonormalize_rotation as orthonormalize
import matplotlib.pyplot as plt
import invariants_py.plotting_functions.plotters as pl
from scipy.spatial.transform import Rotation as R
from invariants_py.initialization import initial_trajectory_movingframe_rotation as FSr_init
from invariants_py.reparameterization import interpR
import yourdfpy as urdf
from invariants_py.kinematics.orientation_kinematics import rot2quat
from ros_spline_fitting_trajectory.msg import Trajectory

class ROSInvariantTrajectoryGeneration:
    def __init__(self, invariant_model_location,pos_location,R_location,FSt_location,FSr_location):
        rospy.init_node('ros_trajectory_generation', anonymous=True)
        self.update_rate = rospy.Rate(20)  # Set the ROS node update rate (Default: 20 Hz)
        
        # Create a ROS topic subscribers and publishers
        rospy.Subscriber('/target_pose_pub', Pose, self.callback_target_pose)
        rospy.Subscriber('/current_pose_pub', Pose, self.callback_start_pose)
        rospy.Subscriber('/progress_partial', std_msgs.msg.Float64, self.callback_progress)
        rospy.Subscriber('/des_ee', std_msgs.msg.Float64MultiArray, self.callback_des_ee)
        self.publisher_traj_gen_marker = rospy.Publisher('/trajectory_marker', Marker, queue_size=10)
        self.publisher_traj_meas = rospy.Publisher('/target_pose_marker', Marker, queue_size=10)
        self.publisher_joint_values = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.publisher_trajectory = rospy.Publisher('/trajectory_pub', Trajectory, queue_size=10)
        self.publisher_desee_marker = rospy.Publisher('/des_ee_marker', Marker, queue_size=10)

        # Initialize invariant trajectory generation problem
        self.invariant_model = rw.read_invariants_from_csv(invariant_model_location)
        self.p_start = None
        self.p_end = None
        self.Robj_start = None
        self.Robj_end = None

        self.demo_pos = np.loadtxt(pos_location, delimiter=",")
        loaded_Robj = np.loadtxt(R_location, delimiter=",")
        self.demo_Robj = loaded_Robj.reshape(loaded_Robj.shape[0], loaded_Robj.shape[1] // 3, 3)
        loaded_FSt = np.loadtxt(FSt_location, delimiter=",")
        self.FSt = loaded_FSt.reshape(loaded_FSt.shape[0], loaded_FSt.shape[1] // 3, 3)
        loaded_FSr = np.loadtxt(FSr_location, delimiter=",")
        self.FSr = loaded_FSr.reshape(loaded_FSr.shape[0], loaded_FSr.shape[1] // 3, 3)

        self.previous_target = [1000,1000,1000]

        self.des_ee = [0,0,0]
        self.counter = 0

    def callback_target_pose(self, pose_msg):
        # Callback function to process the received Pose message
        self.p_end = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        self.quat_obj_end = [pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w]
        self.Robj_end = R.from_quat(self.quat_obj_end).as_matrix()
        
        # Create a Marker message for the sphere
        marker = Marker()
        marker.header = std_msgs.msg.Header()
        marker.header.stamp = rospy.Time.now() # add timestamp
        marker.header.frame_id = 'world' # add frame_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        # Set marker properties
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        # Set marker position
        marker.pose.position.x = pose_msg.position.x
        marker.pose.position.y = pose_msg.position.y
        marker.pose.position.z = pose_msg.position.z
        # Publish the Marker
        self.publisher_traj_meas.publish(marker)

    def callback_start_pose(self, start_pose):
        # Callback function to process the received Pose message
        self.p_start = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
        quat_obj_start = np.array([start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w])
        self.Robj_start = R.from_quat(quat_obj_start).as_matrix()

    def callback_progress(self, progress_msg):
        # Callback function to process the received Float64 message
        self.current_progress = progress_msg.data

    def callback_des_ee(self, des_ee_msg):
        # Callback function to process the received Float64MultiArray message
        self.des_ee = des_ee_msg.data

    def run(self):
        if self.p_end is None or self.Robj_end is None:
            print("Waiting for a target pose to be published on /target_pose_pub topic...")
            rospy.wait_for_message('/target_pose_pub', Pose)
        if self.p_start is None or self.Robj_start is None:
            print("Waiting for a start pose to be published on /current_pose_pub topic...")
            rospy.wait_for_message('/current_pose_pub', Pose)
        # self.p_start = np.array([-2.622134222757627892e-02, -4.767094578195006371e-01, 0.08945761571123636569])
        # self.Robj_start = R.from_quat(np.array([-7.111921398743396017e-01,-8.563669717168140294e-03,-7.027414404159517680e-01,-1.693728620663667583e-02])).as_matrix()
        if self.Robj_start is not None and self.Robj_end is not None and self.p_start is not None and self.p_end is not None:
            
            # current_progress = 0.6 # TAKE THIS FROM ETASLCORE
            number_samples = 100

            # new constraints       
            alpha = 0
            rotate = R.from_euler('z', alpha, degrees=True)
            # Robj_end =  orthonormalize(rotate.as_matrix() @ self.Robj[-1])
            FSt_end = orthonormalize(rotate.as_matrix() @ self.FSt[-1])

            # # Linear initialization
            R_obj_init = interpR(np.linspace(0, 1, len(self.demo_pos)), [0,1], np.array([self.Robj_start, self.Robj_end]))
            # # R_r_init = interpR(np.linspace(0, 1, len(optim_calc_results.FSr_frames)), [0,1], np.array([FSr_start, FSr_end]))

            R_r_init, R_r_init_array, invars_init = FSr_init(self.Robj_start, self.Robj_end)

            boundary_constraints = {
                "position": {
                    "initial": self.p_start, #self.demo_pos[0], #np.array([0.3056, 0.0635, 0.441]), #
                    "final": self.p_end # self.demo_pos[-1] #self.p_obj[-1] #np.array([0.827,0.144,0.552]) 
                },
                "orientation": {
                    "initial": self.demo_Robj[0], #self.Robj_start, # np.eye(3), # # THIS NEEDS TO BE TAKEN FROM THE DEMO BECAUSE THE STARTING ORIENTATION OF TCP IS NOT THE SAME AS DEMO
                    "final": self.Robj_end # self.demo_Robj[-1] #rotate_x(np.pi/16) #
                },
                "moving-frame": {
                    "translational": {
                        "initial": orthonormalize(self.FSt[0]), #np.eye(3), #
                        "final": FSt_end #rotate_x(np.pi/8) #
                    },
                    "rotational": {
                        "initial": orthonormalize(self.FSr[0]), #R_r_init, #np.eye(3), #
                        "final": orthonormalize(self.FSr[-1]) # R_r_init #rotate_x(np.pi/8) #
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
            
            FS_online_generation_problem = OCP_gen.OCP_gen_pose(boundary_constraints,number_samples,fatrop_solver=True,robot_params=robot_params)
            
            # Resample model invariants to desired number of number_samples samples
            current_sample = round(self.current_progress*len(self.invariant_model))
            arclength_n = np.linspace(0,1,number_samples)
            spline_invariant_model = sh.create_spline_model(self.invariant_model[:,0], self.invariant_model[:,1:])
            progress_values = np.linspace(self.current_progress,self.invariant_model[-1,0],number_samples)
            model_invariants,progress_step = sh.interpolate_invariants(spline_invariant_model, progress_values)

            # Define OCP weights
            weights_params = {
                "w_invars": np.array([1, 1, 1, 5*10**1, 1.0, 1.0]),
                "w_high_start": 60,
                "w_high_end": number_samples,
                "w_high_invars": 10*np.array([1, 1, 1, 5*10**1, 1.0, 1.0]),
                "w_high_active": 0
            }

            initial_values = {
                "trajectory": {
                    "position": self.demo_pos,
                    "orientation": self.demo_Robj
                },
                "moving-frame": {
                    "translational": self.FSt,
                    "rotational": self.FSr #R_r_init_array,
                },
                "invariants": model_invariants,
                "joint-values": robot_params["q_init"] if robot_params["urdf_file_name"] is not None else {}
            }

            # Create the Marker message
            marker = Marker()
            marker.header = std_msgs.msg.Header()
            marker.header.frame_id = "world"
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD

            # Set marker properties
            marker.scale.x = 0.01
            marker.color.a = 1.0
            marker.color.r = 1.0

            while not rospy.is_shutdown():

                # Generate a new trajectory only if the target has changed
                if np.linalg.norm(np.array(self.p_end) - np.array(self.previous_target)) > 1E-6:

                    self.previous_target = self.p_end
                    
                    # Resample model invariants to desired number of number_samples samples
                    current_sample = round(self.current_progress*len(self.invariant_model))
                    arclength_n = np.linspace(0,1,number_samples)
                    spline_invariant_model = sh.create_spline_model(self.invariant_model[:,0], self.invariant_model[:,1:])
                    progress_values = np.linspace(self.current_progress,self.invariant_model[-1,0],number_samples)
                    model_invariants,progress_step = sh.interpolate_invariants(spline_invariant_model, progress_values)
                    
                    # Specify the boundary constraints
                    boundary_constraints["position"]["initial"] = self.p_start # self.demo_pos[0] # np.array([0.3056, 0.0635, 0.441]) #
                    boundary_constraints["position"]["final"] = self.p_end # self.demo_pos[-1] #self.p_obj[-1] # np.array([0.827,0.144,0.552]) #np.array([0.69,0.244,0.4]) #
                    boundary_constraints["orientation"]["initial"] = self.demo_Robj[current_sample]
                    boundary_constraints["moving-frame"]["translational"]["initial"] = orthonormalize(self.FSt[current_sample])
                    boundary_constraints["moving-frame"]["rotational"]["initial"] = orthonormalize(self.FSr[current_sample])

                    # Generate trajectory
                    invariants, pos, R_obj, R_t, R_r, time, joint_values = FS_online_generation_problem.generate_trajectory(model_invariants,boundary_constraints,progress_step,weights_params,initial_values)

                    # fig = plt.figure(figsize=(14,8))
                    # ax = fig.add_subplot(111, projection='3d')
                    # ax.plot(self.demo_pos[:,0],self.demo_pos[:,1],self.demo_pos[:,2],'b')
                    # ax.plot(pos[:,0],pos[:,1],pos[:,2],'r')
                    # pl.plot_invariants(model_invariants, invariants, arclength_n, progress_values)
                    # indx_online = np.trunc(np.linspace(0,len(pos)-1,5))
                    # indx_online = indx_online.astype(int)
                    # for i in indx_online:
                    #     pl.plot_3d_frame(self.demo_pos[i,:],self.demo_Robj[i,:,:],1,0.01,['red','green','blue'],ax)
                    #     pl.plot_3d_frame(pos[i,:],R_obj[i,:,:],1,0.01,['red','green','blue'],ax)
                    # plt.show()

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
                    marker.points = []
                    for point in pos:  # Assuming traj is a numpy array of shape (number_samples, 3)
                        p = Point()
                        p.x = point[0]
                        p.y = point[1]
                        p.z = point[2]
                        marker.points.append(p)

                    # Publish the Marker
                    marker.header.stamp = rospy.Time.now() # add timestamp
                    self.publisher_traj_gen_marker.publish(marker)

                    # Move robot
                    if robot_params["urdf_file_name"] is not None:
                        path_to_urdf = rw.find_robot_path(robot_params["urdf_file_name"])
                        robot = urdf.URDF.load(path_to_urdf)
                        joints = JointState()
                        joints.name = [robot._actuated_joints[i].name for i in range(robot.num_actuated_joints)]
                        for i in range(invariants.shape[0]):
                            joints.position = joint_values[i]
                            joints.header.stamp = rospy.Time.now()
                            # print(joint_values[i])
                            self.publisher_joint_values.publish(joints)
                            rospy.sleep(0.1)

                marker_msg = Marker()
                marker_msg.header.frame_id = "world"
                marker_msg.header.stamp = rospy.Time.now()
                marker_msg.type = marker_msg.CUBE
                # marker_msg.action = marker_msg.ADD
                marker_msg.lifetime = rospy.Duration(30)
                marker_msg.id = self.counter
                self.counter += 1
                # marker scale
                marker_msg.scale.x = 0.008
                marker_msg.scale.y = 0.008
                marker_msg.scale.z = 0.008
                # marker color
                marker_msg.color.a = 1.0
                marker_msg.color.r = 0.0
                marker_msg.color.g = 0.0
                marker_msg.color.b = 1.0
                # marker orientaiton
                marker_msg.pose.orientation.x = 0.0
                marker_msg.pose.orientation.y = 0.0
                marker_msg.pose.orientation.z = 0.0
                marker_msg.pose.orientation.w = 1.0
                # add all the points to the marker message
                marker_msg.pose.position.x = self.des_ee[0]
                marker_msg.pose.position.y = self.des_ee[1]
                marker_msg.pose.position.z = self.des_ee[2]
                # add the marker to the publisher
                self.publisher_desee_marker.publish(marker_msg)

                self.update_rate.sleep() # Sleep to maintain the specified update rate

if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        rospack = rospkg.RosPack()
        # model_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_inv.csv"
        # pos_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_pobj.csv"
        # # R_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_Robj.csv"
        # FSt_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_FSt.csv"
        # FSr_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_FSr.csv"

        model_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_inv.csv"
        pos_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_pobj.csv"
        R_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_Robj.csv"
        FSt_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_FSt.csv"
        FSr_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_FSr.csv"
        
        ros_traj_gen_node = ROSInvariantTrajectoryGeneration(model_filename,pos_filename,R_filename,FSt_filename,FSr_filename)
        
        # Run the loop of the node
        ros_traj_gen_node.run()
    except rospy.ROSInterruptException:
        pass