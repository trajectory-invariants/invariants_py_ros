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
from invariants_py.kinematics.orientation_kinematics import rotate_x
import matplotlib.pyplot as plt
import invariants_py.plotting_functions.plotters as pl

class ROSInvariantTrajectoryGeneration:
    def __init__(self, invariant_model_location):
        rospy.init_node('ros_trajectory_generation', anonymous=True)
        self.update_rate = rospy.Rate(20)  # Set the ROS node update rate (Default: 20 Hz)
        
        # Create a ROS topic subscribers and publishers
        rospy.Subscriber('/target_pose', Pose, self.callback_target_pose)
        self.publisher_traj_gen = rospy.Publisher('/trajectory', Marker, queue_size=10)
        self.publisher_traj_meas = rospy.Publisher('/target_pose_marker', Marker, queue_size=10)
        self.joint_values = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Initialize invariant trajectory generation problem
        self.invariant_model = rw.read_invariants_from_csv(invariant_model_location)
        self.target_position = [1.1,0.0,0.0]

    def callback_target_pose(self, pose_msg):
        # Callback function to process the received Pose message
        self.target_position = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        
        # Create a Marker message for the sphere
        marker = Marker()
        marker.header = std_msgs.msg.Header()
        marker.header.stamp = rospy.Time.now() # add timestamp
        marker.header.frame_id = 'world' # add frame_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        # Set marker properties
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
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

    def run(self):

        current_progress = 0
        number_samples = 100
        arclength_n = np.linspace(0,1,number_samples)

        progress_values = np.linspace(current_progress, arclength_n[-1], number_samples)

        # new constraints
        # retrieving data from file.
        # p_obj = np.loadtxt("beer_pobj.csv", delimiter=",")
        # loaded_Robj = np.loadtxt("beer_Robj.csv", delimiter=",")
        # Robj = loaded_Robj.reshape(loaded_Robj.shape[0], loaded_Robj.shape[1] // 3, 3)
        # loaded_FSt = np.loadtxt("beer_FSt.csv", delimiter=",")
        # FSt = loaded_FSt.reshape(loaded_FSt.shape[0], loaded_FSt.shape[1] // 3, 3)
        # loaded_FSr = np.loadtxt("beer_FSt.csv", delimiter=",")
        # FSr = loaded_FSr.reshape(loaded_FSr.shape[0], loaded_FSr.shape[1] // 3, 3)
        
        # current_index = 0
        # p_obj_start = optim_calc_results.Obj_pos[current_index]
        # R_obj_start = orthonormalize(optim_calc_results.Obj_frames[current_index])
        # FSt_start = orthonormalize(optim_calc_results.FSt_frames[current_index])
        # # FSr_start = orthonormalize(optim_calc_results.FSr_frames[current_index])
        # p_obj_end = optim_calc_results.Obj_pos[-1] + np.array([1.15,-0.1,-0.1]) #np.array([0.827,0.7144,0.552]) #
        # alpha = 0
        # rotate = R.from_euler('z', alpha, degrees=True)
        # R_obj_end =  orthonormalize(rotate.as_matrix() @ optim_calc_results.Obj_frames[-1])
        # FSt_end = orthonormalize(rotate.as_matrix() @ optim_calc_results.FSt_frames[-1])
        # # FSr_end = orthonormalize(optim_calc_results.FSr_frames[-1])

        # # define new class for OCP results
        # optim_gen_results = OCP_results(FSt_frames = [], FSr_frames = [], Obj_pos = [], Obj_frames = [], invariants = np.zeros((number_samples,6)))

        # # Linear initialization
        # R_obj_init = interpR(np.linspace(0, 1, len(optim_calc_results.Obj_frames)), [0,1], np.array([R_obj_start, R_obj_end]))
        # # R_r_init = interpR(np.linspace(0, 1, len(optim_calc_results.FSr_frames)), [0,1], np.array([FSr_start, FSr_end]))

        # R_r_init, R_r_init_array, invars_init = FSr_init(R_obj_start, R_obj_end)

        boundary_constraints = {
            "position": {
                "initial": np.array([0.3056, 0.0635, 0.441]), #p_obj[0], #
                "final": np.array([0.69,0.244,0.4]) #p_obj[-1] #np.array([0.827,0.144,0.552]) 
            },
            "orientation": {
                "initial": np.eye(3), #orthonormalize(Robj[0]),
                "final": rotate_x(np.pi/8) #orthonormalize(Robj[-1])
            },
            "moving-frame": {
                "translational": {
                    "initial": np.eye(3), #orthonormalize(FSt[0]),
                    "final": rotate_x(np.pi/8) #orthonormalize(FSt[-1])
                },
                "rotational": {
                    "initial": np.eye(3), #orthonormalize(FSr[0]),
                    "final": rotate_x(np.pi/8) #orthonormalize(FSr[-1])
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

        FS_online_generation_problem = OCP_gen.OCP_gen_pose(boundary_constraints,number_samples,fatrop_solver=True,robot_params=robot_params)

        # Resample model invariants to desired number of number_samples samples
        spline_invariant_model = sh.create_spline_model(self.invariant_model[:,0], self.invariant_model[:,1:])
        progress_values = np.linspace(self.invariant_model[0,0],self.invariant_model[-1,0],number_samples)
        model_invariants,progress_step = sh.interpolate_invariants(spline_invariant_model, progress_values)

        # Define OCP weights
        weights_params = {
            "w_invars": np.array([1, 1, 1, 5*10**1, 1.0, 1.0]),
            "w_high_start": 60,
            "w_high_end": number_samples,
            "w_high_invars": 10*np.array([1, 1, 1, 5*10**1, 1.0, 1.0]),
            "w_high_active": 0
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

            # Specify the boundary constraints
            boundary_constraints["position"]["initial"] = np.array([0.3056, 0.0635, 0.441]) #p_obj[0]#
            boundary_constraints["position"]["final"] = self.target_position #np.array([0.827,0.144,0.552]) #np.array([0.69,0.244,0.4]) #p_obj[-1]#
            print(self.target_position)
            

            # Generate trajectory
            invariants, pos, R_obj, R_t, R_r, time, joint_values = FS_online_generation_problem.generate_trajectory(model_invariants,boundary_constraints,progress_step,weights_params)

            # fig = plt.figure(figsize=(14,8))
            # ax = fig.add_subplot(111, projection='3d')
            # ax.plot(p_obj[:,0],p_obj[:,1],p_obj[:,2],'b')
            # ax.plot(pos[:,0],pos[:,1],pos[:,2],'r')
            # pl.plot_invariants(model_invariants, invariants, arclength_n, progress_values)
            # indx_online = np.trunc(np.linspace(0,len(pos)-1,5))
            # indx_online = indx_online.astype(int)
            # for i in indx_online:
            #     pl.plot_3d_frame(p_obj[i,:],Robj[i,:,:],1,0.01,['red','green','blue'],ax)
            #     pl.plot_3d_frame(pos[i,:],R_obj[i,:,:],1,0.01,['red','green','blue'],ax)
            # plt.show()

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
            self.publisher_traj_gen.publish(marker)

            # Move robot
            self.joint_values = joint_values
            q = JointState()
            self.update_rate.sleep() # Sleep to maintain the specified update rate

if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        rospack = rospkg.RosPack()
        model_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_inv.csv"
        
        ros_traj_gen_node = ROSInvariantTrajectoryGeneration(model_filename)
        
        # Run the loop of the node
        ros_traj_gen_node.run()
    except rospy.ROSInterruptException:
        pass