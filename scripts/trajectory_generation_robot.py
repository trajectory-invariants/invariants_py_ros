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
                "initial": np.array([0,0,0]),
                "final": np.array([0.827,0.7144,0.552])
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

        robot_params = {
            "urdf_file_name": 'ur10.urdf', # use None if do not want to include robot model
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
            boundary_constraints["position"]["initial"] = [0, 0, 0]
            boundary_constraints["position"]["final"] = self.target_position

            # Generate trajectory
            invariants, pos, R_obj, R_t, R_r, time, joint_values = FS_online_generation_problem.generate_trajectory(model_invariants,boundary_constraints,progress_step,weights_params)

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
        model_filename = rospack.get_path("invariants_py_ros")+"/data/"+"sinus_invariants.csv"
        
        ros_traj_gen_node = ROSInvariantTrajectoryGeneration(model_filename)
        
        # Run the loop of the node
        ros_traj_gen_node.run()
    except rospy.ROSInterruptException:
        pass