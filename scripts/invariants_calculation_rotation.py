#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Example of online invariants calculation
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
from math import pi
# from scipy.spatial.transform import Rotation as R
import invariants_py.rockit_calculate_vector_invariants_rotation as invariants_rotation_calculation
import std_msgs.msg
import helper_functions_ros
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import quaternion_matrix

class ROSInvariantsCalculationRotation:
    def __init__(self):

        rospy.init_node('ros_invariants_calculation_rotation', anonymous=True)
        self.update_rate = rospy.Rate(30)  # Set the ROS node update rate (Default: 20 Hz)
                
        # Initialize the start time
        self.starttime = rospy.get_time()

        # Create a ROS topic subscribers and publishers
        rospy.Subscriber('/pose_data', Pose, self.callback_pose)
        self.publisher_invariants_rotation = rospy.Publisher('/invariants_rotation_result', std_msgs.msg.Float32MultiArray, queue_size=10)   

        # Define parameters of the window of measurements
        self.window_nb_samples = 21 # number of samples in window for the given horizon length
        self.window_horizon_length = 2.0 # size of the window in time [sec]
        
        # Initialization window
        self.window_progress_step = self.window_horizon_length/self.window_nb_samples # time step between samples in window [sec]
        self.window_measured_rotations = np.zeros((self.window_nb_samples,3,3))
        self.progress_trigger = rospy.get_time()
        
        # Initialize invariants calculation problem
        self.invariant_rotation_calculator = invariants_rotation_calculation.OCP_calc_rot(window_len=self.window_nb_samples,rms_error_traj=4*pi/180,fatrop_solver=True)
           
    def build_time_window(self, new_rotation):
        # Check if enough progress has passed for the new measurement to be included in the window
        progress_time = rospy.get_time() # measure progress
        #print(progress_time)

        if progress_time > self.progress_trigger:
            #print("New measurement")
            
            # Update window (unless it is a duplicate value)
            if np.all(new_rotation != self.window_measured_rotations[-1]):
            
                self.window_measured_rotations[:-1] = self.window_measured_rotations[1:] # push all measurements one sample back
                self.window_measured_rotations[-1] = new_rotation # add new measurement as last sample

            else:
                print("Skipping new measurement because it is a duplicate")            
            
            # Set new time trigger
            self.progress_trigger = self.progress_trigger + self.window_progress_step     
            #print(self.progress_trigger)

            # Report the number of nonzero samples in the window
            number_window_samples = np.count_nonzero(self.window_measured_rotations[:, 0, 0])
            if number_window_samples != self.window_nb_samples:
                print(f"Filling window: {number_window_samples} out of {self.window_nb_samples}")
            #print(self.window_measured_positions)


    def callback_pose(self, pose_msg):
        # Callback function to process the received Pose message
        new_quaternion = np.array([pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w])
        new_rotation = quaternion_matrix(new_quaternion)[:3,:3]
        
        # Update window of measurements
        self.build_time_window(new_rotation)


    def run(self):

        # Function to be run in a loop at the specified update rate
        while not rospy.is_shutdown():
            
            # Check if window of measurements is not yet full
            if np.all(self.window_measured_rotations[0,0] == 0):
                invariants_rot = 0
            else:  
                
                # print('TIME UNTIL FIRST ROTATION CALCULATION: ', rospy.get_time()-self.starttime)

                # Call the function from your invariant calculator
                # print(self.window_measured_rotations)
                # rot_time = rospy.get_time()
                invariants_rot, traj_rot, mf_rot = self.invariant_rotation_calculator.calculate_invariants(self.window_measured_rotations, self.window_progress_step)
                # print('Time for rotation invariants calculation: ', rospy.get_time()-rot_time)
               
                invariants_rot_float32_array = helper_functions_ros.convert_nparray_to_Float32MultiArray(invariants_rot)

                self.publisher_invariants_rotation.publish(invariants_rot_float32_array)

            self.update_rate.sleep() # Sleep to maintain the specified update rate

if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        ros_invariants_rotation_node = ROSInvariantsCalculationRotation()
        
        # Run the loop of the node
        ros_invariants_rotation_node.run()
    except rospy.ROSInterruptException:
        pass