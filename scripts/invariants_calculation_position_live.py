#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Example of online invariants calculation
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
import invariants_py.rockit_calculate_vector_invariants_position as invariants_position_calculation
import std_msgs.msg
import helper_functions_ros
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_matrix

class ROSInvariantsCalculationPositionLive:
    def __init__(self):

        rospy.init_node('ros_invariants_calculation_position_live', anonymous=True)
        self.update_rate = rospy.Rate(30)  # Set the ROS node update rate (Default: 20 Hz)
                
        # Initialize the start time
        self.starttime = rospy.get_time()

        # Create a ROS topic subscribers and publishers
        rospy.Subscriber('/vive_data', Pose, self.callback_vive_data)

        # Define parameters of the window of measurements
        self.window_nb_samples = 21 # number of samples in window for the given horizon length
        self.window_horizon_length = 2.0 # size of the window in time [sec]
        
        # Initialization window
        self.window_progress_step = self.window_horizon_length/self.window_nb_samples # time step between samples in window [sec]
        self.window_measured_positions = np.zeros((self.window_nb_samples,3))
        self.progress_trigger = rospy.get_time()
        
        # Initialize invariants calculation problem
        self.invariant_position_calculator = invariants_position_calculation.OCP_calc_pos(window_len=self.window_nb_samples,rms_error_traj= 2*10**-2,fatrop_solver=True)

           
    def build_time_window(self, new_position):
        # Check if enough progress has passed for the new measurement to be included in the window
        progress_time = rospy.get_time() # measure progress
        #print(progress_time)

        if progress_time > self.progress_trigger:
            #print("New measurement")
            
            # Update window (unless it is a duplicate value)
            if np.all(new_position != self.window_measured_positions[-1]):

                self.window_measured_positions[:-1] = self.window_measured_positions[1:] # push all measurements one sample back
                self.window_measured_positions[-1] = new_position # add new measurement as last sample

            else:
                print("Skipping new measurement because it is a duplicate")            
            
            # Set new time trigger
            self.progress_trigger = self.progress_trigger + self.window_progress_step     
            #print(self.progress_trigger)

            # Report the number of nonzero samples in the window
            number_window_samples = np.count_nonzero(self.window_measured_positions[:, 0])
            if number_window_samples != self.window_nb_samples:
                print(f"Filling window: {number_window_samples} out of {self.window_nb_samples}")
            #print(self.window_measured_positions)
                
    def callback_vive(self, pose_msg):
        # Callback function to process the received Pose message
        new_position = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
        
        # Update window of measurements
        self.build_time_window(new_position)


    def run(self):

        # Function to be run in a loop at the specified update rate
        while not rospy.is_shutdown():
            
            # Check if window of measurements is not yet full
            if self.window_measured_positions[0,0] == 0:
                invariants_pos = 0
            else:  
                
                # print('TIME UNTIL FIRST POSITION CALCULATION: ', rospy.get_time()-self.starttime)

                # Call the function from your invariant calculator
                # print(self.window_measured_positions)
                invariants_pos, traj, mf = self.invariant_position_calculator.calculate_invariants(self.window_measured_positions, self.window_progress_step)

                # print(traj)
                # print(invariants_pos)

                invariants_pos_float32_array = helper_functions_ros.convert_nparray_to_Float32MultiArray(invariants_pos)

                self.publisher_invariants_position.publish(invariants_pos_float32_array)
                

            self.update_rate.sleep() # Sleep to maintain the specified update rate

if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        ros_invariants_position_live_node = ROSInvariantsCalculationPositionLive()
        
        # Run the loop of the node
        ros_invariants_position_live_node.run()
    except rospy.ROSInterruptException:
        pass
