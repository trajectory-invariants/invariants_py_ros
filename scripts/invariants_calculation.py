#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Example of online invariants calculation
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
import invariants_py.rockit_frenetserret_calculation_minimumjerk as invariants_calculation
import std_msgs.msg
import helper_functions_ros
from nav_msgs.msg import Path

class ROSInvariantsCalculation:
    def __init__(self):
        rospy.init_node('ros_invariants_calculation', anonymous=True)
        self.update_rate = rospy.Rate(30)  # Set the ROS node update rate (Default: 20 Hz)
        
        # Create a ROS topic subscribers and publishers
        rospy.Subscriber('/pose_data', Pose, self.callback_pose)
        #self.result_publisher = rospy.Publisher('/invariant_result', Float64, queue_size=10)  
        self.publisher_traj_calc = rospy.Publisher('/trajectory_online', Path, queue_size=10)
        self.publisher_traj_meas = rospy.Publisher('/pose_data_stamped', PoseStamped, queue_size=10)

        # Define parameters of the window of measurements
        self.window_nb_samples = 21 # number of samples in window
        self.window_horizon_length = 2.0 # size of the window in time [sec]
        
        # Initialization window
        self.window_progress_step = self.window_horizon_length/self.window_nb_samples # time step between samples in window [sec]
        self.window_measured_positions = np.zeros((self.window_nb_samples,3))
        self.progress_trigger = rospy.get_time()
        
        # Initialize invariants calculation problem
        self.invariant_calculator = invariants_calculation.FrenetSerret_calc(nb_samples=self.window_nb_samples,w_pos=1,w_regul_jerk=10-10,fatrop_solver=True)
           
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
                
    def callback_pose(self, pose_msg):
        # Callback function to process the received Pose message
        new_position = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
        
        # Update window of measurements
        self.build_time_window(new_position)

        # Publish Pose message again, but now with a time stamp and frame_id (necessary for visualization in rviz)
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.pose = pose_msg # copy pose message
        pose_stamped_msg.header = std_msgs.msg.Header() # create a new Header message
        pose_stamped_msg.header.stamp = rospy.Time.now() # add timestamp
        pose_stamped_msg.header.frame_id = 'world' # add frame_id
        self.publisher_traj_meas.publish(pose_stamped_msg)

    def run(self):
        # Function to be run in a loop at the specified update rate
        while not rospy.is_shutdown():
            
            # Check if window of measurements is not yet full
            if self.window_measured_positions[0,0] == 0:
                invariants = 0
            else:          
                # Call the function from your invariant calculator
                invariants, traj, mf = self.invariant_calculator.calculate_invariants_online(self.window_measured_positions, self.window_progress_step)
                
                # Visualize the trajectory in rviz
                trajectory_msg = helper_functions_ros.convert_nparray_to_Path(traj[:-1])
                self.publisher_traj_calc.publish(trajectory_msg)
                print(invariants)
                print(traj[:-1])

            self.update_rate.sleep() # Sleep to maintain the specified update rate

if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        ros_invariants_node = ROSInvariantsCalculation()
        
        # Run the loop of the node
        ros_invariants_node.run()
    except rospy.ROSInterruptException:
        pass
