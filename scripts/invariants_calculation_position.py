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

class ROSInvariantsCalculationPosition:
    def __init__(self):

        rospy.init_node('ros_invariants_calculation_position', anonymous=True)
        self.update_rate = rospy.Rate(30)  # Set the ROS node update rate (Default: 20 Hz)
                
        # Initialize the start time
        self.starttime = rospy.get_time()

        # Create a ROS topic subscribers and publishers
        rospy.Subscriber('/pose_data', Pose, self.callback_pose)

        self.publisher_invariants_position = rospy.Publisher('/invariants_position_result', std_msgs.msg.Float32MultiArray, queue_size=10)   
        self.publisher_traj_calc_array = rospy.Publisher('/trajectory_online_array', std_msgs.msg.Float32MultiArray, queue_size=10)
        self.publisher_traj_calc = rospy.Publisher('/trajectory_online', Marker, queue_size=10)
        self.publisher_traj_meas = rospy.Publisher('/pose_data_stamped', Marker, queue_size=10)
        self.publisher_mf = rospy.Publisher('/moving_frame', PoseStamped, queue_size=10)

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
                
    def callback_pose(self, pose_msg):
        # Callback function to process the received Pose message
        new_position = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z,1])

        pose_matrix = np.array([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
        # pose_matrix = np.array([[0.866, 0.5, 0, -1.5],[-0.5, 0.866, 0, 2.3],[0, 0, 1, 1.0],[0, 0, 0, 1]])
        # pose_matrix = np.array([[0.866, 0, 0.5, 1.0],[0.433, 0.707, -0.612, -0.5],[-0.5, 0.707, 0.354, 0.7],[0, 0, 0, 1]])
        # pose_matrix = np.array([[0.5, -0.866, 0.433, 0.5],[0.866, 0.354, -0.25, 1.2],[-0.5, 0.5, 0.707, -0.3],[0, 0, 0, 1]])

        new_position = np.matmul(pose_matrix,new_position)
        
        # Update window of measurements
        self.build_time_window(new_position[0:3])

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



    def run(self):

        # Create the Marker message
        marker = Marker()
        marker.header = std_msgs.msg.Header()
        marker.header.frame_id = "world"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # Set marker properties
        marker.scale.x = 0.0025
        marker.color.a = 1.0
        marker.color.r = 1.0

        # Create a PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"

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
                traj_float32_array = helper_functions_ros.convert_nparray_to_Float32MultiArray(traj)

                self.publisher_invariants_position.publish(invariants_pos_float32_array)
                self.publisher_traj_calc_array.publish(traj_float32_array)

                # print('INVARIANTS PUBLISHED: ', rospy.get_time()-self.starttime)
                # print(max(invariants_pos_float32_array.data[0::3]))

                # Add points to the marker
                marker.points = []
                for point in traj[:-2]:  # Assuming traj is a numpy array of shape (N, 3)
                    p = Point()
                    p.x = point[0]
                    p.y = point[1]
                    p.z = point[2]
                    marker.points.append(p)

                # Publish the Marker
                marker.header.stamp = rospy.Time.now() # add timestamp
                self.publisher_traj_calc.publish(marker)

                # Extend the 3x3 rotation matrix to a 4x4 transformation matrix
                transformation_matrix = np.eye(4)
                transformation_matrix[:3, :3] = mf[-2,:,:]

                # Set the header
                pose_stamped.header.stamp = rospy.Time.now()
                # Set the position
                pose_stamped.pose.position.x = traj[-2, 0]
                pose_stamped.pose.position.y = traj[-2, 1]
                pose_stamped.pose.position.z = traj[-2, 2]
                quaternion = quaternion_from_matrix(transformation_matrix)
                pose_stamped.pose.orientation.x = quaternion[0]
                pose_stamped.pose.orientation.y = quaternion[1]
                pose_stamped.pose.orientation.z = quaternion[2]
                pose_stamped.pose.orientation.w = quaternion[3]

                self.publisher_mf.publish(pose_stamped)
                

            self.update_rate.sleep() # Sleep to maintain the specified update rate

if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        ros_invariants_position_node = ROSInvariantsCalculationPosition()
        
        # Run the loop of the node
        ros_invariants_position_node.run()
    except rospy.ROSInterruptException:
        pass