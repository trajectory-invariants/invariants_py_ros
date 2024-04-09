#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32
from visualization_msgs.msg import Marker
import dtw as dtw

class ROSInvariantsClassification:
    def __init__(self):
        rospy.init_node('ros_invariants_classification', anonymous=True)
        self.update_rate = rospy.Rate(30)  # Set the ROS node update rate (Default: 20 Hz)
        
        # Create a ROS topic subscribers and publishers
        rospy.Subscriber('/first_invariant_pos', Float32MultiArray, self.callback_first_invariant_position)
        # rospy.Subscriber('/second_invariant_pos', Float32MultiArray, self.callback_second_invariant_position)
        # rospy.Subscriber('/third_invariant_pos', Float32MultiArray, self.callback_third_invariant_position)
        rospy.Subscriber('/first_invariant_rot', Float32MultiArray, self.callback_first_invariant_rotation)
        # rospy.Subscriber('/second_invariant_rot', Float32MultiArray, self.callback_second_invariant_rotation)
        # rospy.Subscriber('/third_invariant_rot', Float32MultiArray, self.callback_third_invariant_rotation)

        rospy.Subscriber('/trajectory_online_array', Float32MultiArray, self.callback_trajectory)
        self.publisher_dtw_distance = rospy.Publisher('/dtw_distance', Float32, queue_size=10)

        # Initialize all invariants as the correct message types
        self.pos_1 = Float32MultiArray()
        self.pos_2 = Float32MultiArray()
        self.pos_3 = Float32MultiArray()
        self.rot_1 = Float32MultiArray()
        self.rot_2 = Float32MultiArray()
        self.rot_3 = Float32MultiArray()    

        # Initialize trajectory coordinates
        self.traj_x = Float32MultiArray()
        self.traj_y = Float32MultiArray()
        self.traj_z = Float32MultiArray()

        # Initialize DTW distances
        self.distance_to_segment = None 
        self.previous_distance_to_segment = 100
        self.distance_to_gesture_1 = None
        self.distance_to_gesture_2 = None

        # Initialize current gesture
        self.current_gesture = None
        self.segment_found = False

        # Initialize angle to vertical vector
        self.angle_to_vertical = None

        # Initialize starting time
        self.starting_time = rospy.get_time()



    def callback_first_invariant_position(self, data):
        self.pos_1.data = data.data

        # Invert the sign of the data if the biggest value is negative
        if max(self.pos_1.data) < (-1)*min(self.pos_1.data):
            self.pos_1.data = (-1)*self.pos_1.data


    # def callback_second_invariant_position(self, data):
    #     self.pos_2.data = data.data

    #     # Invert the sign of the data if the biggest value is negative
    #     if max(self.pos_2.data) < (-1)*min(self.pos_2.data):
    #         self.pos_2.data = (-1)*self.pos_2.data
    

    # def callback_third_invariant_position(self, data):
    #     self.pos_3.data = data.data

    #     # Invert the sign of the data if the biggest value is negative
    #     if max(self.pos_3.data) < (-1)*min(self.pos_3.data):
    #         self.pos_3.data = (-1)*self.pos_3.data


    def callback_first_invariant_rotation(self, data):
        self.rot_1.data = data.data

        # Invert the sign of the data if the biggest value is negative
        if max(self.rot_1.data) < (-1)*min(self.rot_1.data):
            self.rot_1.data = (-1)*self.rot_1.data

    
    # def callback_second_invariant_rotation(self, data):
    #     self.rot_2.data = data.data

    #     # Invert the sign of the data if the biggest value is negative
    #     if max(self.rot_2.data) < (-1)*min(self.rot_2.data):
    #         self.rot_2.data = (-1)*self.rot_2.data

    
    # def callback_third_invariant_rotation(self, data):
    #     self.rot_3.data = data.data

    #     # Invert the sign of the data if the biggest value is negative
    #     if max(self.rot_3.data) < (-1)*min(self.rot_3.data):
    #         self.rot_3.data = (-1)*self.rot_3.data


    def callback_trajectory(self, data):
        # Split trajectory data into correct coordinates
        self.traj_x.data = data.data[0::3] # Every third value, starting from the first
        self.traj_y.data = data.data[1::3] # Every third value, starting from the second
        self.traj_z.data = data.data[2::3] # Every third value, starting from the third



    def dtw_distance(self, a, b, band_size):
        # Calculates final value in DTW matrix between signals a and b, while staying inside the specified band
        alignment = dtw.dtw(a, b, keep_internals=True, window_args= {"window_type": "sakoechiba", "window_size": band_size})
        return alignment.distance


    def dtw_segmentation(self, inv_1, references, distance_threshold):
        # Determine size of reference window and set the band size to be half of the window for first invariant
        window_size = len(references[0])
        band_size_v1 = int(window_size/2 - 1)

        # Update DTW distances
        if self.distance_to_segment is not None:
            self.previous_distance_to_segment = self.distance_to_segment
        self.distance_to_segment = self.dtw_distance(inv_1, references[0], band_size_v1)

        # Segment is only recognized if the DTW distance is below the threshold and the distance is increasing
        if (self.previous_distance_to_segment < distance_threshold) and (self.distance_to_segment > self.previous_distance_to_segment):
            self.segment_found = True        
        else:
            self.segment_found = False


    def dtw_classification(self, rot_1, references):
        # Determine size of reference window and set the band size to be more than half of the window for first invariant
        # For second invariant, data might be lagging behind, so we choose a bigger band size
        window_size = len(references[0])
        band_size_w1 = int(window_size/2 - 1) + 3
    
        # Calculate DTW distance to all references
        self.distance_to_gesture_1 = self.dtw_distance(rot_1, references[3], band_size_w1)
        self.distance_to_gesture_2 = self.dtw_distance(rot_1, references[4], band_size_w1)

        # Classification based on Nearest Neighbors approach
        if self.distance_to_gesture_1 - 10 < self.distance_to_gesture_2:
            self.current_gesture = 'START'
        else:
            self.current_gesture = 'STOP'

    
    def angle_to_vertical_axis(self, traj_x, traj_y, traj_z):
        # Calculate the angle between the trajectory and the vertical vector
        
        # Vertical vector
        # vertical_vector = np.array([0.1, 0.4, 0.7])
        vertical_vector = np.array([0, 1, 0])

        # Distance traveled in each direction
        x_dist = traj_x[-1] - traj_x[0]
        y_dist = traj_y[-1] - traj_y[0]
        z_dist = traj_z[-1] - traj_z[0]
        traj_vector = np.array([x_dist, y_dist, z_dist])

        # Angle between the trajectory and the vertical vector
        self.angle_to_vertical = np.arccos(np.dot(vertical_vector, traj_vector) / (np.linalg.norm(vertical_vector) * np.linalg.norm(traj_vector))) * 180 / np.pi
    

    def run(self):
        # Arbitrary values, could be defined more vigorously
        references = np.zeros((5,15)) 
        references[0] = [-0.013336, 0.123938, 0.412655, 0.896227, 1.484626, 2.107109, 2.689924, 3.118931, 3.179457, 2.759977, 2.030783, 1.145614, 0.576452, 0.367894, 0.246758]
        references[1] = [-1.053221, -0.922412, -0.391974, 0.729533, 2.538487, 4.851655, 7.073614, 8.322857, 7.943930, 6.293696, 4.106775, 2.219112, 0.952984, 0.213465, -0.106656]
        references[2] = [-0.704372, -0.737801, -0.749550, -0.718569, -0.601639, -0.314620, 0.265431, 1.226275, 2.349643, 2.815916, 2.225132, 1.232017, 0.754472, 0.686508, 0.618684]
        references[3] = [2.175058, 2.753868, 3.419728, 4.186294, 5.051172, 5.798532, 6.173445, 5.981571, 5.390303, 4.496642, 3.626515, 2.950756, 2.332041, 1.820430, 1.368183]
        references[4] = [0.089681, 0.242293, 0.409137, 0.589363, 0.785925, 1.014047, 1.265902, 1.516832, 1.751753, 1.898293, 1.903506, 1.736325, 1.410860, 1.096605, 0.778782]
        distance_threshold = 12

        while not rospy.is_shutdown():
            
            # Check if data is received yet
            if len(self.pos_1.data) != 0:
                
                # Update segmentation DTW distance
                self.dtw_segmentation(self.pos_1.data, references, distance_threshold)
            
            # Check if data is received yet
            if len(self.traj_x.data) != 0:
                
                # Update angle to vertical vector
                self.angle_to_vertical_axis(self.traj_x.data, self.traj_y.data, self.traj_z.data)

            # Check if data is received yet
            if len(self.rot_1.data) != 0:
                
                # Update classification DTW distance
                self.dtw_classification(self.rot_1.data, references)

            if self.segment_found:
                print('SEGMENT FOUND')

                # Print time passed until a segment is found    
                print('Elapsed time:', rospy.get_time() - self.starting_time)
                
                print('Detected gesture:', self.current_gesture)
                print('DTW distance to START:', self.distance_to_gesture_1)
                print('DTW distance to STOP:', self.distance_to_gesture_2)
                if 60 < self.angle_to_vertical < 120:
                    print('Trajectory is horizontal')
                elif 0 < self.angle_to_vertical < 60:
                    print('Trajectory is upwards')
                elif 120 < self.angle_to_vertical < 180:
                    print('Trajectory is downwards')
                # print(self.angle_to_vertical)
            
                print ('')
            
            # Publish the current DTW distance
            self.publisher_dtw_distance.publish(self.distance_to_segment)

            self.update_rate.sleep() # Sleep to maintain the specified update rate


if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        ros_invariants_classification_node = ROSInvariantsClassification()
        
        # Run the loop of the node
        ros_invariants_classification_node.run()
    except rospy.ROSInterruptException:
        pass