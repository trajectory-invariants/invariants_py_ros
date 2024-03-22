#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32
import dtw as dtw

class ROSInvariantsClassification:
    def __init__(self):
        rospy.init_node('ros_invariants_classification', anonymous=True)
        self.update_rate = rospy.Rate(20)  # Set the ROS node update rate (Default: 20 Hz)
        
        # Create a ROS topic subscribers and publishers
        rospy.Subscriber('/first_invariant', Float32MultiArray, self.callback_first_invariant)
        rospy.Subscriber('/second_invariant', Float32MultiArray, self.callback_second_invariant)
        rospy.Subscriber('/third_invariant', Float32MultiArray, self.callback_third_invariant)
        self.publisher_dtw_distance = rospy.Publisher('/dtw_distance', Float32, queue_size=10)

        # Initialize all invariants as the correct message types
        self.inv_1 = Float32MultiArray()
        self.inv_2 = Float32MultiArray()
        self.inv_3 = Float32MultiArray()

        # Initialize DTW distance
        self.distance_to_segment = None 

        self.starting_time = rospy.get_time()


    def callback_first_invariant(self, data):
        self.inv_1.data = data.data

        # Invert the sign of the data if the biggest value is negative
        if max(self.inv_1.data) < (-1)*min(self.inv_1.data):
            self.inv_1.data = (-1)*self.inv_1.data


    def callback_second_invariant(self, data):
        self.inv_2.data = data.data

        # Invert the sign of the data if the biggest value is negative
        if max(self.inv_2.data) < (-1)*min(self.inv_2.data):
            self.inv_2.data = (-1)*self.inv_2.data
    

    def callback_third_invariant(self, data):
        self.inv_3.data = data.data

        # Invert the sign of the data if the biggest value is negative
        if max(self.inv_3.data) < (-1)*min(self.inv_3.data):
            self.inv_3.data = (-1)*self.inv_3.data


    def dtw_distance(self, a, b, band_size):
        # Calculates final value in DTW matrix between signals a and b, while staying inside the specified band
        alignment = dtw.dtw(a, b, keep_internals=True, window_args= {"window_type": "sakoechiba", "window_size": band_size})
        return alignment.distance


    def dtw_segmentation(self, inv_1, inv_2, references, distance_threshold):
        # Determine size of reference window and set the band size to be half of the window for first invariant
        # For second invariant, data might be lagging behind, so we choose a bigger band size
        window_size = len(references[0])
        band_size_v1 = int(window_size/2 - 1)
        band_size_v2 = band_size_v1 + 3

        # Update current DTW distance
        distance_to_segment = self.dtw_distance(inv_1, references[0], band_size_v1)

        # Segment is only recognized if the DTW distance is below the threshold
        if distance_to_segment < distance_threshold:
            print('SEGMENT FOUND')

            # Calculate DTW distance to all references
            distance_to_gesture_1 = self.dtw_distance(inv_2, references[1], band_size_v1)
            distance_to_gesture_2 = self.dtw_distance(inv_2, references[2], band_size_v1)
            # print(distance_to_gesture_1)
            # print(distance_to_gesture_2)
            print(distance_to_segment)
            print(inv_1)

            # Classification based on Nearest Neighbors approach
            if distance_to_gesture_1 - 5 < distance_to_gesture_2:
                print('GESTURE == START')
            else:
                print('GESTURE == STOP')

            # Print time passed until a segment is found    
            print(rospy.get_time() - self.starting_time)

            # Sleep to avoid multiple recognitions of same segment
            rospy.sleep(1.5)
        return distance_to_segment


    def run(self):
        # Arbitrary values, could be defined more vigorously
        references = np.zeros((3,15)) 
        references[0] = [-0.013336, 0.123938, 0.412655, 0.896227, 1.484626, 2.107109, 2.689924, 3.118931, 3.179457, 2.759977, 2.030783, 1.145614, 0.576452, 0.367894, 0.246758]
        references[1] = [-1.053221, -0.922412, -0.391974, 0.729533, 2.538487, 4.851655, 7.073614, 8.322857, 7.943930, 6.293696, 4.106775, 2.219112, 0.952984, 0.213465, -0.106656]
        references[2] = [-0.704372, -0.737801, -0.749550, -0.718569, -0.601639, -0.314620, 0.265431, 1.226275, 2.349643, 2.815916, 2.225132, 1.232017, 0.754472, 0.686508, 0.618684]
        distance_threshold = 20

        previous_inv_2 = ()

        while not rospy.is_shutdown():

            # Check if data is received yet
            if len(self.inv_1.data) != 0 and len(self.inv_2.data) != 0:
                
                # If data of the second invariant is empty, use the last available data
                # if len(self.inv_2.data) == 0:
                #     self.inv_2.data = previous_inv_2
                
                # Update current DTW distance
                # print(self.inv_2.data)
                self.distance_to_segment = self.dtw_segmentation(self.inv_1.data[3:17], self.inv_2.data[6:20], references, distance_threshold)

                # Update last available data for second invariant
                # previous_inv_2 = self.inv_2.data

                # Publish DTW distance
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