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

        self.current_distance = None # Initialize DTW distance

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


    def dtw_segmentation(self, inv_1, reference_inv_1, distance_threshold):
        # Determine size of reference window and set the band size to be half of the window
        window_size = len(reference_inv_1)
        band_size = int(window_size/2 - 1)

        # Update current DTW distance
        current_distance = self.dtw_distance(inv_1, reference_inv_1, band_size)

        if current_distance < distance_threshold: # Segment is only recognized if the DTW distance is below the threshold
            print('SEGMENT FOUND')
            # print(inv_1)
            # print(current_distance)
            print(rospy.get_time() - self.starting_time) # Print time passed until a segment is found
        return current_distance


    def run(self):
        # Arbitrary values, could be defined more vigorously 
        reference_inv_1 = [-0.013336, 0.123938, 0.412655, 0.896227, 1.484626, 2.107109, 2.689924, 3.118931, 3.179457, 2.759977, 2.030783, 1.145614, 0.576452, 0.367894, 0.246758]
        distance_threshold = 15

        while not rospy.is_shutdown():
            
            # Check if data is received yet
            if len(self.inv_1.data) != 0:
                # Update current DTW distance
                self.current_distance = self.dtw_segmentation(self.inv_1.data[3:17], reference_inv_1, distance_threshold)

                # Publish DTW distance
                self.publisher_dtw_distance.publish(self.current_distance)

                # Check if segment is found
                if self.current_distance < distance_threshold:
                    print(self.inv_2.data[3:17])
                    rospy.sleep(1.5) # Sleep to avoid multiple recognitions of same segment
            self.update_rate.sleep() # Sleep to maintain the specified update rate


if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        ros_invariants_classification_node = ROSInvariantsClassification()
        
        # Run the loop of the node
        ros_invariants_classification_node.run()
    except rospy.ROSInterruptException:
        pass