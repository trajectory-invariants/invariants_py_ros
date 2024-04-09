#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

class ROSInvariantsPlotting:
    def __init__(self):
        rospy.init_node('ros_invariants_plotting', anonymous=True)
        self.update_rate = rospy.Rate(30)  # Set the ROS node update rate (Default: 20 Hz)
        
        # Create a ROS topic subscribers and publishers
        rospy.Subscriber('/invariants_position_result', Float32MultiArray, self.callback_invariants_position)
        rospy.Subscriber('/invariants_rotation_result', Float32MultiArray, self.callback_invariants_rotation)
        self.publisher_pos_1 = rospy.Publisher('/first_invariant_pos', Float32MultiArray, queue_size=10)
        self.publisher_pos_2 = rospy.Publisher('/second_invariant_pos', Float32MultiArray, queue_size=10)
        self.publisher_pos_3 = rospy.Publisher('/third_invariant_pos', Float32MultiArray, queue_size=10)
        self.publisher_rot_1 = rospy.Publisher('/first_invariant_rot', Float32MultiArray, queue_size=10)
        self.publisher_rot_2 = rospy.Publisher('/second_invariant_rot', Float32MultiArray, queue_size=10)
        self.publisher_rot_3 = rospy.Publisher('/third_invariant_rot', Float32MultiArray, queue_size=10)

        # Initialize data structures
        self.invariants_pos = None
        self.invariants_rot = None
        self.pos_1 = Float32MultiArray()
        self.pos_2 = Float32MultiArray()
        self.pos_3 = Float32MultiArray()
        self.rot_1 = Float32MultiArray()
        self.rot_2 = Float32MultiArray()
        self.rot_3 = Float32MultiArray()

    def callback_invariants_position(self, data):
        self.invariants_pos = np.reshape(data.data, (-1, 3))

        # Split data into correct invariants
        self.pos_1.data = data.data[0::3] # Every third value, starting from the first
        self.pos_2.data = data.data[1::3] # Every third value, starting from the second
        self.pos_3.data = data.data[2::3] # Every third value, starting from the third

    def callback_invariants_rotation(self, data):
        self.invariants_rot = np.reshape(data.data, (-1, 3))

        # Split data into correct invariants
        self.rot_1.data = data.data[0::3] # Every third value, starting from the first
        self.rot_2.data = data.data[1::3] # Every third value, starting from the second
        self.rot_3.data = data.data[2::3] # Every third value, starting from the third

    def run(self):
        while not rospy.is_shutdown():

            # Check if data is received yet
            if self.invariants_pos is not None and self.invariants_rot is not None:

                # Publish data to be plotted with rqt_plot
                self.publisher_pos_1.publish(self.pos_1)
                self.publisher_pos_2.publish(self.pos_2)
                self.publisher_pos_3.publish(self.pos_3)
                self.publisher_rot_1.publish(self.rot_1)
                self.publisher_rot_2.publish(self.rot_2)
                self.publisher_rot_3.publish(self.rot_3)

            self.update_rate.sleep() # Sleep to maintain the specified update rate


if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        ros_invariants_plotting_node = ROSInvariantsPlotting()
        
        # Run the loop of the node
        ros_invariants_plotting_node.run()
    except rospy.ROSInterruptException:
        pass