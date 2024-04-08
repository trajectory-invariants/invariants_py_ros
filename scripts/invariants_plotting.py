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
        rospy.Subscriber('/invariants_result', Float32MultiArray, self.callback_invariants)
        self.publisher_inv_1 = rospy.Publisher('/first_invariant', Float32MultiArray, queue_size=10)
        self.publisher_inv_2 = rospy.Publisher('/second_invariant', Float32MultiArray, queue_size=10)
        self.publisher_inv_3 = rospy.Publisher('/third_invariant', Float32MultiArray, queue_size=10)

        # Initialize data structures
        self.invariants = None
        self.inv_1 = Float32MultiArray()
        self.inv_2 = Float32MultiArray()
        self.inv_3 = Float32MultiArray()

    def callback_invariants(self, data):
        self.invariants = np.reshape(data.data, (-1, 3))

        # Split data into correct invariants
        self.inv_1.data = data.data[0::3] # Every third value, starting from the first
        self.inv_2.data = data.data[1::3] # Every third value, starting from the second
        self.inv_3.data = data.data[2::3] # Every third value, starting from the third

    def run(self):
        while not rospy.is_shutdown():

            # Check if data is received yet
            if self.invariants is not None:

                # Publish data to be plotted with rqt_plot
                self.publisher_inv_1.publish(self.inv_1)
                self.publisher_inv_2.publish(self.inv_2)
                self.publisher_inv_3.publish(self.inv_3)

            self.update_rate.sleep() # Sleep to maintain the specified update rate


if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        ros_invariants_plotting_node = ROSInvariantsPlotting()
        
        # Run the loop of the node
        ros_invariants_plotting_node.run()
    except rospy.ROSInterruptException:
        pass
