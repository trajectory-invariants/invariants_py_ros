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
    
        self.invariants = None

    def callback_invariants(self, data):
        self.invariants = data.data

    def run(self):
        while not rospy.is_shutdown():
            if self.invariants is not None:
                print(self.invariants)
            self.update_rate.sleep() # Sleep to maintain the specified update rate


if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        ros_invariants_plotting_node = ROSInvariantsPlotting()
        
        # Run the loop of the node
        ros_invariants_plotting_node.run()
    except rospy.ROSInterruptException:
        pass
