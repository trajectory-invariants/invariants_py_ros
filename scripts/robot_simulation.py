#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Int32
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

class ROSRobotSimulation:
    def __init__(self):
        rospy.init_node('ros_robot_simulation', anonymous=True)
        self.update_rate = rospy.Rate(30)  # Set the ROS node update rate (Default: 20 Hz)
        
        # Create a ROS topic subscribers and publishers
        rospy.Subscriber('/gesture', Int32, self.callback_gesture)

        self.starttime = rospy.get_time()

        # Initialize the gesture as the correct message types
        self.gesture = Int32()  

        # Initialize the robot simulation
        self.speed = 1
        self.direction = 1
        self.status = 0
        self.t = 0
    
    
    def callback_gesture(self, data):
        
        self.gesture.data = data.data

        if self.gesture.data == 1: # ROT HOR
             self.direction = -1

        elif self.gesture.data == 2: # ROT UP
             self.direction = 1

        elif self.gesture.data == 3: # ROT DOWN
             self.status = 1

        elif self.gesture.data == 4: # TRANS HOR
             self.status = 0

        elif self.gesture.data == 5: # TRANS UP
             self.speed = 0.5
            
        elif self.gesture.data == 6: # TRANS DOWN
             self.speed = 1

    
    def run(self):
        
        px = [1]
        py = [0]
        
        

        while not rospy.is_shutdown():
            
            
            # # Check if data is received yet
            # if len(self.gesture.data) != 0:
            # print(self.gesture.data)


            if self.status == 1:
                if self.direction == 1:
                    self.t += 2*np.pi/90
                elif self.direction == -1:
                    self.t -= 2*np.pi/90
                px.append(np.cos(self.speed * self.t))
                py.append(np.sin(self.speed * self.t))
            else:
                px.append(px[-1])
                py.append(py[-1])
            
        

            self.update_rate.sleep() # Sleep to maintain the specified update rate
        # print(px)
        # print(py)
        np.save('px.npy', px)
        np.save('py.npy', py)
        # print(os.path.abspath("."))



if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        ros_robot_simulation_node = ROSRobotSimulation()
        
        # Run the loop of the node
        ros_robot_simulation_node.run()
    except rospy.ROSInterruptException:
        pass