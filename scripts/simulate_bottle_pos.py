#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from pynput.keyboard import Key, Listener

class SimulateBottlePos:
    def __init__(self):
        rospy.init_node('simulate_bottle_pos', anonymous=True)

        self.tf = np.zeros(7)
        self.discrete_moving_tgt = True
        # self.switching_time = 3 # for discrete moving
        self.switching_progress = 0.33 # for discrete moving
        # self.switching_time = 10 # for continuous moving
        # # FOR BOTTLE APPROACH
        # self.pos_w_home = np.array([0.294492,0.068941,0.483837])
        # self.initial_bottle_position = np.array([0.4,0.1,-0.074])
        # self.moved_bottle_position = np.array([0.45,-0.15,-0.074])
        # FOR CRATE FILLING
        self.pos_w_home = np.array([0.25798,-0.0016,0.2561])
        self.initial_bottle_position = np.array([0.3,0,0.015]) # MB2
        self.moved_bottle_position = np.array([0.3,-0.21,0.015]) # LC2
        # np.array([0.21,-0.39,0.015]) # LA1
        # np.array([0.21,-0.3,0.015]) # LB1
        # np.array([0.21,-0.21,0.015]) # LC1
        # np.array([0.3,-0.21,0.015]) # LC2
        # np.array([0.3,-0.09,0.015]) # MA2
        # np.array([0.3,0,0.015]) # MB2
        # np.array([0.3,0.09,0.015]) # MC2
        # np.array([0.39,0.09,0.015]) # MC3
        # np.array([0.39,0.21,0.015]) # RA3
        # np.array([0.21,0.3,0.015]) # RB1
        # np.array([0.3,0.3,0.015]) # RB2
        self.bottle_pos = np.array([0,0,0])
        self.initial_time = 0
        self.motion_duration = 7
        self.progress = 0
        self.switched_time = 100

        rospy.Subscriber('tf_pose', Float64MultiArray, self.callback_tf)
        rospy.Subscriber('progress', Float64MultiArray, self.callback_progress)
        self.pos_publisher = rospy.Publisher('/sim_bottle_pos_pub', Float64MultiArray, queue_size=10, latch=True)


    def callback_tf(self,data):

        self.tf = data.data

    def callback_progress(self,data):

        self.progress = data.data[0]

    def create_bottle_pos(self):
        if not self.tf[0] == 0:
            current_time = rospy.Time.now().to_sec()-self.initial_time
            if self.progress >= self.switching_progress or current_time >= self.switched_time:
                if self.discrete_moving_tgt:
                    self.pos_home_bottle = self.moved_bottle_position
                    self.switched_time = rospy.Time.now().to_sec()-self.initial_time
                else:
                    if current_time - self.initial_time < self.motion_duration + self.switching_time:
                        self.pos_home_bottle = self.initial_bottle_position + (self.moved_bottle_position - self.initial_bottle_position)* (current_time - self.initial_time - self.switching_time)/(self.motion_duration)
            else:
                self.pos_home_bottle = self.initial_bottle_position
        else:
            self.initial_time = rospy.Time.now().to_sec()
            self.pos_home_bottle = self.initial_bottle_position

        self.pos_w_bottle = self.pos_home_bottle + self.pos_w_home

        self.pos_publisher.publish(Float64MultiArray(data=self.pos_w_bottle))

if __name__ == '__main__':
    
    simulator = SimulateBottlePos()
    # This defines the rate at which the node should publish
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        simulator.create_bottle_pos()
        rate.sleep()