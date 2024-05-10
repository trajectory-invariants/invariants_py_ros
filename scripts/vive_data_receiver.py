#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import vr_system as vr
import time
from geometry_msgs.msg import Pose

def selectController(handoverSetup):
    """
    Lists all visible vive components and lets user choose one
    @param handoverSetup: refers to the setup class
    @return controllerName: the chosen controller
    """
    shutdown = False
    while not shutdown:
        try:
            controllerName = handoverSetup.getViveFrameInput()
            shutdown = True
        except (transform.Exception, transform.LookupException, transform.ConnectivityException, transform.ExtrapolationException, SyntaxError):
            print ("EXCEPTION FOUND..")
            continue
        except KeyboardInterrupt:
            shutdown = True
            
        handoverSetup.rate.sleep()
    return controllerName

class ROSViveDataReceiver:
    def __init__(self):

        rospy.init_node('ros_vive_data_receiver', anonymous=True)
        self.update_rate = rospy.Rate(60)

        # Create a ROS topic subscribers and publishers
        self.publisher_vive_data = rospy.Publisher('/vive_data', Float32MultiArray, queue_size=10)

        # Initialize the data structure
        self.vive_data = Pose()

        # Initialize the VR system
        self.vr_system = vr.VR_system()

        print ("select the tracker of which you record the motion")
        self.trackerName = selectController(self.vr_system)
        print ("select the controller/joystick to start/stop the recording")
        self.controllerName = selectController(self.vr_system)
        print ("select the lighthouse as reference frame for the recorded motion")
        self.lightHouse = selectController(self.vr_system)
        print("Press the trigger button on the back of the joystick to start recording. Stop by releasing the trigger")

        self.prev_states = dict({"top" : 0.0, "trigger" : 0.0, "pad" : 0.0, "side_r" : 0.0})
        self.new_states = dict({"top" : 0.0, "trigger" : 0.0, "pad" : 0.0, "side_r" : 0.0})
        self.start_time = 0.0


    def run(self):

        while not rospy.is_shutdown():
    
            self.new_states = self.vr_system.getButtonsState(self.controllerName)
            
            self.vive_data.data = []
            # check trigger pressed
            if (self.prev_states["trigger"] == 0.0) and (self.new_states["trigger"] == 1.0):
                self.start_time = time.time()   
                print ("START RECORDING POSES")
                
            if self.new_states["trigger"] == 1.0:
                quat = self.vr_system.getQuaternion(self.lightHouse, self.trackerName)
                elapsed_time = time.time()- self.start_time
                print ("elapsed time IS:" + str(elapsed_time))
                self.vive_data.position.x = quat[0]
                self.vive_data.position.y = quat[1]
                self.vive_data.position.z = quat[2]
                self.vive_data.orientation.x = quat[3]
                self.vive_data.orientation.y = quat[4]
                self.vive_data.orientation.z = quat[5]
                self.vive_data.orientation.w = quat[6]
            
            # check trigger released
            if (self.prev_states["trigger"] == 1.0) and (self.new_states["trigger"] == 0.0):
                print ("STOP RECORDING POSES")

            self.publisher_vive_data.publish(self.vive_data)
            print(self.vive_data.position.x, self.vive_data.position.y, self.vive_data.position.z)

            self.prev_states = self.new_states.copy()
            self.update_rate.sleep()


if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        ros_vive_data_receiver_node = ROSViveDataReceiver()
        
        # Run the loop of the node
        ros_vive_data_receiver_node.run()
    except rospy.ROSInterruptException:
        pass