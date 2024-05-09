#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 26 13:49:11 2019

@author: zeno

Class to retrieve the topics for the HTC-Vive Virtual Reality(VR) system, can be extended/adapted to other VR systems working with ROS topics.

"""

# rospy is a pure Python client library for ROS. 
# The rospy client API enables Python programmers to quickly interface with ROS Topics, Services, and Parameters.
import rospy

# tf is a ROS-package that lets the user keep track of multiple coordinate frames over time. 
# tf maintains the relationship between coordinate frames in a tree structure buffered in time, and lets the user transform points, vectors, etc between any two coordinate frames at any desired point in time.
import tf

# time is a python module to handle time-related tasks.
import time

# sensor_msgs is a ROS-package that contains ROS-messages for commonly used sensors.
# Joy is a message that reports the states of the axes and buttons of a joystick.
from sensor_msgs.msg import Joy

class VR_system:
    """Class adding a listener to the vive system to retrieve data"""
    
    
    
    def __init__(self):
        
        # rospy.Rate convenience class which makes a best effort at maintaining a particular rate for a loop.
        self.rate = rospy.Rate(60.0)
        
        
        print('Adding tf TransformListener')
        self.listener = tf.TransformListener()
        
        #retrieve all frames that start with 'lighthouse', 'controller', 'tracker', 'world' for th HTC system
        [self.all_frames, self.vive_frames] = [],[]
        self.updateFrames()
        
        self.buttonSubscriberExists = False
        self.buttonControllername = ''
        self.buttonSubscriber = 0
        self.controllerJoy = Joy()
        self.buttonState = dict({"top" : 0.0, "trigger" : 0.0, "pad" : 0.0, "side_r" : 0.0})
        
        
        
    def updateFrames(self):
        # Lists all visible Vive frames
        
        time.sleep(0.5)
        allFrames = self.listener.getFrameStrings()
        viveFrames = []
        viveFrames.append('world')
        for frame in allFrames:
            if frame.startswith('lighthouse') or frame.startswith('controller') or frame.startswith('world') or frame.startswith('hmd') or frame.startswith('tracker'):
                # print frame
                viveFrames.append(frame)
        
        print ('--VR_frames updated--')
        
        [self.__allFrames, self.__viveFrames] = allFrames, viveFrames
        
    def getViveFrames(self):
        return self.__viveFrames
    
    def getAllTFFrames(self):
        return self.__allFrames
    
    def getViveFrameInput(self):
        self.updateFrames()
        index = 0
        for frame in self.__viveFrames:
            print(str(index) + ': ' +  frame)
            index += 1
        
        index = input('Select the frame using the numbers above: ')
        
        return self.getViveFrameIndex(index)
    
    def getViveFrameIndex(self, index):
        return self.__viveFrames[int(index)]
        
            
    def getPose(self, fromFrame, toFrame):
        
        shutdown = False
        while not shutdown:
            try:
                if fromFrame == None or toFrame == None:
                    time.sleep(0.5)
                    print ('Select from which frame using the numbers above: ')
                    fromFrame = self.getViveFrameInput()
                    print ('Select to which frame using the numbers above: ')
                    toFrame = self.getViveFrameInput()
                                           
                if fromFrame != None and toFrame != None:
                    #print 'Record from ||', fromFrame, '|| to frame ||', toFrame, '||'
                    
                    now = rospy.Time.now()
                    (trans,rot) = self.listener.lookupTransform(fromFrame, toFrame, rospy.Time(0))
                   
                    quat = [trans[0], trans[1], trans[2], rot[0], rot[1],rot[2],rot[3]]
                    matrix_pose = self.listener.fromTranslationRotation(trans, rot)
                    
                    shutdown = True
                    return quat, matrix_pose, fromFrame, toFrame
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, SyntaxError):
                print ("EXCEPTION FOUND..")
                continue
            except KeyboardInterrupt:
                shutdown = True
            self.rate.sleep()
            
    def getQuaternion(self, fromFrame, toFrame):
        return self.getPose(fromFrame, toFrame)[0]
    
    def getTransform(self, fromFrame, toFrame):
        return self.getPose(fromFrame, toFrame)[1]
            
    def recordDuration(self, fromFrame = None, toFrame = None, duration = 5.0):
        # Record pose of a frame w.r.t. another during a certain time interval.
        # Write recorded data to .csv file.
        print ("RECORDING from " + fromFrame + " TO: " +  toFrame + " (" + str(duration) + " seconds)")
        first = True
        shutdown = False
        
        rows = []
        while not shutdown:
            try:
                quat = self.getQuaternion(fromFrame, toFrame)
                
                if first:
                    oldclock = time.time()
                    first = False
                clock = time.time() - oldclock
                
                if clock < 10**-5:
                    clock = 0.0
                    
                if clock > duration:
                    print ("___Recorded " +  str(duration) + " seconds of data____")
                    shutdown = True
                    
                else:
                     rows.append([clock, quat[0], quat[1], quat[2], quat[3], quat[4], quat[5], quat[6]])
    
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, SyntaxError):
                print ("EXCEPTION FOUND..")
                continue
            
            self.rate.sleep()
        return rows    
 
#%% functions to register button presses
        
    def joyMsgCallback(self, joyData):
            self.controllerJoy.header = joyData.header
            self.controllerJoy.axes = joyData.axes
            self.controllerJoy.buttons = joyData.buttons
            
            
    def getButtonsState(self, controllerName):
        
        if (not self.buttonSubscriberExists) or (self.buttonSubscriberExists and not(self.buttonControllername == controllerName)):
            self.buttonSubscriber = rospy.Subscriber('/vive/' + controllerName + '/joy', Joy, self.joyMsgCallback, queue_size=1)
            self.buttonSubscriberExists = True
            self.buttonControllername = controllerName
            time.sleep(0.5)
    
        self.buttonState["top"] = self.controllerJoy.buttons[0]
        self.buttonState["trigger"] = self.controllerJoy.buttons[1]
        self.buttonState["pad"] = self.controllerJoy.buttons[2]
        self.buttonState["side_r"] = self.controllerJoy.buttons[3]
        return self.buttonState
    
    
if __name__ == '__main__':
    rospy.init_node('vr_system', anonymous=False)
    vr_system = VR_system()
    print ('sys ready')
#    print vr_system.buttonState
#    
#    #%% testing if button state shows up
#    controller = vr_system.getViveFrameInput()
#    try:
#        while True:
#            buttons = vr_system.getButtonsState(controller)
#            print buttons
#            vr_system.rate.sleep()
#            
#    except KeyboardInterrupt:
#        print 'KEYBOARD INTERRUPT'
    
    
