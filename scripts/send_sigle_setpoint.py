#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from pynput.keyboard import Key, Listener
from ros_spline_fitting_trajectory.msg import Trajectory

class SendSetPoint:
    def __init__(self):
        rospy.init_node('simulate_bottle_pos', anonymous=True)

        self.tf = np.zeros(7)
        self.target_pos = np.zeros(3)
        self.initial_time = rospy.Time.now().to_sec()
        self.desired_total_time = 9
        self.inv_node_output = np.ones(10)*100
        self.discrete_traj = None

        rospy.Subscriber('tf_pose', Float64MultiArray, self.callback_tf)
        rospy.Subscriber('raw_inv_gen_node_output', Float64MultiArray, self.callback_invnode)
        rospy.Subscriber("trajectory_pub", Trajectory, self.callback_traj)
        self.pub_node_output = rospy.Publisher('inv_gen_node_output', Float64MultiArray, queue_size=10)#, latch=True)
        self.pos_publisher = rospy.Publisher('/model_set_point', Float64MultiArray, queue_size=10, latch=True)

    def callback_tf(self,data):

        self.tf = data.data

    def callback_invnode(self,data):
        self.inv_node_output = data.data

    def callback_traj(self, trajectory):
        self.discrete_traj = trajectory.poses

    def evaluate_set_point(self):
        if not self.tf[0] == 0 and not self.inv_node_output[0] == 100 and not self.discrete_traj == None:
            x = []
            y = []
            z = []
            for pose in self.discrete_traj:
                x.append(pose.position.x)
                y.append(pose.position.y)
                z.append(pose.position.z)
            trajectory = np.array([x,y,z]).T
            current_time = rospy.Time.now().to_sec()-self.initial_time
            progress = current_time/self.desired_total_time
            if progress <= 1:
                self.target_pos = [np.interp(progress,np.linspace(0,1,len(trajectory)),trajectory[:,i]) for i in range(3)]
            self.pos_publisher.publish(Float64MultiArray(data=self.target_pos))
            print(self.target_pos)

            self.pub_node_output.publish(Float64MultiArray(data=self.inv_node_output))
        
        else:
            self.initial_time = rospy.Time.now().to_sec()

if __name__ == '__main__':
    
    setpoint = SendSetPoint()
    
    # This defines the rate at which the node should publish
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        setpoint.evaluate_set_point()
        rate.sleep()