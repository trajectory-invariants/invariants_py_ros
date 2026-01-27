#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped

class EndEffectorPosePublisher:
    def __init__(self):
        rospy.init_node('publish_tcp_pose')
        self.pub = rospy.Publisher('tcp_pose', Float64MultiArray, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(100)  # 50 Hz

        # Set your frames here
        self.base_frame = rospy.get_param('~base_frame', 'world')
        self.ee_frame = rospy.get_param('~ee_frame', 'TCP_frame')

    def run(self):
        while not rospy.is_shutdown():
            try:
                trans: TransformStamped = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rospy.Time(0), rospy.Duration(1.0))
                pos = trans.transform.translation
                ori = trans.transform.rotation
                pose = [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
                self.pub.publish(Float64MultiArray(data=pose))
            except Exception as e:
                rospy.logwarn_throttle(5.0, f"TF lookup failed: {e}")
            self.rate.sleep()

if __name__ == '__main__':
    node = EndEffectorPosePublisher()
    node.run()