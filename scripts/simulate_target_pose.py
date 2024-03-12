#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class SimulateTargetPose:
    def __init__(self, radius, frequency):
        rospy.init_node('simulate_target_pose', anonymous=True)
        self.pose_publisher = rospy.Publisher('/target_pose', Pose, queue_size=10)
        self.radius = radius
        self.frequency = frequency
        self.pose = Pose()

    def simulate_motion(self):
        rate = rospy.Rate(30)  # 30 Hz
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()

            # Set the position of the target pose, following a circular motion
            angle = 2 * np.pi * self.frequency * (current_time - start_time)
            self.pose.position.x = self.radius * np.cos(angle)
            self.pose.position.y = self.radius * np.sin(angle)
            self.pose.position.z = 1.0

            # Keep the orientation constant
            quaternion = quaternion_from_euler(0, 0, 0)
            self.pose.orientation.x = quaternion[0]
            self.pose.orientation.y = quaternion[1]
            self.pose.orientation.z = quaternion[2]
            self.pose.orientation.w = quaternion[3]
            self.pose_publisher.publish(self.pose)

            rate.sleep() # Sleep for the time remaining to hit the 30 Hz update rate

if __name__ == '__main__':
    try:
        simulator = SimulateTargetPose(radius=1.0, frequency=0.1)
        simulator.simulate_motion()
    except rospy.ROSInterruptException:
        pass