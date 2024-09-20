#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

class SimulateTargetPose:
    def __init__(self, radius, frequency):
        rospy.init_node('simulate_target_pose', anonymous=True)
        self.pose_publisher = rospy.Publisher('/target_pose_pub', Pose, queue_size=10)
        self.radius = radius
        self.frequency = frequency
        self.pose = Pose()

    def simulate_motion(self):
        rate = rospy.Rate(50)  # 30 Hz
        start_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()

            # Set the position of the target pose, following a circular motion
            angle = 2 * np.pi * self.frequency * (current_time - start_time)
            if current_time - start_time > 12: #13
                self.pose.position.x = 0.5288147543322843758 #0.9056+self.radius * np.cos(angle)
                self.pose.position.y = -0.35021277091361127431 #0.0635+self.radius * np.sin(angle)
                self.pose.position.z = 0.2434438793779414023 #0.441+0.1
            else:
                self.pose.position.x = 0.4788147543322843758 
                self.pose.position.y = -0.1021277091361127431 
                self.pose.position.z = 0.2434438793779414023 

            # Keep the orientation constant
            # quaternion = quaternion_from_euler(0, 0, 0)    
            self.pose.orientation.x = -8.155408847887090085e-01 #-0.416769310582221 #quaternion[0]
            self.pose.orientation.y = -4.793533449288836512e-01 #0.034580327044172 #quaternion[1]
            self.pose.orientation.z = 2.993281992424272064e-01 #-0.0419729444377253 #quaternion[2]
            self.pose.orientation.w = -1.245634981919912510e-01 #0.907384050264036 #quaternion[3]
            self.pose_publisher.publish(self.pose)

            rate.sleep() # Sleep for the time remaining to hit the 30 Hz update rate

if __name__ == '__main__':
    try:
        simulator = SimulateTargetPose(radius=0, frequency=0.1)
        simulator.simulate_motion()
    except rospy.ROSInterruptException:
        pass