#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Example of online invariants calculation
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
import invariants_py.rockit_calculate_vector_invariants_position_mj as invariants_calculation
import std_msgs.msg
import helper_functions_ros
from nav_msgs.msg import Path
import rospkg
from invariants_py import data_handler as rw
from invariants_py import rockit_generate_position_from_vector_invariants as OCP_gen
import invariants_py.spline_handler as sh
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class ROSInvariantTrajectoryGeneration:
    def __init__(self, invariant_model_location):
        rospy.init_node('ros_trajectory_generation', anonymous=True)
        self.update_rate = rospy.Rate(20)  # Set the ROS node update rate (Default: 20 Hz)
        
        # Create a ROS topic subscribers and publishers
        rospy.Subscriber('/target_pose', Pose, self.callback_target_pose)
        self.publisher_traj_gen = rospy.Publisher('/trajectory', Marker, queue_size=10)
        self.publisher_traj_meas = rospy.Publisher('/target_pose_marker', Marker, queue_size=10)

        # Initialize invariant trajectory generation problem
        self.invariant_model = rw.read_invariants_from_csv(invariant_model_location)
        self.target_position = [1.1,0.0,0.0]

    def callback_target_pose(self, pose_msg):
        # Callback function to process the received Pose message
        self.target_position = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        
        # Create a Marker message for the sphere
        marker = Marker()
        marker.header = std_msgs.msg.Header()
        marker.header.stamp = rospy.Time.now() # add timestamp
        marker.header.frame_id = 'world' # add frame_id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        # Set marker properties
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        # Set marker position
        marker.pose.position.x = pose_msg.position.x
        marker.pose.position.y = pose_msg.position.y
        marker.pose.position.z = pose_msg.position.z
        # Publish the Marker
        self.publisher_traj_meas.publish(marker)

    def run(self):

        N = 50

        FS_online_generation_problem = OCP_gen.OCP_gen_pos(window_len=N, fatrop_solver=True, bounds_mf=False)

        # Resample model invariants to desired number of N samples
        spline_invariant_model = sh.create_spline_model(self.invariant_model[:,0], self.invariant_model[:,1:])
        progress_values = np.linspace(self.invariant_model[0,0],self.invariant_model[-1,0],N)
        model_invariants,progress_step = sh.interpolate_invariants(spline_invariant_model, progress_values)


        # Create the Marker message
        marker = Marker()
        marker.header = std_msgs.msg.Header()
        marker.header.frame_id = "world"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # Set marker properties
        marker.scale.x = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0

        while not rospy.is_shutdown():

            # Specify the boundary constraints
            boundary_constraints = {"position": {"initial": [0, 0, 0], "final": self.target_position}}

            # Generate trajectory
            invariants, traj, mf = FS_online_generation_problem.generate_trajectory_online(model_invariants,boundary_constraints,step_size=progress_step)

            # Add points to the marker
            marker.points = []
            for point in traj:  # Assuming traj is a numpy array of shape (N, 3)
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = point[2]
                marker.points.append(p)

            # Publish the Marker
            marker.header.stamp = rospy.Time.now() # add timestamp
            self.publisher_traj_gen.publish(marker)
            self.update_rate.sleep() # Sleep to maintain the specified update rate

if __name__ == '__main__':
    try:
        # Create and initialize ROS node
        rospack = rospkg.RosPack()
        model_filename = rospack.get_path("invariants_py_ros")+"/data/"+"sinus_invariants.csv"
        
        ros_traj_gen_node = ROSInvariantTrajectoryGeneration(model_filename)
        
        # Run the loop of the node
        ros_traj_gen_node.run()
    except rospy.ROSInterruptException:
        pass