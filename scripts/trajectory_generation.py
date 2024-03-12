#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Example of online invariants calculation
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped
import invariants_py.rockit_frenetserret_calculation_minimumjerk as invariants_calculation
import std_msgs.msg
import helper_functions_ros
from nav_msgs.msg import Path
import rospkg
from invariants_py import read_and_write_data as rw
from invariants_py import rockit_class_frenetserret_generation_position as OCP_gen
import invariants_py.spline_handler as sh

class ROSInvariantTrajectoryGeneration:
    def __init__(self, invariant_model_location):
        rospy.init_node('ros_trajectory_generation', anonymous=True)
        self.update_rate = rospy.Rate(30)  # Set the ROS node update rate (Default: 20 Hz)
        
        # Create a ROS topic subscribers and publishers
        rospy.Subscriber('/target_pose', Pose, self.callback_target_pose)
        self.publisher_traj_gen = rospy.Publisher('/trajectory', Path, queue_size=10)

        # Initialize invariant trajectory generation problem
        self.invariant_model = rw.read_invariants_from_csv(invariant_model_location)
        self.target_position = [1.0,1.0,1.0]

    def callback_target_pose(self, pose_msg):
        # Callback function to process the received Pose message
        self.target_position = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        
    def run(self):

        N = 40

        FS_online_generation_problem = OCP_gen.FrenetSerret_gen_pos(window_len=N, fatrop_solver=True, bounds_mf=False)

        # Resample model invariants to desired number of N samples
        spline_invariant_model = sh.create_spline_model(self.invariant_model[:,0], self.invariant_model[:,1:])
        progress_values = np.linspace(self.invariant_model[0,0],self.invariant_model[-1,0],N)
        model_invariants,progress_step = sh.interpolate_invariants(spline_invariant_model, progress_values)
        
        #print(model_invariants)
        
        # Function to be run in a loop at the specified update rate
        while not rospy.is_shutdown():
            
            # Specify the boundary constraints
            boundary_constraints = {"position": {"initial": [0, 0, 0], "final": self.target_position}}

            # Generate trajectory
            invariants, traj, mf = FS_online_generation_problem.generate_trajectory_online(model_invariants,boundary_constraints,step_size=progress_step)

            print(invariants)

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