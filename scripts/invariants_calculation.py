#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from invariants_py import YourInvariantCalculator  # Replace with the actual module name

class ROSInvariantsCalculation:
    def __init__(self):
        rospy.init_node('ros_invariants_calculation', anonymous=True)

        # Create a subscriber for the geometry_msgs/Pose topic
        rospy.Subscriber('/your_pose_topic', Pose, self.pose_callback)

        # Create a publisher for the result
        self.result_publisher = rospy.Publisher('/invariant_result', Float64, queue_size=10)  

        # Create an instance of your invariant calculator
        self.invariant_calculator = YourInvariantCalculator()  # Replace with the actual class name

        # Set the update rate
        self.update_rate = rospy.Rate(20)  # 20 Hz

    def pose_callback(self, pose_msg):
        # Callback function to process the received Pose message
        # Extract relevant information from the Pose message and perform calculations
        # For example:
        x_position = pose_msg.position.x
        y_position = pose_msg.position.y
        z_position = pose_msg.position.z


    def callback_update_window_measurements(self, pose_msg):
        "Update the window of position measurements"
        
        # Check if enough progress has passed for the next measurement to be included in the window
        if self.progress_etasl > self.progress_trigger:
        
            # Check if this is the very first measurement
            if self.window_measured_positions[-1,0] == 0:
                # Add new data as the last sample N of the window
                self.window_measured_positions[-1] = np.array((pose_msg.position.x,pose_msg.position.y,pose_msg.position.z))
        
            # Add new measurement unless it is a duplicate value
            elif data.position.x != self.window_measured_positions[-1,0] and pose_msg.position.y != self.window_measured_positions[-1,1] and data.position.z != self.window_measured_positions[-1,2]:            	
                # Push all samples in the window one place to the back + discard the first one
                self.window_measured_positions[0:-1] = self.window_measured_positions[1:]
                # Add new data as the last sample N of the window
                self.window_measured_positions[-1] = np.array((pose_msg.position.x,pose_msg.position.y,pose_msg.position.z))
            else:
                print("skipping data because of duplicates")
                
            # Set trigger for next time
            self.progress_trigger = self.progress_etasl + self.stepsize     

            # Report the number of nonzero samples in the window
            number_window_samples = np.count_nonzero(self.window_measured_positions,0)[0]
            if number_window_samples != self.window_nb_samples:
                print(f"filling window: {number_window_samples} out of {self.window_nb_samples}")
    

    def run(self):
        # Function to be run in a loop at the specified update rate
        while not rospy.is_shutdown():
            
            # Call the function from your invariant calculator
            result = self.invariant_calculator.calculate_invariant(x_position, y_position, z_position)
           
            # Publish the result
            self.result_publisher.publish(result)
		
            # Sleep to maintain the specified update rate
            self.update_rate.sleep()

if __name__ == '__main__':
    try:
        ros_invariants_node = ROSInvariantsCalculation()
        ros_invariants_node.run()
    except rospy.ROSInterruptException:
        pass


























#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Main script for progress estimation, can be used both for real application and offline testing (20 Hz)
"""

# Package management. This is necessary for ROS to find the Python modules below.
import sys, rospkg
rospack = rospkg.RosPack()
sys.path.append(rospack.get_path('contour_progress_estimation') + '/contour_following/')

import time
import numpy as np
import rospy

from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, Float64MultiArray
import contour_following.helper_functions_ros as helper_functions_ros

import contour_following.helper_functions as helper_functions
from contour_following.invariants_pf_velocity_input import ProgressModelROS2
import invariants_py.class_frenetserret_calculation_minimumjerk as FS_optistack
import invariants_py.rockit_frenetserret_calculation_minimumjerk as FS_rockit

class ProgressEstimator:

    def __init__(self, invariant_model_file_name, contour_model):
   
        ''' Subscribers '''
        rospy.Subscriber("pose_robot_toolframe",Pose, self.callback_update_window_measurements)
        rospy.Subscriber('progress_etasl',Float64MultiArray,self.callback_progress_variable)
        rospy.Subscriber('current_velocity',Float64MultiArray,self.callback_velocity)
        
        ''' Publishers '''
        self.publisher_progress_PF = rospy.Publisher('progress_PF', Float64, queue_size=10) # progress estimated by particle filter
        self.publisher_curvature_PF = rospy.Publisher('curvature_PF', Float64, queue_size=10) # curvature estimated by particle filter
        self.publisher_curvature_inv = rospy.Publisher('curvature_inv', Float64, queue_size=10) # curvature calculated by invariants
        self.publisher_point_cloud = rospy.Publisher('augmented_point_particles', PointCloud2, queue_size=10)
        #self.publisher_estimated_value = rospy.Publisher('estimated_value', Float64MultiArray, queue_size=10)
        self.publisher_trajectory = rospy.Publisher('estimated_trajectory', Float64MultiArray, queue_size=10) # trajectory calculated by invariants
        # self.publisher_particles = rospy.Publisher('particles', Float32MultiArray, queue_size=10)
        # self.publisher_augmented_particles = rospy.Publisher('augmented_particles', Float32MultiArray, queue_size=10)
        # self.publisher_true_value = rospy.Publisher('true_value', Float64MultiArray, queue_size=10)
        
        ''' Parameters of invariants calculation '''
        OCP_specification = 1 # {0 : OptiStack (old), 1 : Rockit (new)}
        solver = 1 # {0 : ipopt, 1 : Fatrop}, note that Fatrop is only available for OCP_specification = Rockit
        weight_measurements = 1
        weight_regularization = 10**-12
        self.window_nb_samples = 21 # number of samples in window
        self.window_length = 0.10 # length of the window in [m], currently 0.05m
        self.model_nb_samples = 250 # number of samples in invariant model
        self.stepsize = self.window_length/self.window_nb_samples # interval between samples in window [m]
        self.window_mid_sample = int(round(self.window_nb_samples/2,0)) # index of middle sample in window
        
        ''' Parameters of application '''
        freq = 20.0 # how fast the estimator can work, maximum value 20 Hertz from experiments
        self.rate = rospy.Rate(freq)
        self.dt = 1/freq
        
        ''' Initialization of class variables'''
        self.progress_etasl = 0 # integral of tangential speed returned by etasl
        self.progress_trigger = 0 # for checking when a sample must be added to window
        self.window_measured_positions = np.zeros((self.window_nb_samples,3))
        self.spline_model_trajectory = contour_model
        self.now_time = 0 # DEBUGGING, for checking timing of component
        self.previous_time = 0 # DEBUGGING, for checking timing of component
        
        ''' Initialize particle filter component '''
        print(f"invariant_model_file_name {invariant_model_file_name}")
        self.inv = ProgressModelROS2(model_file_location=invariant_model_file_name, # invariant spline
                                     contour_model=contour_model, # trajectory spline
                                     initial_vel=0, # initial tangential velocity of robot
                                     num_of_particles=1000, # number of particles
                                     resampling_algo='systematic', # systematic resampling
                                     sigmaY=0.3, sigmaX=0.000000001, # covariance of measurement / proces
                                     dt=self.dt) # timestep between estimates 
        
        ''' Initialize invariants estimation component '''
        if OCP_specification == 0:
            self.invariants_optim_problem = FS_optistack.FrenetSerret_calc(window_len=self.window_nb_samples, w_pos = weight_measurements, w_regul = weight_regularization, planar_task = True)
        elif OCP_specification == 1:
            self.invariants_optim_problem = FS_rockit.FrenetSerret_calc(nb_samples=self.window_nb_samples, w_pos = weight_measurements, w_regul_jerk = weight_regularization, fatrop_solver = solver)
                
    def loop_estimator(self):
        "Continuously run the estimator: (1) calculate invariants on current window of measurements, (2) use particle filter to estimate progress"
        
        while not rospy.is_shutdown():         
            
            # # [DEBUGGING] check timing of estimator
            # self.now_time = time.time()      
            # freq_reported = 1/(self.now_time - self.previous_time)
            # print(f"Estimator frequency: {freq_reported}")
            # self.previous_time = self.now_time

            # Window of measurements is not yet full
            if self.window_measured_positions[0,0] == 0:            
                #print('window of measurements not full yet') #rospy.loginfo("window of measurements not full yet")                
                # Set all results to zero
                progress_estimated = 0.0
                curvature_estimated_pf = 0.0
                curvature_estimated_inv = 0.0
                particles = [0]
                estimated_trajectory = helper_functions_ros.convert_nparray_to_Float64MultiArray(np.zeros((self.window_nb_samples,3)))
            else:      
                #print(f"Window of measurements: {self.window_measured_positions}")
                
                # Calculate invariants for window of measurements
                #start_time = time.time()
                invariants_online, trajectory_online, mf = self.invariants_optim_problem.calculate_invariants_online(self.window_measured_positions, self.stepsize, 1)
                #end_time = time.time()
                #print(f"Solution time invariants: {end_time - start_time}")
                #print(f"Calculated trajectory: {trajectory_online}")
                #print(f"Calculated invariants: {invariants_online}")
                
                # Retrieve geometric curvature at end of window, found as curvature rate / velocity, units: [rad/m] = [rad/s] / [m/s]
                invariants_online[:,1] = invariants_online[:,1]/invariants_online[:,0] # to get geometric curvature [rad/m]
                invariants_online[:,2] = invariants_online[:,2]/invariants_online[:,0] # to get geometric torsion [rad/m]
                
                # Middle of window is more reliable
                #trajectory_mid_window = trajectory_online[self.window_mid_sample,:]
                invariants_mid_window = invariants_online[self.window_mid_sample,:]
                curvature_estimated_inv =  invariants_mid_window[1]

                # Convert the online trajectory in a feasible data type for Publisher
                estimated_trajectory = helper_functions_ros.convert_nparray_to_Float64MultiArray(trajectory_online)

                # Estimate progress with particle filter
                # TODO: Use self.velocity in particle filter. Watch out for the values 10^(-6) published on the topic regularly.
                particle_filter_result = self.inv.callback_noisy_Kappa(observation=curvature_estimated_inv,velocity_input=self.velocity)
                progress_estimated = particle_filter_result['max_likelihood_estimate_of_progress'] + self.window_length/2 # correct for fact we are in middle of window
                curvature_estimated_pf = particle_filter_result['weighted_average_curvature'] 
                particles = particle_filter_result['progress_particles'] 
                #max_likelihood = particle_filter_result['max_likelihood_estimate_of_progress']
                #print(f"Estimated progress: {progress_estimated}")
                rospy.loginfo("Estimated progress: %f"%progress_estimated)

            # Publish results
            augmented_particle_data = []
            augmented_particle_data.append(self.window_length/2)
            augmented_particle_data.append(curvature_estimated_pf)
            augmented_particle_data.append(curvature_estimated_inv)
            estimated_value = self.spline_model_trajectory(progress_estimated)[0:3]
            #print(f"Estimated progress: {estimated_value}")
            augmented_particle_data.extend(estimated_value)
            augmented_particle_data.extend(particles)
            self.publisher_point_cloud.publish(helper_functions_ros.pack_data(augmented_particle_data))
            self.publisher_curvature_inv.publish(curvature_estimated_inv)
            self.publisher_progress_PF.publish(progress_estimated)           
            self.publisher_curvature_PF.publish(curvature_estimated_pf) 
            self.publisher_trajectory.publish(estimated_trajectory) 
            #estimated_value_msg = Float64MultiArray()
            #estimated_value_msg.data = estimated_value
            #self.publisher_estimated_value.publish(estimated_value_msg)

            # Make loop run at fixed frequency/rate
            self.rate.sleep()


    def callback_progress_variable(self,progress_etasl):
        '''Save ROS topic message to class property (measured path length of robot)'''
        
        # If new value is smaller than current value, reset trigger (this can happen when you use pre-recorded data)
        if progress_etasl.data[0] < self.progress_etasl:
            self.progress_trigger = 0
    
        self.progress_etasl = progress_etasl.data[0]
        # Print value for debugging
        #print(f"progress_etasl: {progress_etasl}")
        #rospy.loginfo("etaslprogress %f/%f"%self.progress_etasl%self.window_length)#print(self.progress_etasl)

    def callback_velocity(self,velocity):
        '''Save ROS topic message to class property (current velocity of robot)'''   
        self.velocity = velocity.data[0]

if __name__ == '__main__':

    ''' Choose contour: 0=Rob, 1=Sabine, 2=Murtaza(convex) '''
    contour_number = 2  # {0 : Rob's contour, 1 : Sabine's contour, 2 : Murtaza's contour}
    contour_positions_filename, invariant_model_filename = helper_functions.select_contour(contour_number)

    print(f"invariant_model_file_name={invariant_model_filename}")
    spline_model_trajectory = helper_functions.load_spline_model_trajectory(contour_positions_filename)[0]
    
    try:
        # Initialize ROS node
        if not rospy.core.is_initialized():
            rospy.init_node('ros_progress_estimator', anonymous=True)

        # Initialize instance of estimator class using the provided model of the contour   
        estimator = ProgressEstimator(invariant_model_filename, contour_model=spline_model_trajectory)
        estimator.loop_estimator()      

    except Exception as e:    
        print(f"either contour model not found or another problem occured: {e}")
        pass