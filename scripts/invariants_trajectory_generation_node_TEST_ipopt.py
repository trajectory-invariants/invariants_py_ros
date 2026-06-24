#!/usr/bin/env python3

import rospy
import json
import numpy as np
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as Rot
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from invariants_py.kinematics.orientation_kinematics import rot2quat
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
import std_msgs.msg
from invariants_py.kinematics.rigidbody_kinematics import orthonormalize_rotation as orthonormalize
from invariants_py.ocp_initialization import initial_trajectory_movingframe_rotation as FSr_init
from invariants_py.reparameterization import interpR
from invariants_py.generate_trajectory import opti_generate_pose_traj_from_vector_invars as OCP_gen
from ros_spline_fitting_trajectory.msg import Trajectory
import rospkg
from invariants_py import data_handler as dh
import helper_funtions as hf
from scipy.spatial.transform import Rotation as Rot
import time
import matplotlib.pyplot as plt
import casadi as cas
import invariants_py.dynamics_vector_invariants as dynamics
from invariants_py.ocp_helper import tril_vec, tril_vec_no_diag, extract_robot_params
import urdf2casadi.urdfparser as u2c

class OCP_results:

    def __init__(self,FSt_frames,FSr_frames,Obj_pos,Obj_frames,invariants):
        self.FSt_frames = FSt_frames
        self.FSr_frames = FSr_frames
        self.Obj_pos = Obj_pos
        self.Obj_frames = Obj_frames
        self.invariants = invariants


# class OCP_gen_pose:

    # def __init__(self, boundary_constraints, init_vals, invariant_model, step_size, weights_params, N = 40, bool_unsigned_invariants = False, solver = 'ipopt', robot_params = {}, dummy = {}):  
def generate_trajectory_ipopt(boundary_constraints, init_vals, invariant_model, step_size, weights_params, N = 40, bool_unsigned_invariants = False, solver = 'ipopt', robot_params = {}, dummy = {}):
    # Robot urdf location
    urdf_file_name = robot_params.get('urdf_file_name', None)
    path_to_urdf = dh.find_robot_path(urdf_file_name) 
    include_robot_model = True if path_to_urdf is not None else False
    if include_robot_model:
        nb_joints,q_limits,root,tip,q_init = extract_robot_params(robot_params,path_to_urdf,urdf_file_name)

    dummy_inv_sol = dummy.get('inv_sol', 0.001+np.zeros((N,6)))
    dummy_inv_demo = dummy.get('inv_demo', 0.001+np.zeros((N,6)))
    dummy_R_t = dummy.get('R_t',np.array([np.hstack(np.eye(3)) for i in range(N)]).reshape(N,3,3))
    dummy_R_r = dummy.get('R_r',np.array([np.hstack(np.eye(3)) for i in range(N)]).reshape(N,3,3))

    ''' Create decision variables and parameters for the optimization problem '''
    opti = cas.Opti() # use OptiStack package from Casadi for easy bookkeeping of variables 

    # Define system states X (unknown object pose + moving frame pose at every time step) 
    p_obj = []
    R_obj = []
    R_t = []
    R_r = []
    X = []
    q = []
    for k in range(N):
        R_r.append(opti.variable(3,3)) # rotational Frenet-Serret frame
        R_obj.append(opti.variable(3,3)) # object orientation
        R_t.append(opti.variable(3,3)) # translational Frenet-Serret frame
        p_obj.append(opti.variable(3,1)) # object position
        if include_robot_model:
            q.append(opti.variable(nb_joints,1))
        X.append(cas.vertcat(cas.vec(R_r[k]), cas.vec(R_obj[k]),cas.vec(R_t[k]), cas.vec(p_obj[k])))
    if include_robot_model:
        epsilon = opti.variable(3,1) # slack variable for gap closing constraint


    invars = []
    qdot = []
    for k in range(N-1):
        invars.append(opti.variable(6,1)) # invariants
        if include_robot_model:
            qdot.append(opti.variable(nb_joints,1))

    # Boundary values
    if "position" in boundary_constraints and "initial" in boundary_constraints["position"]:
        p_obj_start = opti.parameter(3,1)
    if "position" in boundary_constraints and "final" in boundary_constraints["position"]:
        p_obj_end = opti.parameter(3,1)
    if "orientation" in boundary_constraints and "initial" in boundary_constraints["orientation"]:
        R_obj_start = opti.parameter(3,3)
    if "orientation" in boundary_constraints and "final" in boundary_constraints["orientation"]:
        R_obj_end = opti.parameter(3,3)
    if "moving-frame" in boundary_constraints and "translational" in boundary_constraints["moving-frame"] and "initial" in boundary_constraints["moving-frame"]["translational"]:
        R_t_start = opti.parameter(3,3)
    if "moving-frame" in boundary_constraints and "translational" in boundary_constraints["moving-frame"] and "final" in boundary_constraints["moving-frame"]["translational"]:
        R_t_end = opti.parameter(3,3)
    if "moving-frame" in boundary_constraints and "rotational" in boundary_constraints["moving-frame"] and "initial" in boundary_constraints["moving-frame"]["rotational"]:
        R_r_start = opti.parameter(3,3)
    if "moving-frame" in boundary_constraints and "rotational" in boundary_constraints["moving-frame"] and "final" in boundary_constraints["moving-frame"]["rotational"]:
        R_r_end = opti.parameter(3,3)
    
    # Define system parameters P (known values in optimization that need to be set right before solving)
    h = opti.parameter(1,1) # step size for integration of dynamic model
    invars_demo = []
    w_invars = []
    for k in range(N-1):
        invars_demo.append(opti.parameter(6,1)) # model invariants
        w_invars.append(opti.parameter(6,1)) # weights for invariants
    if include_robot_model:
        q_lim = opti.parameter(nb_joints*2,1)


    ''' Specifying the constraints '''
    
    # Constrain rotation matrices to be orthogonal (only needed for one timestep, property is propagated by integrator)
    opti.subject_to(tril_vec(R_t[0].T @ R_t[0] - np.eye(3)) == 0)
    opti.subject_to(tril_vec(R_obj[0].T @ R_obj[0] - np.eye(3)) == 0)
    opti.subject_to(tril_vec(R_r[0].T @ R_r[0] - np.eye(3)) == 0)

    # Boundary constraints
    if "position" in boundary_constraints and "initial" in boundary_constraints["position"]:    
        opti.subject_to(p_obj[0] == p_obj_start)
    if "orientation" in boundary_constraints and "initial" in boundary_constraints["orientation"]:
        opti.subject_to(tril_vec_no_diag(R_obj[0].T @ R_obj_start - np.eye(3)) == 0.)
    if "moving-frame" in boundary_constraints and "translational" in boundary_constraints["moving-frame"] and "initial" in boundary_constraints["moving-frame"]["translational"]:
        opti.subject_to(tril_vec_no_diag(R_t[0].T @ R_t_start - np.eye(3)) == 0.)
    if "moving-frame" in boundary_constraints and "rotational" in boundary_constraints["moving-frame"] and "initial" in boundary_constraints["moving-frame"]["rotational"]:
        opti.subject_to(tril_vec_no_diag(R_r[0].T @ R_r_start - np.eye(3)) == 0.)
    if "position" in boundary_constraints and "final" in boundary_constraints["position"]:
        if include_robot_model:
            opti.subject_to(p_obj[-1] - p_obj_end == epsilon)
        else:
            opti.subject_to(p_obj[-1] == p_obj_end)
    if "orientation" in boundary_constraints and "final" in boundary_constraints["orientation"]:
        opti.subject_to(tril_vec_no_diag(R_obj[-1].T @ R_obj_end - np.eye(3)) == 0.)
    if "moving-frame" in boundary_constraints and "translational" in boundary_constraints["moving-frame"] and "final" in boundary_constraints["moving-frame"]["translational"]:
        opti.subject_to(tril_vec_no_diag(R_t[-1].T @ R_t_end - np.eye(3)) == 0.)
    if "moving-frame" in boundary_constraints and "rotational" in boundary_constraints["moving-frame"] and "final" in boundary_constraints["moving-frame"]["rotational"]:
        opti.subject_to(tril_vec_no_diag(R_r[-1].T @ R_r_end - np.eye(3)) == 0.)
        
    if include_robot_model:
        for k in range(N):
            for i in range(nb_joints):
                # ocp.subject_to(-q_lim[i] <= (q[k][i] <= q_lim[i])) # This constraint definition does not work with fatrop, yet
                opti.subject_to(q[k][i] >= q_lim[i])
                opti.subject_to(q[k][i] <= q_lim[nb_joints+i])

    # Dynamic constraints
    integrator = dynamics.define_integrator_invariants_pose(h)
    # p_obj_fwkin = np.zeros((3,N))
    # R_obj_fwkin = np.zeros((3,3,N))
    # integrator = dynamics.define_integrator_invariants_pose(h,include_robot_model)
    for k in range(N-1):
        # Integrate current state to obtain next state (next rotation and position)
        Xk_end = integrator(X[k],invars[k],h)
        # Gap closing constraint
        opti.subject_to(X[k+1]==Xk_end)
        if include_robot_model:
            opti.subject_to(q[k+1]==qdot[k]*h+q[k])

        
    # Forward kinematics
    if include_robot_model:
        ur10 = u2c.URDFparser()
        ur10.from_file(path_to_urdf)
        fk_dict = ur10.get_forward_kinematics(root, tip)
        robot_forward_kinematics = fk_dict["T_fk"]
        q_sim = cas.MX.sym('q',nb_joints,1)
        # pos_sim = cas.MX.sym('pos',3,1)
        # R_sim = cas.MX.sym('R',3,3)

        # T_sim = cas.vertcat(cas.horzcat(R_sim, pos_sim),)
        T_sim = robot_forward_kinematics(q_sim.T)
        pos_sim = T_sim[0:3,3]
        R_sim = T_sim[0:3,0:3]
        # p_obj_fwkin, R_obj_fwkin = robot_forward_kinematics(q[k],path_to_urdf,root,tip)
        fw_kin = cas.Function('fw_kin', [q_sim], [pos_sim, R_sim])
        for k in range(N):
            p_obj_fwkin, R_obj_fwkin =  fw_kin(q[k])
            opti.subject_to(p_obj[k] ==  p_obj_fwkin)
            opti.subject_to(tril_vec_no_diag(R_obj[k].T @ R_obj_fwkin - np.eye(3)) == 0.)       
            # opti.subject_to(tril_vec_no_diag(np.eye(3).T @ R_obj_fwkin - np.eye(3)) == 0.) # used for crate filling       

    ''' Specifying the objective '''

    # Fitting constraint to remain close to measurements
    objective_fit = 0
    for k in range(N-1):
        err_invars = w_invars[k]*(invars[k] - invars_demo[k])
        # err_invars = w_invars[k][3:]*(invars[k][3:] - invars_demo[k][3:]) # used for crate filling
        objective_fit += 1/N*cas.dot(err_invars,err_invars)
        # objective_fit += 1*cas.dot(err_invars,err_invars) # used for crate filling
    objective = objective_fit

    ''' Define solver and save variables '''
    opti.minimize(objective)

    if solver == 'ipopt':
        opti.solver('ipopt',{"print_time":True,"expand":True},{'max_iter':100,'tol':1e-6,'print_level':0,'ma57_automatic_scaling':'no','linear_solver':'mumps','print_info_string':'yes'})
    elif solver == 'fatrop':
        opti.solver('fatrop',{"expand":True,'fatrop.max_iter':300,'fatrop.tol':1e-6,'fatrop.print_level':5, "structure_detection":"auto","debug":True,"fatrop.mu_init":0.1})

    for k in range(N):
        opti.set_initial(R_t[k], init_vals["moving-frame"]["translational"][k])
        opti.set_initial(R_r[k], init_vals["moving-frame"]["translational"][k])
        if include_robot_model:
            p_obj_dummy, R_obj_dummy = fw_kin(q_init) #robot_forward_kinematics(q_init,path_to_urdf,root,tip)
            opti.set_initial(q[k],q_init)
            opti.set_value(q_lim,q_limits)
        else:
            p_obj_dummy = np.zeros(3)
            R_obj_dummy = np.eye(3)
        opti.set_initial(p_obj[k], init_vals["trajectory"]["position"][k])
        opti.set_initial(R_obj[k], init_vals["trajectory"]["orientation"][k])
    for k in range(N-1):
        opti.set_initial(invars[k], init_vals["invariants"][k]) 
        opti.set_value(invars_demo[k], invariant_model[k])
        if weights_params["w_high_active"] == 1:
            if k >= weights_params["w_high_start"] and k <= weights_params["w_high_end"]:
                opti.set_value(w_invars[k], weights_params["w_high_invars"])
            else:
                opti.set_value(w_invars[k], weights_params["w_invars"])
        else:
            opti.set_value(w_invars[k], weights_params["w_invars"])
        if include_robot_model:
            opti.set_initial(qdot[k], 0.001*np.ones((nb_joints)))
    opti.set_value(h,step_size)
    # Boundary constraints
    if "position" in boundary_constraints and "initial" in boundary_constraints["position"]:
        opti.set_value(p_obj_start, boundary_constraints["position"]["initial"])
    if "position" in boundary_constraints and "final" in boundary_constraints["position"]:
        opti.set_value(p_obj_end, boundary_constraints["position"]["final"])
    if "orientation" in boundary_constraints and "initial" in boundary_constraints["orientation"]:
        opti.set_value(R_obj_start, boundary_constraints["orientation"]["initial"])
    if "orientation" in boundary_constraints and "final" in boundary_constraints["orientation"]:
        opti.set_value(R_obj_end, boundary_constraints["orientation"]["final"])
        # opti.set_value(R_obj_end, rotate_x(np.pi/60) @ R_obj_dummy) # used for crate filling
    if "moving-frame" in boundary_constraints and "translational" in boundary_constraints["moving-frame"] and "initial" in boundary_constraints["moving-frame"]["translational"]:
        opti.set_value(R_t_start, boundary_constraints["moving-frame"]["translational"]["initial"])
    if "moving-frame" in boundary_constraints and "translational" in boundary_constraints["moving-frame"] and "final" in boundary_constraints["moving-frame"]["translational"]:
        opti.set_value(R_t_end, boundary_constraints["moving-frame"]["translational"]["final"])
    if "moving-frame" in boundary_constraints and "rotational" in boundary_constraints["moving-frame"] and "initial" in boundary_constraints["moving-frame"]["rotational"]:
        opti.set_value(R_r_start, boundary_constraints["moving-frame"]["rotational"]["initial"])
    if "moving-frame" in boundary_constraints and "rotational" in boundary_constraints["moving-frame"] and "final" in boundary_constraints["moving-frame"]["rotational"]:
        opti.set_value(R_r_end, boundary_constraints["moving-frame"]["rotational"]["final"])
    sol = opti.solve_limited()

    # Extract the solved variables
    invariants = np.array([sol.value(i) for i in invars])
    invariants =  np.vstack((invariants,[invariants[-1,:]]))
    p_obj_sol = np.array([sol.value(i) for i in p_obj])
    R_obj_sol = np.array([sol.value(i) for i in R_obj])
    R_t_sol = np.array([sol.value(i) for i in R_t])
    R_r_sol = np.array([sol.value(i) for i in R_r])
    joint_val = []

    return invariants, p_obj_sol, R_obj_sol, R_t_sol, R_r_sol, joint_val

class invariants_traj_gen_node:

    def __init__(self, name = "talker") -> None:

        rospy.init_node(name, anonymous=True)
        
        #Initialise variables 
        self.tf = None # [x,y,z,qx,qy,qz,qw]
        self.progress = 0
        self.progress_fv = 0
        self.current_progress_offset = 0
        self.progress_sum = 0
        self.progress_offset = 0
        self.counter = 0
        self.condition = 0
        self.dist_ratio = 1
        self.enter_recovery_mode = 1
        self.recovery_target = 100 * np.ones(7)
        self.current_sample = 0
        self.factor_execution_speed = 1
        self.jointvel = np.zeros(7)
        self.R_w_tgt = None
        self.pos_w_tgt = None
        self.bottle_pos_real = np.array([100,100,100]) # to understand if use real position measurements or simulated
        self.bottle_pos_sim = np.array([100,100,100])
        self.previous_pos_w_bottle = np.array([100,100,100])
        self.home = np.zeros(3)
        self.trajectory = Trajectory()

        rospack = rospkg.RosPack()
        #%%
        rospy.Subscriber('sim_bottle_pos_pub', Float64MultiArray, self.callback_bottle_pos_sim)
        rospy.Subscriber('/pickit/objects_wrt_robot_frame', Float64MultiArray, self.callback_bottle_pos_real) # THIS IS EXAMPLE OF BOTTLE POSITION DETECTED BY SENSOR - NOT IMPLEMENTED CORRECTLY YET (update also in listener_node)
        rospy.Subscriber('tcp_pose', Float64MultiArray, self.callback_tf)
        rospy.Subscriber('progress', Float64MultiArray, self.callback_progress) # I need some kind of progress, representing the progress of the robot along the trajectory
        rospy.Subscriber('progress_fv', Float64MultiArray, self.callback_progress_fv) # This should be the value of the progress feature variable
        rospy.Subscriber('progress_offset_previous', Float64MultiArray, self.callback_progress_offset_previous) # It's the progress offset of the previous sample
        rospy.Subscriber('factor_execution_speed', Float64MultiArray, self.callback_factorexecspeed)
        rospy.Subscriber('jointvel', Float64MultiArray, self.callback_jointvel)
        rospy.Subscriber('home_position', Float64MultiArray, self.callback_home)
        self.pub_node_output = rospy.Publisher('/raw_inv_gen_node_output', Float64MultiArray, queue_size=10, latch=True)
        self.publisher_trajectory = rospy.Publisher('/trajectory_pub', Trajectory, queue_size=10)
        self.publisher_traj_gen_marker = rospy.Publisher('/trajectory_marker', Marker, queue_size=10)
        #%% User inputs
        self.number_samples = 50
        self.threshold_new_target_measurement = 0.005
        self.delay_sample = 1
        self.max_inv_err = 40
        self.debug_mode = False


        # Solver choice
        self.solver = "fatrop" # options "ipopt", "fatrop"

        # OCP params
        # Robot parameters
        self.robot_params = {
            "urdf_file_name": None, # use None if do not want to include robot model
            # "q_init": np.array([-np.pi, -2.27, 2.27, -np.pi/2, -np.pi/2, np.pi/4]), # Initial joint values for UR10
            "q_init": np.array([-0.06999619209628122, -0.9936042374309739, 0.04052275688381114, -3.040355360901146, 0.024184582866498106, 2.0659148485780445, 0.7930461100688347]), # Initial joint values for Franka Panda
            "tip": 'TCP_frame' # Name of the robot tip (if empty standard 'tool0' is used)
        }
        # Define OCP weights
        self.initial_w_high_start = round(0.7*self.number_samples)
        self.weights_params = {
            "w_invars": 0.1*np.array([0.1*1, 0.1*1, 0.1*1, 5, 1.0, 1.0]),
            # "w_invars": 0.1*np.array([0.1*1, 0.1*1, 0.1*1, 50, 10.0, 10.0]), # to use when include robot kin model
            "w_high_start": self.initial_w_high_start,
            "w_high_end": self.number_samples,
            "w_high_invars": 0.5*np.array([0.1*1/5, 0.1*1/5, 0.1*1/5, 5, 1, 1]),
            # "w_high_invars": 0.5*np.array([0.1*1/5, 0.1*1/5, 0.1*1/5, 50, 10, 10]), # to use when include robot kin model
            "w_high_active": 1
        }

        # Parameters for orientation defintion
        self.quat_demo = np.array([0.907466,-0.416623,0.0328278,-0.0430416]) # orientation quaternion (defined as [qw,qx,qy,qz]) of the final pose of the human demonstration
        self.radius = 0.0145 + 0.015 # radius of the bottle (stella :0.0145, FM: 0.021) + buffer of 0.015
        self.alpha = np.array([np.pi/2]) # angle of approach direction around z-axis in rad

        # Choice of demonstration
        # Crate filling
        model_filename = rospack.get_path("invariants_py_ros")+"/data/"+"cf_inv.csv"
        pos_filename = rospack.get_path("invariants_py_ros")+"/data/"+"cf_pobj.csv"
        R_filename = rospack.get_path("invariants_py_ros")+"/data/"+"cf_Robj.csv"
        FSt_filename = rospack.get_path("invariants_py_ros")+"/data/"+"cf_FSt.csv"
        FSr_filename = rospack.get_path("invariants_py_ros")+"/data/"+"cf_FSr.csv"

        # Water pouring
        # model_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_inv.csv"
        # pos_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_pobj.csv"
        # R_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_Robj.csv"
        # FSt_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_FSt.csv"
        # FSr_filename = rospack.get_path("invariants_py_ros")+"/data/"+"pouring_FSr.csv"

        # Bottle approach
        # model_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_inv.csv"
        # pos_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_pobj.csv"
        # R_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_Robj.csv"
        # FSt_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_FSt.csv"
        # FSr_filename = rospack.get_path("invariants_py_ros")+"/data/"+"beer_FSr.csv"

        self.invariant_model = dh.read_invariants_from_csv(model_filename)
        self.demo_pos = np.loadtxt(pos_filename, delimiter=",")
        self.demo_pos -= self.demo_pos[0]
        loaded_Robj = np.loadtxt(R_filename, delimiter=",")
        self.demo_Robj = loaded_Robj.reshape(loaded_Robj.shape[0], loaded_Robj.shape[1] // 3, 3)
        loaded_FSt = np.loadtxt(FSt_filename, delimiter=",")
        self.demo_FSt = loaded_FSt.reshape(loaded_FSt.shape[0], loaded_FSt.shape[1] // 3, 3)
        loaded_FSr = np.loadtxt(FSr_filename, delimiter=",")
        self.demo_FSr = loaded_FSr.reshape(loaded_FSr.shape[0], loaded_FSr.shape[1] // 3, 3)

        # new constraints       
        alpha = 0
        rotate = R.from_euler('z', alpha, degrees=True)
        R_w_tgt =  orthonormalize(rotate.as_matrix() @ self.demo_Robj[-1])
        FSt_end = orthonormalize(rotate.as_matrix() @ self.demo_FSt[-1])

        # # Linear initialization
        R_obj_init = interpR(np.linspace(0, 1, self.number_samples), [0,1], np.array([np.eye(3), R_w_tgt]))

        R_r_init, R_r_init_array, invars_init = FSr_init(np.eye(3), R_w_tgt, N=self.number_samples)
        
        # Initialising the boundary constraints with zeros or I
        self.boundary_constraints = {
            "position": {
                "initial": np.zeros(3),
                "final": np.zeros(3)
            },
            "orientation": {
                "initial": np.eye(3),
                "final": np.eye(3)
            },
            "moving-frame": {
                "translational": {
                    "initial": np.eye(3),
                    "final": np.eye(3)
                },
                "rotational": {
                    "initial": np.eye(3),
                    "final": np.eye(3)
                }
            },
        }
        
        dummy = { "inv_sol": np.loadtxt(dh.find_data_path("dummy_sol.csv"),delimiter=","), 
          "inv_demo": np.loadtxt(dh.find_data_path("dummy_inv.csv"),delimiter=","), 
          "R_t": np.loadtxt(dh.find_data_path("dummy_R_t.csv"),delimiter=",").reshape(100,3,3), 
          "R_r": np.loadtxt(dh.find_data_path("dummy_R_r.csv"),delimiter=",").reshape(100,3,3)}
        
        self.new_traj = OCP_results(FSt_frames = [], FSr_frames = [], Obj_pos = 100*np.ones((self.number_samples,3)), Obj_frames = np.ones((self.number_samples,3,3))*np.eye(3), invariants = np.zeros((self.number_samples,6)))
        self.current_traj = OCP_results(FSt_frames = [], FSr_frames = [], Obj_pos = 100*np.ones((self.number_samples,3)), Obj_frames = np.ones((self.number_samples,3,3))*np.eye(3), invariants = np.zeros((self.number_samples,6)))

        self.FS_online_generation_problem = OCP_gen.OCP_gen_pose(self.boundary_constraints,self.number_samples,solver=self.solver,robot_params=self.robot_params, dummy=dummy, bool_unsigned_invariants=True)
        
        # Resample model invariants to desired number of self.number_samples samples
        progress_values,model_invariants,progress_step = hf.resample_invariants(self.invariant_model,self.progress,self.number_samples)

        self.initial_values = {
            "trajectory": {
                "position": np.array([np.interp(np.linspace(0,len(self.demo_pos)-1,num=self.number_samples),np.linspace(0,len(self.demo_pos)-1,num=len(self.demo_pos)),self.demo_pos[:,i]) for i in range(3)]).T, 
                "orientation": R_obj_init
            },
            "moving-frame": {
                "translational": interpR(np.linspace(0,1,self.number_samples),np.linspace(0,1,len(self.demo_pos)),self.demo_FSt),
                "rotational": R_r_init_array
            },
            "invariants": model_invariants,
            "joint-values": self.robot_params["q_init"] if self.robot_params["urdf_file_name"] is not None else {}
        }

        self.lookup_table,self.lookup_table_peak = hf.setup_heuristic_lookuptable(self.demo_pos)

        # Create the Marker message
        self.marker = Marker()
        self.marker.header = std_msgs.msg.Header()
        self.marker.header.frame_id = "world"
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD

        # Set self.marker properties
        self.marker.scale.x = 0.003
        self.marker.scale.y = 0.003
        self.marker.scale.z = 0.003
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0

        if self.pos_w_tgt is None:
            print("Waiting for a target pose to be published on /sim_bottle_pos_pub topic...")
            rospy.wait_for_message('/sim_bottle_pos_pub', Pose)
        if self.tf is None:
            print("Waiting for a start pose to be published on /tcp_pose topic...")
            rospy.wait_for_message('/tcp_pose', Pose)
            
    def callback_bottle_pos_sim(self,data):

        self.bottle_pos_sim = data.data

    def callback_bottle_pos_real(self,data):

        self.bottle_pos_real = data.data

    def callback_progress(self,data):

        self.progress = data.data[0]

    def callback_progress_fv(self,data):

        self.progress_fv = data.data[0]
    
    def callback_progress_offset_previous(self,data):

        self.current_progress_offset = data.data[0]
        
    def callback_tf(self,data):

        self.tf = data.data

    def callback_jointvel(self,data):

        self.jointvel = data.data

    def callback_factorexecspeed(self,data):

        self.factor_execution_speed = data.data[0]

    def callback_home(self,data):

        self.home = data.data

    def check_condition(self):
        if not self.tf[0] == 0:
            # Automatically detect if there is a real bottle position measurement or if the simulated one should be used
            if all(i == 100 for i in self.bottle_pos_real):
                self.pos_w_bottle = self.bottle_pos_sim
            else:
                self.pos_w_bottle = self.bottle_pos_real

            if all(np.isclose(self.pos_w_bottle,self.previous_pos_w_bottle,0,self.threshold_new_target_measurement)) and self.enter_recovery_mode == 0:
                self.condition = 1
            else:
                if self.debug_mode and self.enter_recovery_mode == 1:
                    print("New target detected, a new trajectory will be generated")
                self.condition = 0
                self.previous_pos_w_bottle = self.pos_w_bottle

    def generate_trajectory(self):
        if not self.tf[0] == 0 and not self.home[0] == 0:
            # Define the target pose, by adding the bottle radius distance to the bottle position measurement and a small offset to ensure no contact between opener and bottle
            # Rotate the target pose around the z-axis by the alpha_deg angle (currently fixed), defining the desired approach direction
            # FOR BOTTLE APPROACH
            # self.pos_w_tgt = np.array([self.pos_w_bottle[0]-self.radius*np.sin(self.alpha[0]),self.pos_w_bottle[1]+self.radius*np.cos(self.alpha[0]),self.pos_w_bottle[2]])
            # self.R_w_tgt = Rot.from_euler('z', self.alpha[0]).as_matrix() @ Rot.from_quat([self.quat_demo[1],self.quat_demo[2],self.quat_demo[3],self.quat_demo[0]]).as_matrix()
            # FOR CRATE FILLING
            self.pos_w_tgt = self.pos_w_bottle
            self.R_w_tgt = np.eye(3)
            quat_w_tgt = rot2quat(self.R_w_tgt.reshape(1,3,3)).reshape(4) # [qx,qy,qz,qw]
            self.pose_w_tgt = np.hstack([self.pos_w_tgt,np.array([quat_w_tgt[3],quat_w_tgt[0],quat_w_tgt[1],quat_w_tgt[2]])])
            
            if self.counter == 0:
                self.previous_target = self.pos_w_tgt

            self.current_sample = hf.find_current_sample(self.tf[:3],self.current_traj.Obj_pos,self.number_samples)

            # Predict the robot pose in 100ms (ros node rate) by taking the model pose in self.delay_sample sample(s)
            pos_w_tcp, R_w_tcp = hf.predict_robot_pose(self.delay_sample,self.jointvel,self.tf[:3],self.tf[3:],self.enter_recovery_mode,self.current_sample,self.current_traj.Obj_pos,self.current_traj.Obj_frames,self.progress)
            
            # s_prior_old,_ = hf.progress_heuristic_old(self.previous_target,pos_w_tcp,self.pos_w_tgt,self.progress_fv,self.current_progress_offset)
            s_prior,self.progress_sum = hf.progress_heuristic(self.pos_w_tgt,pos_w_tcp,self.progress_fv,self.current_progress_offset,self.lookup_table,self.lookup_table_peak,)
            if self.counter == 0:
                s_prior = 0
            # print("s_prior,counter,s_prior_old", s_prior, self.counter, s_prior_old)

            # new constraints TODO  
            alpha = 0
            rotate = R.from_euler('z', alpha, degrees=True)
            R_w_tgt =  orthonormalize(rotate.as_matrix() @ self.demo_Robj[-1])
            FSt_end = orthonormalize(rotate.as_matrix() @ self.demo_FSt[-1])

            # Resample model invariants to desired number of self.number_samples samples
            progress_values,model_invariants,progress_step = hf.resample_invariants(self.invariant_model,s_prior,self.number_samples)

            # Specify the boundary constraints
            self.boundary_constraints["position"]["initial"] = pos_w_tcp
            self.boundary_constraints["position"]["final"] = self.pos_w_tgt
            self.boundary_constraints["orientation"]["initial"] = R_w_tcp
            self.boundary_constraints["orientation"]["final"] = R_w_tgt # Start simple, where R_w_tgt is constant and equal to demo_Robj (for cf), TODO add variability in orientation!
            if self.counter == 0:
                self.boundary_constraints["moving-frame"]["translational"]["initial"] = orthonormalize(self.demo_FSt[round(self.progress*len(self.demo_pos))]) # round((self.current_sample)*len(self.demo_pos)/self.number_samples)
                self.boundary_constraints["moving-frame"]["rotational"]["initial"] = orthonormalize(self.demo_FSr[round(self.progress*len(self.demo_pos))]) # round((self.current_sample)*len(self.demo_pos)/self.number_samples)
            else:
                if self.enter_recovery_mode == 0:
                    self.boundary_constraints["moving-frame"]["translational"]["initial"] = orthonormalize(self.current_traj.FSt_frames[round(self.progress*self.number_samples)]) # self.current_sample+self.delay_sample
                    self.boundary_constraints["moving-frame"]["rotational"]["initial"] = orthonormalize(self.current_traj.FSr_frames[round(self.progress*self.number_samples)]) # self.current_sample+self.delay_sample
                else:
                    self.boundary_constraints["moving-frame"]["translational"]["initial"] = Rot.from_euler('Z', np.pi).as_matrix() @ orthonormalize(self.current_traj.FSt_frames[round(self.progress*self.number_samples)]) # Rot.from_euler('Z', np.pi).as_matrix() @  # [self.current_sample-self.delay_sample]
                    self.boundary_constraints["moving-frame"]["rotational"]["initial"] = orthonormalize(self.current_traj.FSr_frames[round(self.progress*self.number_samples)]) # self.current_sample-self.delay_sample
            self.boundary_constraints["moving-frame"]["translational"]["final"] = FSt_end
            self.boundary_constraints["moving-frame"]["rotational"]["final"] = self.demo_FSr[-1]

            self.initial_values["trajectory"]["position"] += self.home
            self.weights_params["w_high_start"] = round(((1-s_prior)-(1-self.initial_w_high_start/self.number_samples))/(1-s_prior)*self.number_samples) # ADAPT WHEN TO INCREASE WEIGHT BASED ON NEW PROGRESS?
            # print("NEW WEIGHT", self.weights_params["w_high_start"])

            if self.enter_recovery_mode == 1 and self.counter > 0:
                vector_tcp_to_tgt = self.pos_w_tgt[:2] - pos_w_tcp[:2]
                angle_tcp_to_tgt = np.arccos(np.dot(np.array([1,0]),vector_tcp_to_tgt)/(1*np.linalg.norm(vector_tcp_to_tgt)))
                for k in range(self.number_samples):
                    interp_initpos = np.array([np.interp(np.linspace(round(s_prior*len(self.demo_pos)),len(self.demo_pos)-1,num=self.number_samples),np.linspace(round(s_prior*len(self.demo_pos)),len(self.demo_pos)-1,num=len(self.demo_pos)-round(s_prior*len(self.demo_pos))),self.demo_pos[round(s_prior*len(self.demo_pos)):,i]) for i in range(3)]).T
                    self.initial_values["trajectory"]["position"][k,:] = Rot.from_euler('z', angle_tcp_to_tgt if vector_tcp_to_tgt[1] >= 0 else -angle_tcp_to_tgt).as_matrix() @  interp_initpos[k,:]
                    # self.initial_values["trajectory"]["position"] = np.array([np.interp(np.linspace(round(s_prior*len(self.demo_pos)),len(self.demo_pos)-1,num=self.number_samples),np.linspace(round(s_prior*len(self.demo_pos)),len(self.demo_pos)-1,num=len(self.demo_pos)-round(s_prior*len(self.demo_pos))),self.demo_pos[round(s_prior*len(self.demo_pos)):,i]) for i in range(3)]).T
                    # self.initial_values["trajectory"]["orientation"] =
                    interp_initR = interpR(np.linspace(s_prior,1,self.number_samples),np.linspace(s_prior,1,len(self.demo_pos)-round(s_prior*len(self.demo_pos))),self.demo_FSt[round(s_prior*len(self.demo_pos)):])
                    self.initial_values["moving-frame"]["translational"][k] = Rot.from_euler('z', angle_tcp_to_tgt if vector_tcp_to_tgt[1] >= 0 else -angle_tcp_to_tgt).as_matrix() @ interp_initR[k]
                    # self.initial_values["moving-frame"]["translational"] = interpR(np.linspace(s_prior,1,self.number_samples),np.linspace(s_prior,1,len(self.demo_pos)-round(s_prior*len(self.demo_pos))),self.demo_FSt[round(s_prior*len(self.demo_pos)):])
                    # self.initial_values["moving-frame"]["rotational"] =
                self.initial_values["invariants"] = model_invariants
                self.initial_values["trajectory"]["position"] -= self.initial_values["trajectory"]["position"][0]
                self.initial_values["trajectory"]["position"] += pos_w_tcp  # self.home
                if self.debug_mode:
                    A = self.initial_values["trajectory"]["position"][0,:]
                    print(f"s_prior= {s_prior},init_pos start = {A}, first point old traj {self.new_traj.Obj_pos[0,:]}")
                    B = self.boundary_constraints["position"]["initial"]
                    print(f"current sample = {self.current_sample}, bound constr pos init {B},home pos {self.home}")
                # self.initial_values["joint-values"]

            # Generate trajectory
            if self.debug_mode:
                print("")
                print("Generating new traj")
            self.new_traj.invariants, self.new_traj.Obj_pos, self.new_traj.Obj_frames, self.new_traj.FSt_frames, self.new_traj.FSr_frames, joint_values = generate_trajectory_ipopt(self.boundary_constraints,self.initial_values,model_invariants,progress_step,self.weights_params,self.number_samples,True)
            # self.new_traj.invariants, self.new_traj.Obj_pos, self.new_traj.Obj_frames, self.new_traj.FSt_frames, self.new_traj.FSr_frames, joint_values, inv_err = self.FS_online_generation_problem.generate_trajectory(model_invariants,self.boundary_constraints,progress_step,self.weights_params,self.initial_values,output_inverr=True,recovery_mode=self.enter_recovery_mode)
            inv_err = 0.01
            if self.debug_mode:
                print(f"Invariant error = {inv_err}")

            # # Plot invariants signature of first trajectory vs demonstration
            # if self.counter == 1:
            #     print(self.new_traj.Obj_pos)
            #     hf.plot_invariants(self.invariant_model,progress_values,self.new_traj.invariants)
            #     # hf.plot_invariants(self.current_traj.invariants,np.linspace(0,1,self.number_samples),self.new_traj.invariants)
            #     fig = plt.figure()
            #     ax3d = fig.add_subplot(111, projection='3d')
            #     ax3d.plot(self.new_traj.Obj_pos[:,0],self.new_traj.Obj_pos[:,1],self.new_traj.Obj_pos[:,2],'o')
            #     plt.show()

            if np.linalg.norm(self.pos_w_tgt - self.new_traj.Obj_pos[-1]) < 0.01 and inv_err < self.max_inv_err and np.linalg.norm(self.new_traj.Obj_pos[0,:]-pos_w_tcp) < 0.02:
                if self.debug_mode:
                    print("The trajectory fits the requirements - sent to the controller")
                self.enter_recovery_mode = 0
                self.recovery_target = self.pose_w_tgt.copy()
                self.progress_offset = self.current_progress_offset + s_prior - self.progress_sum
                # Publish trajectory
                self.trajectory = Trajectory()
                quaternion = rot2quat(self.new_traj.Obj_frames)
                for i in range(self.new_traj.invariants.shape[0]):
                    pose = Pose()
                    pose.position.x = self.new_traj.Obj_pos[i,0]
                    pose.position.y = self.new_traj.Obj_pos[i,1]
                    pose.position.z = self.new_traj.Obj_pos[i,2]
                    pose.orientation.x = quaternion[i,0]
                    pose.orientation.y = quaternion[i,1]
                    pose.orientation.z = quaternion[i,2]
                    pose.orientation.w = quaternion[i,3]
                    self.trajectory.poses.append(pose)

                self.current_traj.Obj_pos = self.new_traj.Obj_pos.copy()
                self.current_traj.invariants = self.new_traj.invariants.copy()
                self.current_traj.FSt_frames = self.new_traj.FSt_frames.copy()
                self.current_traj.FSr_frames = self.new_traj.FSr_frames.copy()
            else:
                if self.debug_mode:
                    print("ENTERING RECOVERY MODE because:")
                    if not np.linalg.norm(self.pos_w_tgt - self.new_traj.Obj_pos[-1]) < 0.01:
                        print(f"- New trajectory doesn't reach the target, dist = {np.linalg.norm(self.pos_w_tgt - self.new_traj.Obj_pos[-1])} > 0.01")
                    if inv_err >=self.max_inv_err:
                        print(f"- Invariants error = {inv_err} >= {self.max_inv_err}")
                    if not np.linalg.norm(self.new_traj.Obj_pos[0,:]-pos_w_tcp) < 0.02:
                        print(f"- New trajectory doesn't start at pos_w_tcp, dist = {np.linalg.norm(self.new_traj.Obj_pos[0,:]-pos_w_tcp)} > 0.02")
                self.enter_recovery_mode = 1
                self.pose_w_tgt = self.recovery_target.copy()
            if self.debug_mode:
                print(f"s_prior = {s_prior}, dist initial point new traj to estimated tcp {np.linalg.norm(self.new_traj.Obj_pos[0,:]-pos_w_tcp)}")
                print(f"actual tcp pos = {self.tf[:3]},estimated tcp pos = {pos_w_tcp}, initial point new traj = {self.new_traj.Obj_pos[0,:]}")
                print("")

            # Add points to the marker
            self.marker.points = []
            for point in self.new_traj.Obj_pos:  # Assuming traj is a numpy array of shape (self.number_samples, 3)
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = point[2]
                self.marker.points.append(p)

            # Publish the Marker
            self.marker.header.stamp = rospy.Time.now() # add timestamp
            self.publisher_traj_gen_marker.publish(self.marker)

            self.counter += 1

            self.previous_target = self.pos_w_tgt

            if self.counter > 0:
                self.publisher_trajectory.publish(self.trajectory)

            outputs = np.hstack((self.progress_offset, self.pose_w_tgt, self.enter_recovery_mode, self.alpha))

            self.pub_node_output.publish(Float64MultiArray(data=outputs))

    def reset_OCP(self):
        if not self.tf[0] == 0:
            if self.home[0] == 0 and self.counter > 0:
                if self.debug_mode:
                    print("RESETTING OCP",self.home)
                self.FS_online_generation_problem = OCP_gen.OCP_gen_pose(self.boundary_constraints,self.number_samples,solver=self.solver,robot_params=self.robot_params)
                self.counter = 0
                self.enter_recovery_mode = 1



if __name__ == '__main__':

    inv_node = invariants_traj_gen_node("talker")

    # This defines the rate at which the node should publish
    sampling_time_ms = 150 #ms
    sampling_time = sampling_time_ms/1000
    rate = rospy.Rate(1/sampling_time) # 10hz
    while not rospy.is_shutdown():
        starttime = time.time()
        inv_node.check_condition()
        if inv_node.condition == 0:
            inv_node.generate_trajectory()
        else:
            inv_node.reset_OCP()
        endtime = time.time()
        if endtime-starttime > sampling_time:
            print(f"WARNING! The node calculations exceeded the allocated {sampling_time*1000}ms. Total time: {endtime-starttime} s")
        rate.sleep()