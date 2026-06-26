import numpy as np
from invariants_py.kinematics.orientation_kinematics import quat2rot, rot2quat
import invariants_py.spline_handler as sh
import matplotlib.pyplot as plt
from invariants_py.reparameterization import interpR

def find_current_sample(tf_pos,current_traj_pos,N=50):
    # Find the sample on the current trajectory which corresponds to the current robot pose
    dist_robot_to_model = np.array([np.linalg.norm(tf_pos - current_traj_pos[i,:]) for i in range(N)])
    current_sample = np.argmin(dist_robot_to_model)
    
    return current_sample

def predict_robot_pose(delay_sample,jointvel,tf_pos,tf_quat,recovery_mode,current_sample,pos_current_traj,R_current_traj,current_progress,N=50):
    # Predict the robot pose in 100ms (ros node rate) by taking the model pose in delay_sample sample(s)
    if all(abs(jointvel[i]) < 1e-6 for i in range(len(jointvel))):
        pos_w_tcp = np.array([tf_pos[0],tf_pos[1],tf_pos[2]])
        R_w_tcp = quat2rot(np.hstack([tf_quat[0], tf_quat[1], tf_quat[2],tf_quat[3]]).reshape(1,4)).reshape(3,3)
    else:
        # if recovery_mode == 0:
        #     if current_sample + delay_sample>= len(pos_current_traj):
        #         pos_w_tcp = pos_current_traj[current_sample]
        #         R_w_tcp = R_current_traj[current_sample]
        #     else:
        #         pos_w_tcp = pos_current_traj[current_sample+delay_sample]
        #         R_w_tcp = R_current_traj[current_sample+delay_sample]
        # else:
        #     if current_sample - delay_sample -1 < 0:
        #         pos_w_tcp = pos_current_traj[0]
        #         R_w_tcp = R_current_traj[0]
        #     else:
        #         pos_w_tcp = pos_current_traj[current_sample-delay_sample-1]
        #         R_w_tcp = R_current_traj[current_sample-delay_sample-1]
        # pos_w_tcp_test = np.array([np.interp(current_sample/50+0.08,np.linspace(0,1,num=50),pos_current_traj[:,i]) for i in range(3)])
        pos_w_tcp = np.array([np.interp(current_progress,np.linspace(0,1,num=N),pos_current_traj[:,i]) for i in range(3)])
        R_w_tcp = np.array([interpR([current_progress],np.linspace(0,1,num=N),R_current_traj)]).reshape(3,3)
        # print(pos_w_tcp,pos_w_tcp_test)

    return pos_w_tcp, R_w_tcp

def progress_heuristic_old(previous_target,pos_w_tcp,pos_w_tgt,progress_fv,current_progress_offset):
    # Calculate s_prior by comparing the distance of the current robot pose to the previous target and to the current target
    dist_to_prev_target = np.linalg.norm(previous_target - pos_w_tcp)
    dist_to_target = np.linalg.norm(pos_w_tgt - pos_w_tcp)
    progress_sum = progress_fv + current_progress_offset
    dist_ratio = dist_to_target / dist_to_prev_target
    s_prior = progress_sum + progress_sum * (1 - dist_ratio)
    if s_prior < 0:
        s_prior = 0
    elif s_prior > 0.75:
        s_prior = 0.75

    return s_prior,progress_sum

def setup_heuristic_lookuptable(demo_pos):
    lookup_table = np.array([np.linalg.norm(demo_pos[i,:]-demo_pos[-1,:]) for i in range(len(demo_pos))])
    peak = np.argmax(lookup_table)

    return lookup_table,peak

def progress_heuristic(pos_w_tgt,pos_w_tcp,progress_fv,current_progress_offset,lookup_table,peak):
    progress_sum = progress_fv + current_progress_offset
    dist_to_new_target = np.linalg.norm(pos_w_tgt - pos_w_tcp)
    if round(progress_sum*len(lookup_table)) >= peak:
        s_prior = (np.argmin(np.array([round(abs(lookup_table[peak+i]-dist_to_new_target),3) for i in range(len(lookup_table)-peak)])) + peak)/len(lookup_table)
    else:
        s_prior = (np.argmin(np.array([round(abs(lookup_table[i]-dist_to_new_target),3) for i in range(peak)])))/len(lookup_table)

    s_prior -= 0.1 # TESTING

    return s_prior,progress_sum

def resample_invariants(invariant_model,progress,N=50):
    # Resample model invariants to desired number of self.number_samples samples
    spline_invariant_model = sh.create_spline_model(invariant_model[:,0], invariant_model[:,1:])
    progress_values = np.linspace(progress,invariant_model[-1,0],N)
    model_invariants,progress_step = sh.interpolate_invariants(spline_invariant_model, progress_values)

    return progress_values, model_invariants, progress_step

def plot_invariants(invariant_model,progress_values,new_traj_invariants):
    arclength_n = np.linspace(0,invariant_model[-1,0],len(invariant_model))
    fig = plt.figure()
    plt.subplot(2,3,1)
    plt.plot(arclength_n,invariant_model[:,1],'b')
    plt.plot(progress_values,new_traj_invariants[:,0],'r')
    plt.plot(0,0)
    plt.title('i_r1')

    plt.subplot(2,3,2)
    plt.plot(arclength_n,invariant_model[:,2],'b')
    plt.plot(progress_values,new_traj_invariants[:,1],'r')
    plt.plot(0,0)
    plt.title('i_r2')

    plt.subplot(2,3,3)
    plt.plot(arclength_n,invariant_model[:,3],'b')
    plt.plot(progress_values,new_traj_invariants[:,2],'r')
    plt.plot(0,0)
    plt.title('i_r3')

    plt.subplot(2,3,4)
    plt.plot(arclength_n,invariant_model[:,4],'b')
    plt.plot(progress_values,new_traj_invariants[:,3],'r')
    plt.plot(0,0)
    plt.title('i_t1')

    plt.subplot(2,3,5)
    plt.plot(arclength_n,invariant_model[:,5],'b')
    plt.plot(progress_values,new_traj_invariants[:,4],'r')
    plt.plot(0,0)
    plt.title('i_t2')

    plt.subplot(2,3,6)
    plt.plot(arclength_n,invariant_model[:,6],'b')
    plt.plot(progress_values,new_traj_invariants[:,5],'r')
    plt.plot(0,0)
    plt.title('i_t3')

    plt.show()