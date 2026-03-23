import numpy as np
from matplotlib import pyplot as plt

pos_filename = "/home/riccardo/inv_ws/src/invariants_py_ros/data/cf_pobj.csv"

demo_pos = np.loadtxt(pos_filename, delimiter=",")

# Definition of lookup table
lookup_table = np.array([np.linalg.norm(demo_pos[i,:]-demo_pos[-1,:]) for i in range(len(demo_pos))])
# lookup_table = np.array([np.linalg.norm(self.initial_values["trajectory"]["position"][i,:]-self.demo_pos[-1,:]) for i in range(self.number_samples)])

# Old  heuristic
for k in range(10):
    distance_old_afterpeak = lookup_table[k*10]
    progress_old_afterpeak = k/10
    # distance_old_beforepeak = 0.346
    # progress_old_beforepeak = 0.2
    distance_new = np.interp(np.linspace(0,1,200),np.linspace(0,1,len(lookup_table)),lookup_table)
    # s_prior_beforepeak = []
    s_prior_afterpeak = []
    for j in range(len(distance_new)):
        # s_prior_beforepeak.append(progress_old_beforepeak + progress_old_beforepeak * (1-distance_new[j]/distance_old_beforepeak))
        s_prior_afterpeak.append(progress_old_afterpeak + progress_old_afterpeak * (1-distance_new[j]/distance_old_afterpeak))
        if s_prior_afterpeak[j] < 0:
            s_prior_afterpeak[j] = 0
        elif s_prior_afterpeak[j] > 0.75:
            s_prior_afterpeak[j] = 0.75

    plt.plot(progress_old_afterpeak,distance_old_afterpeak,'yo')
    # plt.plot(progress_old_beforepeak,distance_old_beforepeak,'yo')
    plt.plot(s_prior_afterpeak,distance_new,'m')
    # plt.plot(s_prior_beforepeak,distance_new,'c')

# Heuristic based on lookup table
peak = np.argmax(lookup_table)
distance_new_afterpeak = np.interp(np.linspace(0,1,100),np.linspace(0,1,len(lookup_table)-peak),lookup_table[peak:])
distance_new_beforepeak = np.interp(np.linspace(0,1,100),np.linspace(0,1,peak),lookup_table[:peak])
progress_new_afterpeak = []
progress_new_beforepeak = []
for j in range(len(distance_new_afterpeak)):
    progress_new_afterpeak.append((np.argmin(np.array([round(abs(lookup_table[peak+i]-distance_new_afterpeak[j]),3) for i in range(len(lookup_table)-peak)])) + peak)/len(lookup_table))
    progress_new_beforepeak.append((np.argmin(np.array([round(abs(lookup_table[peak-i]-distance_new_beforepeak[j]),3) for i in range(peak)])))/len(lookup_table))

progress_new_beforepeak = np.flip(progress_new_beforepeak)

plt.plot(np.linspace(0,1,len(demo_pos)),lookup_table)
plt.plot(progress_new_afterpeak,distance_new_afterpeak,'r')
plt.plot(progress_new_beforepeak,distance_new_beforepeak,'r')
plt.xlabel("Progress given by heuristic")
plt.ylabel("Distance tcp to new target")
plt.show()