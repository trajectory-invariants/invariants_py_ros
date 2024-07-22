# Real-time gesture recognition (thesis Kiron Plessers 2023-2024)

This git repository contains the full package that is needed to run a real-time gesture recognition application and is the main result of my thesis 'Toward real-time human-robot interaction using coordinate-invariant hand gestures'. This package shows how [invariants_py](https://github.com/trajectory-invariants/invariants_py) can be integrated in a ROS application. In this description, the prerequisites and functionalities that come along with this package is explained. For any further questions or problems, you can always contact me on my personal email address: plesserskiron@gmail.com.

## Installation

Dependencies:

- ROS Noetic on Ubuntu 20.04 is assumed to be installed. Other versions of ROS and Ubuntu should normally work as well but haven't been tested. 
- A catkin workspace is assumed to be created (e.g. at `~/catkin_ws`) and configured for Python 3 using `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`.
- [invariants_py](https://github.com/trajectory-invariants/invariants_py) should be installed in your Python environment.
- [dtw-python](https://pypi.org/project/dtw-python/) should be installed in your Python environment by running:

```
pip install dtw-python
```

To prepare the environment to run the application, open a terminal and clone this repository into your Catkin workspace. Switch to this branch if necessary:
```shell
cd ~/catkin_ws/src
git clone https://gitlab.kuleuven.be/robotgenskill/ros-packages/invariants_py_ros.git
cd invariants_py_ros
git checkout kiron_branch
```

Build the package and source the workspace:
```shell
cd ~/catkin_ws/src
catkin_make
source devel/setup.bash
rospack profile
```

### Overview of different nodes

The gesture recognition application consists of several different nodes, with each having different topics, message types and functions. 

The _ros_invariants_calculation_position_ node calculates the vector invariants for a point trajectory from a given .bag file, which can be specified in the launch files. It contains the following topics:

- **/pose_data** (input): measured pose, message type: `geometry_msgs/Pose`
- **/invariants_position_result** (output): calculated invariants of the moving window, message type: `std_msgs/Float32MultiArray`
- **/trajectory_online_array** (output): estimated trajectory by the invariant optimization,  message type: `std_msgs/Float32MultiArray`
- **/trajectory_online** (output): estimated trajectory, used for visualisation, message type: `visualization_msgs.msg/Marker`
- **/pose_data_stamped** (output): measured trajectory, used for visualisation, message type: `visualization_msgs.msg/Marker`

The _ros_invariants_calculation_position_live_ node is identical to the previous one, except that it uses live data from the HTC Vive as input. This means that the **\pose_data** topic is here replaced with a **\vive_data** topic, but it still has the same message type of `geometry_msgs/Pose`. When working with real-time trajectory data from the HTC Vive, the _ros_vive_data_receiver_ node is active aswell. This node continuously receives the trajectoy data (3 positions and 4 quaternions) from the tracker and publishes it to the **/vive_data** topic.

The _ros_invariants_calculation_rotation_ node is once again very similar, but it calculates the rotational Frenet-Serret invariants instead of the translational invariants for a point trajectory. This node works, but it is not yet implemented in the gesture recognition on this branch. The rotational invariants are however incorperated on the branch 'kiron_branch_rotation' of this repository, but this has not been tested and might contain bugs or problems.

After the invariants have been calculated (from one or more of the three nodes above), they are passed to the _ros_invariants_segmentation_ node, where a DTW algorithm is executed to find the similarity between the received invariants and a reference signal for a gesture, based on the first translational invariants. If this distance is below an experimentally devised threshold, a `True` value is published to the **\segment_found** topic, which has a message type of `std_msgs/Bool`. Otherwise, a `False` value is published.

This Boolean value, the calculated invariants and the estimated trajectoyr are the inputs of the _ros_invariants_classification_ node. Here, a DTW algorithm finds to which reference the invariants are the most similar based on the second translation invariants, resulting in the assignment of either a rotation or a translation as the geometry of the gesture. Simultaneously, the angle that the trajectory makes with the gravity vector is calculated to find the direction of the gesture. This gravity vector has to be changed manually. The result of this classification is published onto the **\gesture** topic with a message type of `std_msgs/Int32`, where each number stands for a different combination of trajectory and direction.

This **\gesture** topic is the input of the _ros_robot_simulation_ node, where each recognized gesture changes something about a 2D trajectory that is followed (different speed, changing direction, stop, resume) to simulate a possible application for gesture recognition. The coordinates of this 2D trajectory are stored on your computer as Python numpy arrays (.npy files). These coordinates are used for the simulation, which is obtained by running:

```
python3 src/invariants_py_ros/scripts/simulation_animation.py
```
This saves the animation to your `/catkin_ws` directory.

The _ros_invariants_plotting_ node puts all invariants on different topics (**\first_invariant_pos**, **\second_invariant_rot**, etc.). This can be useful for visualizing the results with `/rqt_plot`. For example, to show the middle value of the window for the first translational invariant in real-time, run:

```
rqt_plot /first_invariant_pos.data[10]
```

## Running the application 

There are several possible versions of the application that can be used and each one has their own launch file. The most important ones are the following:

- Real-time gesture recognition: This requires the HTC Vive to be connected and prints the recognized gestures in real-time, while also saving the .npy files that can be used for obtaining the simulation.
```
roslaunch invariants_py_ros launch_gesture_recognition.launch
```

- Gesture recognition on recorded .bag files: This prints the recognitized gestures in real-time, while also saving the .npy files that can be used for obtaining the simulation. The specific .bag file that is used can be changed in the launch file itself.
```
roslaunch invariants_py_ros launch_invars_classification.launch
```

- Calculating the invariants and plotting the estimated trajectory: This can be used to check the results from the OCP that calculated the invariants and judge their accuracy by looking at the estimated trajectory.
```
roslaunch invariants_py_ros launch_invars_calc_test.launch
```

## Dataset

The _data_ folder contains several different .bag files of human movements that can be useful when testing the gesture recognition:

- _forced_gestures_pose.bag_: four gestures with retrained motion in between that were the subject of Chapter 2 of my thesis text (rotation-translation-rotation-translation).
- _straight_motions.bag_ and _curved_motions.bag_: as the name says, these files contain either translations or rotations in multiple directions and can be used to test the system on the recogntion of trajectories.
- _demo.bag_: this file contains five different gestures and was used for the demonstration, of which the results can be found in the section below.
- _rot_hor.bag_, _trans_up.bag_, etc.: these six files were used to generate the results that are discussed in Chapter 4 of my thesis text. Each file only contains one gesture, which is executed 10 consecutive times.

It should be noted that these .bag files all contain the trajectory data in a message type of `geometry_msgs/Pose`, which consists of 3 positions (x,y,z) and 4 quaternions (x,y,z,w). As sensors often produce a .csv file as output, a useful conversion script between these file tyoes can be found on the [specific git repository for my thesis](https://gitlab.kuleuven.be/robotgenskill/master_thesis_code/thesis-kiron-plessers).

## Demonstration

A demonstration of how the gesture recognition from this repository works can be found at [this link](https://kuleuven-my.sharepoint.com/:v:/g/personal/kiron_plessers_student_kuleuven_be/EWamOLHZiqxIiNZgg7TlFiABiv-45zbGUmNTU5Ebdv54aw?nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZy1MaW5rIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXcifX0%3D&e=fj4j3O). It shows how the different gestures are executed, what the possible commands could be for a simple application and what the reaction times are.
