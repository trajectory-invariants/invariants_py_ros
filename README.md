This package shows how [invariants_py](https://github.com/trajectory-invariants/invariants_py) can be integrated in a ROS application. 

Currently there are two examples:

1. online calculation of trajectory invariants from trajectory data on a subscribed ROS topic with message type `geometry_msgs/Pose`
1. online trajectory generation from an invariant model towards a changing position provided by a ROS topic with message type `geometry_msgs/Point`

### Installation

Dependencies:

- ROS Noetic on Ubuntu 20.04 is assumed to be installed. Other versions of ROS/Ubuntu should work as well. 
- A catkin workspace is assumed to be created (e.g. at `~/catkin_ws`) and configured for Python 3 using catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`.
- [invariants_py](https://github.com/trajectory-invariants/invariants_py) should be installed in your Python environment.

Open a terminal and clone the [invariants_py_ros](https://gitlab.kuleuven.be/robotgenskill/ros-packages/invariants_py_ros) repository into your Catkin workspace:
```shell
cd ~/catkin_ws/src
git clone https://gitlab.kuleuven.be/robotgenskill/ros-packages/invariants_py_ros
```

Build the package and source the workspace:
```shell
cd ~/catkin_ws/src
catkin_make
source devel/setup.bash
rospack profile
```

### Online invariants calculation

The invariants calculation node assumes that trajectory data is being streamed on a `geometry_msgs/Pose` topic.

The node can be launched using:

```shell
roslaunch invariants_py_ros launch_invars_calc.launch
```

To test the calculation you can try using prerecorded data from the following bagfile:

```shell
roscd invariants_py_ros
rosbag play -l -r 0.5 data/example_pose_data.bag
```

Suggested improvements:

- Only the position data is being used currently. Extend the code so that invariants for rotation are included. To deal with pose messages check the etasl_invariants_integration repository in ERC/python_projects
- The invariants are now time-based using a window of measurements wrt time. How would you obtain time-invariance, a.k.a. geometric invariants that are defined w.r.t. a certain progress variable? 
- Options to visualize results:
-- put results on ROS topics and make an additional ROS node that plots the data in Python
-- put results on ROS topics and use RQTgraph and/or rviz

Example of vizualization in rviz (black = estimated trajectory of the measured object inside window of OCP, axes = current pose measurement but only the position is being used now).
![screenshot rviz](data/screenshot_rviz_trajectory.png)

### Online trajectory generation

TODO
