This package shows how [invariants_py](https://github.com/trajectory-invariants/invariants_py) can be integrated in a ROS application. 

There are two main functionalities:

1. online calculation of trajectory invariants from trajectory data on a ROS topic (e.g. `geometry_msgs/Pose`)
1. online trajectory generation with continuously changing boundary constraints coming in from a ROS topic

# Installation

Dependencies:
- ROS Noetic on Ubuntu 20.04 is assumed to be installed. Other versions of ROS/Ubuntu should work as well. A catkin workspace is assumed to be created (e.g. `~/catkin_ws`).
- [invariants_py](https://github.com/trajectory-invariants/invariants_py) is assumed to be installed in your Python environment.

Clone the [invariants_py_ros](https://gitlab.kuleuven.be/robotgenskill/ros-packages/invariants_py_ros) repository into your workspace:
```shell
cd ~/catkin_ws/src
git clone https://gitlab.kuleuven.be/robotgenskill/ros-packages/invariants_py_ros
```

Build the package and source the workspace:
```shell
cd ~/catkin_ws/src
catkin_make
source devel/setup.bash
```

# Quickstart

## Invariants calculation

The invariants calculation node assumes that trajectory data is being streamed on a `geometry_msgs/Pose` topic.

The node can be launched using:

```shell
roslaunch invariants_py_ros launch_invars_calc.launch
```

To test the calculation you can try using prerecorded data from the following [bagfile](https://wiki.ros.org/rosbag):

```shell
rosbag play -l data/....bag
```

TODO
- [ ] build_time_window // build_progress_window 
- [ ] include a bag file: check etasl_invariants_integration or contour_progress_estimation
- [ ] visualization results in rviz or rqt graph

## Trajectory generation

TODO after invariants calculation is finished
