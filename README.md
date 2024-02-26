This package serves as a ROS wrapper around [invariants_py](https://github.com/trajectory-invariants/invariants_py). 

It shows how [invariants_py](https://github.com/trajectory-invariants/invariants_py) can be integrated in a ROS application. There are two main functionalities:

1. online calculation of trajectory invariants from trajectory data on a ROS topic (e.g. `geometry_msgs/Pose`)
1. online trajectory generation with continuously changing boundary constraints coming in from a ROS topic

# Installation

ROS Noetic on Ubuntu 20.04 is assumed to be installed. Other versions of ROS/Ubuntu should work as well. A catkin workspace (`~/catkin_ws`) is assumed to be created.

Clone the [ros_invariants_py](https://gitlab.kuleuven.be/robotgenskill/ros-packages/ros_invariants_py) repository into your workspace:
```shell
cd ~/catkin_ws/src
git clone https://gitlab.kuleuven.be/robotgenskill/ros-packages/ros_invariants_py
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
roslaunch ros_invariants_py launch_invars_calc.launch
```

To test the calculation you can try using prerecorded data from the following [bagfile](https://wiki.ros.org/rosbag):

```shell
rosbag play -l data/....bag
```

TODO
- [ ] include window building
- [ ] include a bag file: check etasl_invariants_integration or contour_progress_estimation
- [ ] visualization in rviz or rqt graph

## Trajectory generation

TODO
