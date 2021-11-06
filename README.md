# ROS integration for Franka Emika research robot
# Impedance Controller CoR Lab TU Delft
[![Build Status][travis-status]][travis]

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[travis-status]: https://travis-ci.org/frankaemika/franka_ros.svg?branch=kinetic-devel
[travis]: https://travis-ci.org/frankaemika/franka_ros

This repository does only contain the impedance controller that is used in the CoR lab of TU Delft. 
This impedance control allows to control the stiffness with a node in real time without using a server in the three different cartesian directions, rotations and null-space control. It also allow to have as input a desired configuriation control for doing null-space control. 


For more informations or info please send an email to Giovanni  at g.franzese@tudelft.nl

The author of the repository wil not take any responsability in case of undesired behaviour of the repository. Please refer to the original one in case you were not directly asked to use this repository.

This repository was tested with libfranka 0.8 with ros noetic in Ubuntu 20.


## How to install the controller
https://frankaemika.github.io/docs/installation_linux.html#building-the-ros-packages 
1. Be sure to have libfranka installed 
2. create a new workspace
3. clone this folder in the scr foulder and not the orginal one. 
4. catkin_make -DMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=<path/to/libfranka>

## How to start the impedance controller 
roslaunch franka_example_controllers cartesian_impedance_example_controller.launch robot_ip:=<robot_ip> load_gripper:=True

## How to run a publisher for reading the current robot position and orientation without the need of special message types. 
rosrun franka_example_controllers read_position

## How to control the gripper
rosrun franka_gripper franka_gripper_online

To change the width of the gripper you can publish 
rostopic pub /gripper_online msgs/Float32 "data: 0.01"  
in the data you can specify your desired gripper width in meters.

## 
