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
This impedance control allows to control the stiffness with a node in real time without using a server in the three different cartesian directions, rotations and null-space control. It also allow to have as input a desired configuriation control for doing null-space control. The reading of the end-effector position is already avaialable in the in the topic and no special type of message is necessary. The update of the reading is the same of the robot control, i.e. 1 KHz. 

This repository has been used in different article, for example [ILoSaA: Interactive Learning of Stiffness and Attractors](https://arxiv.org/abs/2103.03099)

For more informations or info please send an email to Giovanni  at g.franzese@tudelft.nl

The author of the repository wil not take any responsability in case of undesired behaviour of the repository. Please refer to the original one in case you were not directly asked to use this repository.


## How to install the controller
https://frankaemika.github.io/docs/installation_linux.html#building-the-ros-packages 
1. Be sure to have libfranka installed 
2. create a new catkin workspace catkin_ws:
3. Creat a src folder catkin_ws/src
3. clone this folder in the src folder
**git clone https://github.com/giovannifranzese94/franka_ros_TUD.git src/franka_ros_TUD**
4. Make the repository specifying where libfranka is located.

**catkin_make -DMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=<path/to/libfranka>**

5. Source ros in the repository 
**source devel/setup.sh**

**NB: remeber to source in every new terminal that you open**


## Ensure that the robot eternet cable and the computer cable are connected correctly to the router switch

## Set the computer on the right IP address:
How to find the right IP address? 
The IP address is composed by 4 number A.B.C.D. The first three need to be set as the same of the one set in the robot when it was installed. Please ask for this number. This number is stored in the NETWORK sction of the Desk interface of the robot. 
 The number D need to be different than the robot one because the robot and the computer do need to be in different IPs. 
Computer IP is for example A.B.C.E. For example, the robot is 172.16.0.2 and the computer network on 172.16.0.1. The netmask is 255.255.255.0 thas is also specified in the Network session of the DESK interface. 
**<computer_ip>=172.16.0.1
<robot_ip>=172.16.0.2**
## Add the ROS_IP information in the bash of your terminal
1. Open the bash file
gedit ~/.bashrc
2. Paste this two lines
export ROS_IP=<computer_ip>
export ROS_MASTER_URI=http://<computer_ip>:11311

## Open the desk interface to unlock and start the robot

In the broswer of Mozilla, go to the address https://<robot_ip>/desk/ where the IP address is the IP address of the robot.


## How to start the impedance controller 
roslaunch franka_example_controllers cartesian_impedance_example_controller.launch robot_ip:=<robot_ip> load_gripper:=True

For example,
<robot_ip>=172.16.0.2

## How to read the current position and orientation of the end-effector? 

rostopic echo /cartesian_pose

## How to connect your PC to the network and read and send commands to the controller. 
1. Connect your PC to the network
2. Create a new wired network and in IPv4 set Manual and put a new ip for your computer
<pc_ip>=A.B.C.F where F is different from the <computer_ip> or the <robot_ip>. Netmask is the same  255.255.255.0. 
Save the network. 
3.Add this to your bash file (gedit ~/.bashrc): 
export ROS_MASTER_URI=http://<computer_ip>:11311 
export ROS_IP=<pc_ip>
export ROS_HOSTNAME=<pc_ip>
4. source ros
source /opt/ros/<ros_version>/setup.bash
5. Test the data_streaming with 
rostopic list 

Example: **<computer_ip>=172.16.0.10**

## How to control the gripper
**rosrun franka_gripper franka_gripper_online**

To change the width of the gripper you can publish 
rostopic pub /gripper_online msgs/Float32 "data: 0.01"  
in the data you can specify your desired gripper width in meters.

## Stiffness modulation with topic
In this version of the code you can perform a fast modulation of the stiffness of the end effector cartesian position, orientation and null-space without necessity of using a service. You will also see it displayed in the usual interface. In the topic vector, please send the stiffness in the order that are displayed in the graphic interface. Decide if you want to use the graphic interface or the topic before hand. Try to not send a command with the topic and then click on the interface. 

## Change of the Jacobian Matrix
In case you do not want to do a pure cartesian impedance control but you want to control only position and have the orientation controlled as null-space control, it is enough to modify the jacobian matrix in this way. 
Remember that the order of variables are x,y,z,rx,ry,rz. 
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  for(int i=3; i<6; i++){
    for(int j=0; j<7; j++){
      jacobian_EE(i, j) = 0;
    }
  }
  
