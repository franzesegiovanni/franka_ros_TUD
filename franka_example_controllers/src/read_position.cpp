#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <franka/exception.h>
#include <franka_example_controllers/cartesian_impedance_example_controller.h>
#include <cmath>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include "pseudo_inversion.h"
#include "franka_msgs/FrankaState.h"


std::array<double, 3> Position;
std::array<double, 4> Orientation;

void chatterCallback(franka_msgs::FrankaState robot_state) //Probably here is missing a pointer but, if I add the pointer the code is not compiling 
{
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation()); 
  Eigen::Quaterniond orientation(transform.linear());
  Position[0]= position[0];
  Position[1]= position[1];
  Position[2]= position[2];

  Orientation[0] = orientation.x();  //x
  Orientation[1] = orientation.y();  //y
  Orientation[2] = orientation.z();  //z
  Orientation[3] = orientation.w();  //w
  //std::cout << position;
  //position=msg.O_T_EE;
  //robot_state = state_handle_->getRobotState().O_T_EE_d;
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "Position_EE");


  ros::NodeHandle node_position;
  ros::Rate loop_rate(100); // was 100

  ros::Subscriber sub = node_position.subscribe("/franka_state_controller/franka_states", 1000, chatterCallback);

  ros::Publisher pub = node_position.advertise<geometry_msgs::PoseStamped>("/cartesian_pose", 1000);
  

  //std_msgs::Float32MultiArray msg;
  geometry_msgs::PoseStamped msg;
  //std_msgs::Float32 msg;
  //std::array<double, 16> pose;
  

  while (ros::ok())
  {
   

    //Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    
    msg.pose.position.x=Position[0];
    msg.pose.position.y=Position[1];
    msg.pose.position.z=Position[2];

    msg.pose.orientation.x=Orientation[0];
    msg.pose.orientation.y=Orientation[1];
    msg.pose.orientation.z=Orientation[2];
    msg.pose.orientation.w=Orientation[3];
    
    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  




  //ros::spin();


  return 0;
}
