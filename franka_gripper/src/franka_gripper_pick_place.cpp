#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <franka_gripper/franka_gripper.h>

double width =0;
double flag_move =0;
double flag_grasp =0;
void PickCallback(const std_msgs::Float32::ConstPtr& msg)
{
  width=msg->data;
  flag_move = 1;
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%
void PlaceCallback(const std_msgs::Float32::ConstPtr& msg)
{
  width=msg->data;
  flag_grasp = 1;
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
}
// %EndTag(CALLBACK)%
int main(int argc, char **argv)
{

  ros::init(argc, argv, "gripper");


  ros::NodeHandle n;
  ros::Rate loop_rate(30);

  ros::Subscriber sub_pick = n.subscribe("/gripper_pick", 1, PickCallback);
  ros::Subscriber sub_place = n.subscribe("/gripper_place", 1, PlaceCallback);
  ros::Publisher pub_move = n.advertise<franka_gripper::MoveActionGoal>("/franka_gripper/move/goal", 1);
  ros::Publisher pub_stop = n.advertise<franka_gripper::StopAction>("/franka_gripper/move/goal", 1);
  ros::Publisher pub_grasp = n.advertise<franka_gripper::GraspActionGoal>("/franka_gripper/grasp/goal", 1);

  franka_gripper::MoveActionGoal msg_move;
  franka_gripper::GraspActionGoal msg_grasp;
  msg_move.goal.speed = 1;
  msg_grasp.goal.speed = 1;
  while (ros::ok())
  {
   if(flag_move==1)
   {

     msg_move.goal.width = width;
     pub_move.publish(msg_move);
     flag_move = 0;
    }
  if(flag_grasp==1)
   {

     msg_grasp.goal.width = width;
     pub_grasp.publish(msg_grasp);
     flag_grasp = 0;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }
  




  //ros::spin();


  return 0;
}

