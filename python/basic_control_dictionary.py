#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct  9 12:42:04 2021

@author: Giovanni Franzese 

This code would not work as it is. It is just for dictionary
"""

import rospy
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Point, WrenchStamped, PoseStamped, Vector3
from std_msgs.msg import Float32MultiArray

def ee_pos_callback(data):
    curr_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z]) #this maybe need to be changed
    curr_orientation=np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
    
  
def joint_callback(data):
    joint_pos = np.array(data.position)

rospy.Subscriber("/cartesian_pose", Point, ee_pos_callback)
rospy.Subscriber("/joint_states", JointState, joint_callback)
goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=10) #check if it is the one where you also have the orientation
stiff_pub = rospy.Publisher('/sub_stiffness_', Float32MultiArray, queue_size=10) #in this vector we can send [x, y, z, ns, rot] stiffness
null_space_eq= rospy.Publisher('/equilibrium_configuration', Float32MultiArray, queue_size=10)



stiff_des.data = np.array([600.0, 600.0, 600.0, 30.0, 30.0, 30.0, 5.0]).astype(np.float32) #stiffness in [x,y,z, rot(x), rot(y), rot(z), null_space]
stiff_pub.publish(stiff_des)

joint_ns_des.data= np.array([joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4], joint_pos[5], joint_pos[6]]).astype(np.float32) #put the desired joint configuration that you would like to have in the null-space
null_space_eq.publish(joint_ns_des)