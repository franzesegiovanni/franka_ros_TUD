#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 29 17:17:53 2021

@author: oem
"""
import rospy
import math
import numpy as np
import pygame
import time
import scipy
import dynamic_reconfigure.client
import pandas as pd
from pygame.locals import *
from sensor_msgs.msg import JointState, Joy
from geometry_msgs.msg import Point, WrenchStamped, PoseStamped, Vector3
from std_msgs.msg import Float32MultiArray
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, WhiteKernel, ConstantKernel as C
from scipy.spatial.transform import Rotation
from spatialmath import SE3
import roboticstoolbox as rtb

offset = [0, 0, 0, 0, 0, 0]
curr_pos = None
joint_pos = None
goal_ori_eul=[0,0,0]
def spacenav_callback(data):
    global offeset
    offset[0]= 0.002*data.axes[0]
    offset[1]= 0.002*data.axes[1]
    offset[2]= 0.002*data.axes[2]
    offset[3]= 0.02*data.axes[3]
    offset[4]= 0.02*data.axes[4] 
    offset[5]= 0.02*data.axes[5]

def teleoperation():
    global offset
    robot = rtb.models.DH.Panda()
    x_new = 0.5
    y_new = 0.0
    z_new = 0.5  
    
    #pygame.init()
    #pygame.display.set_mode((440, 80))
    #pygame.display.set_caption('Stop Execution With Keypress')
    #key_pressed = False
    r = rospy.Rate(50)
    while 1: 
        #joint=joint_pos[0:7].reshape(1, -1)
        x_new = x_new + offset[0]
        y_new = y_new + offset[1]
        z_new = z_new + offset[2]  
        goal_ori_eul[0]=goal_ori_eul[0]+offset[3]
        goal_ori_eul[1]=goal_ori_eul[1]+offset[4]
        goal_ori_eul[2]=goal_ori_eul[2]+offset[5]
        #stiff = np.clip((1- (sigma/max_sigma)**5)*NS_stiffness, 0, NS_stiffness)
        rot=Rotation.from_euler('xyz', goal_ori_eul)
        goal_quat=rot.as_quat()
        #print(goal_quat)
        #Pubblish goal of the end-effector
        goal = PoseStamped()
    
        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
    
        goal.pose.position.x = x_new
        goal.pose.position.y = y_new
        goal.pose.position.z = z_new
    
        goal.pose.orientation.x = goal_quat[0]
        goal.pose.orientation.y = goal_quat[1]
        goal.pose.orientation.z = goal_quat[2]
        goal.pose.orientation.w = goal_quat[3]
    
        goal_pub.publish(goal)
        print(goal)    
        #Set the stiffness of the robot 
            
        stiff_des = Float32MultiArray()
        stiff_des.data = np.array([600.0, 600.0, 600.0, 30.0, 30.0, 30.0, 10]).astype(np.float32)
        stiff_pub.publish(stiff_des)
            
        #Set the null-space equilibrium configuration
    
        r.sleep()
        
    
#%%    
if __name__ == '__main__':
    rospy.init_node('ILoSA', anonymous=True)
    #rospy.Subscriber("/cartesian_position", Point, ee_pos_callback)
    #rospy.Subscriber("/cartesian_impedance_example_controller/equilibrium_pose", Point, ee_pos_callback)
    #rospy.Subscriber("/joint_states", JointState, joint_callback)
    rospy.Subscriber("/spacenav/joy", Joy, spacenav_callback)
    # attractor publisher
    #goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=10)
    goal_pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    stiff_pub = rospy.Publisher('/sub_stiffness_', Float32MultiArray, queue_size=10) #in this vector we can send [x, y, z, ns, rot] stiffness
    null_space_eq= rospy.Publisher('/equilibrium_configuration', Float32MultiArray, queue_size=10)

#%%

    teleoperation()
