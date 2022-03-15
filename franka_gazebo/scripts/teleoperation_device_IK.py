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

offset = [0, 0, 0, 0, 0, 0,0]
curr_pos = None
joint_pos = None
def spacenav_callback(data):
    global offeset
    offset[0]= 0.002*data.axes[0]
    offset[1]= 0.002*data.axes[1]
    offset[2]= 0.002*data.axes[2]
    offset[3]= 0.02*data.axes[3]
    offset[4]= 0.02*data.axes[4] 
    offset[5]= 0.02*data.axes[5]
    offset[6]= 0.02*data.button[1]-0.02*data.button[0]
    
    
'''
def ee_pos_callback(data):
    global curr_pos
    curr_pos = np.array([data.x, data.y, data.z])
'''
def joint_callback(data):
    global joint_pos
    joint_pos = np.array(data.position)
    
def teleoperation():
    global offset, curr_pos, joint_pos
    robot = rtb.models.DH.Panda()
    x_new = 0.5
    y_new = 0
    z_new = 0.5
    joint=joint_pos[0:7].reshape(1, -1)
    fw_pose = robot.fkine(joint)
    goal_ori = fw_pose.R
    rot = Rotation.from_matrix(goal_ori)
    goal_ori_eul = rot.as_euler('xyz')      
    #pygame.init()
    #pygame.display.set_mode((440, 80))
    #pygame.display.set_caption('Stop Execution With Keypress')
    #key_pressed = False
    r = rospy.Rate(50)
    while 1:#not key_pressed: 
        joint=joint_pos[0:7].reshape(1, -1)
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
    
        goal.pose.orientation.x = goal_quat[0]#1.0
        goal.pose.orientation.y = goal_quat[1]#0.0
        goal.pose.orientation.z = goal_quat[2]#0.0
        goal.pose.orientation.w = goal_quat[3]#0.0
    
        goal_pub.publish(goal)
            
        #Set the stiffness of the robot 
            
        stiff_des = Float32MultiArray()
        #stiff_des.data = np.array([600.0, 600.0, 600.0, stiff[0][0], 30.0]).astype(np.float32)
        stiff_des.data = np.array([600.0, 600.0, 600.0, 30.0, 30.0, 30.0, 2.0]).astype(np.float32)
        stiff_pub.publish(stiff_des)
        
        
        # Create message
        req = JointTrajectory()
        req.joint_names = ["panda_finger_joint1", "panda_finger_joint2"]
        point = JointTrajectoryPoint()
        point.positions = [offset[6], offset[6]]
        point.effort = [0.0, 0.0]
        point.velocities = [0.0, 0.0]
        point.accelerations = [0.0, 0.0]
        point.time_from_start = rospy.Duration(secs=1)
        req.points.append(point)

        # Close
        pub.publish(req)
        #Set the null-space equilibrium configuration
        #joint=joint_pos[0:7].reshape(1, -1)
        #T = SE3([x_new, y_new, z_new])*SE3.Rz(goal_ori_eul[2])*SE3.Ry(goal_ori_eul[1])*SE3.Rx(goal_ori_eul[0])
        #q_ik = robot.ikine(T, q0=joint, tol=1e-7)
        #joint_des=Float32MultiArray()
        #temp=q_ik[0]
        #print(temp)
        #temp[0]=np.clip(temp[0], -2.7, 2.7)
        #temp[1]=np.clip(temp[1], -1.6, 1.6)
        #temp[2]=np.clip(temp[2], -2.7, 2.7)
        #temp[3]=np.clip(temp[3], -2.9, -0.1)
        #temp[4]=np.clip(temp[4], -2.7, 2.7)
        #temp[5]=np.clip(temp[5], 0.1, 3.6)
        #temp[6]=np.clip(temp[6], -2.7, 2.7)
        #temp=temp.astype(np.float32)
        #joint_des.data=temp.astype(np.float32)
        #null_space_eq.publish(joint_des)
    
        r.sleep()
        
    
#%%    
if __name__ == '__main__':
    rospy.init_node('ILoSA', anonymous=True)
    #rospy.Subscriber("/cartesian_position", Point, ee_pos_callback)
    rospy.Subscriber("/franka_state_controller/joint_states", JointState, joint_callback)
    rospy.Subscriber("/spacenav/joy", Joy, spacenav_callback)
    # attractor publisher
    goal_pub = rospy.Publisher('cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    stiff_pub = rospy.Publisher('/sub_stiffness_', Float32MultiArray, queue_size=10) #in this vector we can send [x, y, z, ns, rot] stiffness
    null_space_eq= rospy.Publisher('/equilibrium_configuration', Float32MultiArray, queue_size=10)
    pub = rospy.Publisher("/panda_gripper_trajectory_controller/command", JointTrajectory, queue_size=10)
    time.sleep(2)
#%%

    teleoperation()