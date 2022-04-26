#%%
#!/usr/bin/env python
#from winreg import REG_EXPAND_SZ
import rospy
import math
import numpy as np
import time

#from zmq import RECONNECT_IVL_MAX
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, Float32
import dynamic_reconfigure.client
from sys import exit
from pynput.keyboard import Listener, KeyCode

class LfD():
    def __init__(self):
        rospy.init_node('LfD', anonymous=True)
        self.r=rospy.Rate(10)
        self.curr_pos=None
        self.width=None
        self.recorded_traj = None 
        self.recorded_gripper= None
        self.end = False
        self.save_joint_position = False
        self.gripper_sub=rospy.Subscriber("/joint_states", JointState, self.gripper_callback)
        self.joints=rospy.Subscriber("/joint_states", JointState, self.joint_callback)  
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=0)
        self.grip_pub = rospy.Publisher('/gripper_online', Float32, queue_size=0)
        self.stiff_mat_pub_ = rospy.Publisher('/stiffness', Float32MultiArray, queue_size=0) #TODO check the name of this topic  
        self.joint_pub = rospy.Publisher('/equilibrium_configuration', JointState , queue_size=0)
        self.listener = Listener(on_press=self._on_press)
        self.listener.start()
    def gripper_callback(self, data):
        self.width =data.position[7]+data.position[8]
        #rospy.loginfo(self.width)
    def joint_callback(self,data):
        self.curr_joint =data.position[0:7]      

    def _on_press(self, key):
        # This function runs on the background and checks if a keyboard key was pressed
        if key == KeyCode.from_char('e'):
            self.end = True        
        if key == KeyCode.from_char('j'):
            self.save_joint_position = True  

    def set_stiffness_joint(self, k_1, k_2, k_3, k_4, k_5, k_6, k_7):

        set_K = dynamic_reconfigure.client.Client('/dynamic_reconfigure_compliance_param_node', config_callback=None)
        set_K.update_configuration({"joint_1": k_1})
        set_K.update_configuration({"joint_2": k_2})
        set_K.update_configuration({"joint_3": k_3})        
        set_K.update_configuration({"joint_4": k_4}) 
        set_K.update_configuration({"joint_5": k_5}) 
        set_K.update_configuration({"joint_6": k_6})
        set_K.update_configuration({"joint_7": k_7}) 

    def joint_rec_point(self):
        #stiff_des = Float32MultiArray()
        #stiff_des.data = np.array([0.0, 0.0, 0.0, 30.0, 30.0, 30.0, 20.0]).astype(np.float32)
        #self.stiff_pub.publish(stiff_des) 
        self.set_stiffness_joint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.recorded_joint = self.curr_joint
        self.recorded_gripper= self.width

        while self.end ==False:       
            if  self.save_joint_position==True: 
                self.recorded_joint = np.c_[self.recorded_joint, self.curr_joint]
                self.recorded_gripper = np.c_[self.recorded_gripper, self.width]
                print('Adding joint position')
                time.sleep(0.5) 
                self.save_joint_position=False   
        print('End of the demonstration')        
  

    def joint_rec(self):
        self.set_stiffness_joint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        trigger = 0.05 
        init_joint = self.curr_joint
        vel = 0
        while not(vel > trigger):
            vel= math.sqrt((self.curr_joint[0]-init_joint[0])**2 + (self.curr_joint[1]-init_joint[1])**2 + (self.curr_joint[2]-init_joint[2])**2+ (self.curr_joint[3]-init_joint[3])**2+ (self.curr_joint[4]-init_joint[4])**2+ (self.curr_joint[5]-init_joint[5])**2+(self.curr_joint[6]-init_joint[6])**2)
        print("Recording Trajectory")
        self.recorded_joint = self.curr_joint
        self.recorded_gripper= self.width
        # recorded_joint = joint_pos
        self.end=False
        while not(self.end):
            now = time.time()      

            self.recorded_gripper = np.c_[self.recorded_gripper, self.width]
            self.recorded_joint = np.c_[self.recorded_joint, self.curr_joint]
            self.r.sleep()

        self.end=False    


    def go_to_start_joint(self):
        start = self.curr_joint
        goal_=np.array([self.recorded_joint[0][0], self.recorded_joint[1][0], self.recorded_joint[2][0], self.recorded_joint[3][0], self.recorded_joint[4][0], self.recorded_joint[5][0], self.recorded_joint[6][0]])
        print("goal:", goal_)
        # interpolate from start to goal with attractor distance of approx 1 cm
        squared_dist = np.sum(np.subtract(start, goal_)**2, axis=0)
        dist = np.sqrt(squared_dist)
        print("dist", dist)
        interp_dist = 0.05
        step_num = math.floor(dist / interp_dist)
        print("num of steps", step_num)
        q1 = np.linspace(start[0], goal_[0], step_num)
        q2 = np.linspace(start[1], goal_[1], step_num)
        q3 = np.linspace(start[2], goal_[2], step_num)
        q4 = np.linspace(start[3], goal_[3], step_num)
        q5 = np.linspace(start[4], goal_[4], step_num)
        q6 = np.linspace(start[5], goal_[5], step_num)
        q7 = np.linspace(start[6], goal_[6], step_num)
        goal=JointState()
        goal.position=[q1[0],q2[0],q3[0],q4[0],q5[0],q6[0],q7[0]]
        self.joint_pub.publish(goal)
        self.set_stiffness_joint(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 5.0)

        for i in range(step_num):
            goal=JointState()
            goal.position=[q1[i],q2[i],q3[i],q4[i],q5[i],q6[i],q7[i]]
            self.joint_pub.publish(goal)
            self.r.sleep()  


    def execute_joints(self):
        self.set_stiffness_joint(20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 2.0)

        for i in range(np.shape(self.recorded_joint)[1]): 
            goal=JointState()
            goal.position=self.recorded_joint[:,i] 
            self.joint_pub.publish(goal)
            self.r.sleep()

    def execute_joints_points(self):
        self.set_stiffness_joint(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 2.0)

        for i in range(np.shape(self.recorded_joint)[1]): 
            goal=JointState()
            goal.position=self.recorded_joint[:,i]
            print(self.recorded_joint[:,i]) 
            self.joint_pub.publish(goal)
            time.sleep(3)  

    #def start_ros(self):
#%%    
LfD=LfD()

#%%
LfD.joint_rec_point()  # a new window should open for stopping the recording

#%%
LfD.go_to_start_joint()

#%%
LfD.execute_joints_points()

#%%
LfD.joint_rec()  # a new window should open for stopping the recording

#%%
LfD.go_to_start_joint()

#%%
LfD.execute_joints()

# %%
