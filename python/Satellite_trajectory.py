#%%
#!/usr/bin/env python
import rospy
import math
import numpy as np
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float32MultiArray, Float32
import csv
class LfD():
    def __init__(self):
        self.recorded_traj = None
        self.recorded_gripper= None
        self.gripper_width=0.035
    def start_ros(self):
        self.r=rospy.Rate(100)
        self.curr_pos=None
        self.width=None
        self.grip_pub = rospy.Publisher('/gripper_online', Float32, queue_size=0)
        self.pos_sub= rospy.Subscriber("/cartesian_pose", PoseStamped, self.ee_pos_callback)
        self.gripper_sub= rospy.Subscriber("/joint_states", JointState, self.gripper_callback)
        self.goal_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=1)
        self.stiff_pub = rospy.Publisher('/stiffness', Float32MultiArray, queue_size=1)
        self.velocity_pub=rospy.Publisher('/equilibrium_vel', TwistStamped, queue_size=1)
        self.configuration_pub=rospy.Publisher("/equilibrium_configuration",Float32MultiArray, queue_size=1)
    def ee_pos_callback(self, data):
        self.curr_pos = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.curr_ori =np.array([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        grip_command = Float32()
        grip_command.data = self.gripper_width#self.recorded_gripper[i][0]
        self.grip_pub.publish(grip_command) 
    def gripper_callback(self, data):
        self.width=data.position[7]+data.position[8]    
        #rospy.loginfo(self.width)
    def load_traj(self,str):
        with open(str) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',',quoting=csv.QUOTE_NONNUMERIC)
            #count=0
            recorded_traj=np.asarray(list(csv_reader))

            self.position=recorded_traj[:,1:4]
            self.orientation=-recorded_traj[:,7:]
            self.velelocity=recorded_traj[:,4:7]
            self.configuration=np.array([0.00, -1.41, 0.01, -1.74, -0.00, 1.90, -2.36]).astype(np.float32)
    # control robot to desired goal position
    def go_to_start(self):
        grip_command = Float32()
        grip_command.data = self.gripper_width#self.recorded_gripper[i][0]
        self.grip_pub.publish(grip_command) 
        stiff_des = Float32MultiArray()


        stiff_des.data = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0]).astype(np.float32)
        self.stiff_pub.publish(stiff_des)

        joint_des=Float32MultiArray()

        joint_des.data=self.configuration
        self.configuration_pub.publish(joint_des)
        time.sleep(5)
        start_pos = self.curr_pos
        start_rot  = self.curr_ori
        goal_pos=self.position[0]
        goal_rot=self.orientation[0]
        # interpolate from start to goal with attractor distance of approx 1 mm
        squared_dist = np.sum(np.subtract(start_pos, goal_pos)**2, axis=0)
        dist = np.sqrt(squared_dist)
        interp_dist = 0.001  # [m]a
        step_num = math.floor(dist / interp_dist)

        x = np.linspace(start_pos[0], goal_pos[0], step_num)
        y = np.linspace(start_pos[1], goal_pos[1], step_num)
        z = np.linspace(start_pos[2], goal_pos[2], step_num)

        rot_x=np.linspace(start_rot[0], goal_rot[1], step_num)
        rot_y=np.linspace(start_rot[1], goal_rot[2], step_num)
        rot_z=np.linspace(start_rot[2], goal_rot[3], step_num)
        rot_w=np.linspace(start_rot[3], goal_rot[0], step_num)
        
        goal = PoseStamped()
        
        goal.pose.position.x = x[0]
        goal.pose.position.y = y[0]
        goal.pose.position.z = z[0]

        goal.pose.orientation.x = rot_x[0]
        goal.pose.orientation.y = rot_y[0]
        goal.pose.orientation.z = rot_z[0]
        goal.pose.orientation.w = rot_w[0]

        self.goal_pub.publish(goal)
        
        stiff_des = Float32MultiArray()


        stiff_des.data = np.array([1000.0, 1000.0, 1000.0, 10.0, 10.0, 10.0, 0.0]).astype(np.float32)
        self.stiff_pub.publish(stiff_des)

        joint_des=Float32MultiArray()

        joint_des.data=self.configuration
        self.configuration_pub.publish(joint_des)

        for i in range(step_num):
            now = time.time()            # get the time
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = x[i]
            goal.pose.position.y = y[i]
            goal.pose.position.z = z[i]

            goal.pose.orientation.x = rot_x[i]
            goal.pose.orientation.y = rot_y[i]
            goal.pose.orientation.z = rot_z[i]
            goal.pose.orientation.w = rot_w[i]

            self.goal_pub.publish(goal)
            grip_command = Float32()
            grip_command.data = self.gripper_width#self.recorded_gripper[i][0]
            self.grip_pub.publish(grip_command) 
            self.r.sleep()   


    def execute(self):
        goal = PoseStamped()

        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = self.position[0][0] 
        goal.pose.position.y = self.position[0][1]
        goal.pose.position.z = self.position[0][2]

        goal.pose.orientation.x = self.orientation[0][1]
        goal.pose.orientation.y = self.orientation[0][2]
        goal.pose.orientation.z = self.orientation[0][3]
        goal.pose.orientation.w = self.orientation[0][0]
        stiff_des = Float32MultiArray()
        stiff_des.data = np.array([2000.0, 2000.0, 2000.0, 30.0, 30.0, 30.0, 0.0]).astype(np.float32)
        self.stiff_pub.publish(stiff_des)
        for i in range (np.shape(self.position)[0]):
            goal = PoseStamped()

            goal.header.seq = 1  
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = self.position[i][0] 
            goal.pose.position.y = self.position[i][1]
            goal.pose.position.z = self.position[i][2]

            goal.pose.orientation.x = self.orientation[i][1]
            goal.pose.orientation.y = self.orientation[i][2]
            goal.pose.orientation.z = self.orientation[i][3]
            goal.pose.orientation.w = self.orientation[i][0]

            goal_vel= TwistStamped()

            goal_vel.twist.linear=[self.velelocity[i][0:3]]
            goal_vel.twist.angular=[0.0,0.0,0.0]

            grip_command = Float32()

            grip_command.data = self.gripper_width#self.recorded_gripper[i][0]

            self.goal_pub.publish(goal)
            self.velocity_pub.publish(goal_vel)
            self.grip_pub.publish(grip_command) 

            self.r.sleep()


    #def start_ros(self):

#%%
if __name__ == '__main__':
    rospy.init_node('LfD', anonymous=True)
#%%    
    LfD=LfD()
    #rospy.spin()
#%%
    LfD.start_ros()
#%%
    LfD.load_traj('trajectoryData4.csv')
#%%
    LfD.go_to_start()

#%%
    LfD.execute()
#%%


