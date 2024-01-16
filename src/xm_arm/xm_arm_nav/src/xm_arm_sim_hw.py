#!/usr/bin/env python
#coding=utf-8
'''
Author: jacy
Date: 2020-09-01 10:56:27
LastEditTime: 2020-09-06 21:50:48
LastEditors: Please set LastEditors
Description: 模拟机械臂真实的硬件接口,接口与handsfree_hw完全一致，考虑了电机的不同速度和刷新频率
FilePath: /undefined/home/jacy/gazebo_test_ws/src/xm_arm/xm_arm_sim_hw/src/xm_arm_sim_hw.py
'''


import rospy
import roslib
import actionlib
from std_msgs.msg import *
from control_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
from xm_msgs.srv import *
#from xm_msgs.msg import *

PI = 3.1415926535 

class ARM_SIM_HW:
    '''
    description: 
    param {type} 
    return {type} 
    '''
    def __init__(self):
        rospy.init_node('arm_sim_hw', anonymous = True)

        #为了仿真的效果，我们在这里设置了关节更新的速度
        self.plat_velocity = 0.004                                              #升降移动的速度             m/s
        # self.arm_joint_velocity = [PI/5,PI/6.43,PI/3.4,PI/4.93,PI/5.57,PI/3.5]  #不同的关节的速度           radis/s   
        self.arm_joint_velocity = [PI/5,PI/6.43,PI/3.4,PI/4.93]
        self.frequency = float(50.0)                                           #设置刷新频率               hz

        #初始化机器人位姿
        self.arm_pos = [0,0.2978, 0.2978, 0]
        self.gripper_pos = [1]
        self.plat_pos = [1]
        self.camera_pos = [0]

        self.joint_states = JointState()
        self.joint_names = ['lift_joint',
            'arm_joint_0', 'arm_joint_1', 'arm_joint_2', 'arm_joint_3','gripper_joint']
        self.joint_states.name = self.joint_names

        #添加接口(在这里去掉了/mobile_base)
        # rospy.Subscriber('/set_arm_joints_pos', Float32MultiArray, self.set_arm_pos)
        
        # rospy.Service('/gripper_command', xm_Gripper, self.set_gripper_pos)
        # rospy.Service('/plat_command', xm_Plat, self.set_plat_pos)
        
        #有关摄像头的控制方式，在跟随人中，摄像头移动频繁，适合使用topic通信方式，
        #主要目的在于只发送命令，不需要反馈，因为这不会对决策有影响
        #但是在其他项目中，有时候是需要摄像头进行些许移动，用topic又不是很方便
        #故采用服务的形式，且简化他的反馈信号，直接返回true即可
        #具体方式，还需要具体测试,为此，摄像头的仿真无法加入平滑移动的功能
        # rospy.Service('/camera_command', xm_camera, self.set_camera_pos) 
        
        pub = rospy.Publisher('/joint_states',JointState, queue_size=10)
        
        rospy.loginfo("\033[1;32mArm sim successful!\033[0m")
        
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            
            #设置时间戳
            header = Header()
            header.stamp = rospy.Time.now()
            self.joint_states.header = header
            
            #总和各个关节位置并发出
            self.joint_states.position = list(self.plat_pos) + list(self.arm_pos) + list(self.gripper_pos) 
            pub.publish(self.joint_states)
            rate.sleep()


    def set_arm_pos(self,data):
        '''
        更新机械臂关节position
        '''
        arm_direction = [0,0,0,0]
        count = [0,0,0,0]
        tmp_arm_pos = [0,0,0,0]

        #判断各个关节移动的方向并计算其移动步数
        for i in range(len(self.arm_pos)):
            if self.arm_pos[i] < data.data[i]:
                arm_direction[i] = 1
            else:
                arm_direction[i] = -1

            count[i] = int ( abs( self.arm_pos[i] - data.data[i] ) / (self.arm_joint_velocity[i] / self.frequency) )
            tmp_arm_pos[i] = self.arm_pos[i]

        max_count = max(count)
        for i in range(max_count):
            
            #对每个未到达位置的关节匀速的增加或减小position
            for j in range(len(self.arm_pos)):
                
                #arm_joint_[j]关节为到达指定位置，需要继续移动
                if count[j]>0:
                    tmp_arm_pos[j] = tmp_arm_pos[j] + arm_direction[j] * (self.arm_joint_velocity[j] / self.frequency)
                    count[j] -= 1
            
            #更新
            self.arm_pos = list(tmp_arm_pos)
            #以一定频率运动
            rospy.sleep(1.0/self.frequency)



        self.arm_pos = data.data

    def set_gripper_pos(self,req):
        if req.command == 0:
            print("close")
            self.gripper_pos = [-0.03]
        elif req.command == 1:
            print("open")
            self.gripper_pos = [0.03]
        else:
            print("release")
            self.gripper_pos = [0]
            
        #TODO 检测爪子是否完成操作

        res = xm_GripperResponse()
        res.result = True
        return res

    def set_plat_pos(self,req):
        
        #判断增大还是减小
        if self.plat_pos[0] < req.height : 
            direction = 1
        else :
            direction = -1

        #进行的步数
        count = int ( abs( self.plat_pos[0] - req.height ) / (self.plat_velocity / self.frequency) )

        for i in range(count):
            self.plat_pos = [ self.plat_pos[0] + direction * (self.plat_velocity / self.frequency) ]
            #rospy.logwarn(self.plat_pos[0])
            rospy.sleep(1.0/self.frequency)
        
        self.plat_pos = [req.height]

        #TODO 检测升降台是否到达

        res = xm_PlatResponse()
        res.result = True
        return res

    def set_camera_pos(self,req):
        yaw = req.yaw
        pitch = req.pitch
        self.camera_pos = [yaw,pitch]

        res = xm_cameraResponse()
        res.result = True
        return res


if __name__ == '__main__':
        
    arm_sim_hw = ARM_SIM_HW()
    
