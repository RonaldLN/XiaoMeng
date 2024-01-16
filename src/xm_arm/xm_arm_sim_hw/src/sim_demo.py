#!/usr/bin/env python
#coding=utf-8

import rospy
import roslib
import actionlib
from std_msgs.msg import *
from control_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
from xm_msgs.srv import *


class ARM_SIM_HW:

    def __init__(self):
        rospy.init_node('sim_hw', anonymous = True)

        #初始化机器人位姿
        self.arm_pos = [0,-0.2790,-1.1550,0.0290]
        # self.arm_pos = [0.2,-0.2,-0.2,-0.2]
        self.gripper_pos = [0]
        self.plat_pos = [0]

        self.joint_states = JointState()
        self.joint_names = ['lift_joint','arm_joint_0', 'arm_joint_1', 'arm_joint_2', 'arm_joint_3','left_gripper_joint']
        self.joint_states.name = self.joint_names
        
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
            # self.joint_states.position =[-0.17451368952563218, 0.7797491767417611, -0.02920367324999651, 1.843133994164995, 1.8139303209149984]
            print(self.joint_states.position)
            pub.publish(self.joint_states)
            rate.sleep()



if __name__ == '__main__':
        
    arm_sim_hw = ARM_SIM_HW()
    
