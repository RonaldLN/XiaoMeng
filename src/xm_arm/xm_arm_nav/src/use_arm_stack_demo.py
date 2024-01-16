#!/usr/bin/env python
# encoding:utf8

'''
Author: jacy
Date: 2020-09-04 20:25:04
LastEditTime: 2020-09-04 20:44:37
LastEditors: Please set LastEditors
Description: show how it work
FilePath: /undefined/home/jacy/gazebo_test_ws/src/xm_arm/xm_arm_nav/scripts/use_arm_stack_demo.py
'''

import rospy, sys
import actionlib
from xm_msgs.srv import *
from xm_msgs.msg import *

def use_arm_stack_demo(target_camera_point,target_size,table_depth):
    '''
    - INPUT:
        - target_camera_point(PointStamp)
        - target_size(list)
        - table_depth(float)
    - OUTPUT
    '''
    
    arm_stack_client = actionlib.SimpleActionClient("/xm_arm/arm_stack", xm_ArmStackAction)
    
    goal = xm_ArmStackGoal()
    goal.target_camera_point  = target_camera_point
    goal.target_size_l = target_size[0]
    goal.target_size_w = target_size[1]
    goal.target_size_h = target_size[2]
    goal.table_depth = table_depth
    
    arm_stack_client.send_goal(goal)
    arm_stack_client.wait_for_result(rospy.Duration(60.0))


