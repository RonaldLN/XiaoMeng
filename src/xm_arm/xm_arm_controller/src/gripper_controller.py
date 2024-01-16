#!/usr/bin/env python
#coding=utf-8
'''
Author: jacy
Date: 2020-09-03 17:57:38
LastEditTime: 2020-09-04 10:09:06
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: /src/gripper_controller.py
'''

import rospy, actionlib
# import thread
import roslib
from control_msgs.msg import GripperCommandAction
from std_msgs.msg import Float64
from math import asin
from xm_msgs.srv import *
#from xm_msgs.msg import *

class GripperActionController:
    """ The actual action callbacks. """
    def __init__(self):
        rospy.init_node('gripper_controller')
        #删掉客户端命名空间的/moblie_base
        self.gripper_client = rospy.ServiceProxy('/gripper_command',xm_Gripper)
        self.server = actionlib.SimpleActionServer('gripper_controller/follow_joint_trajectory', GripperCommandAction, execute_cb=self.actionCb, auto_start=False)
        self.server.start()
        rospy.loginfo("\033[1;32mGripper Controller Start!\033[0m")
        rospy.spin()

    def actionCb(self, goal):
        
        """ Take an input command of width to open gripper. """
        rospy.loginfo("\033[1;33mgripper controller get the goal\033[0m")
        rospy.loginfo('Gripper controller action goal recieved:%f' % goal.command.position)
        # send command to gripper
        gripper_command = xm_GripperRequest()
        self.gripper_client.wait_for_service(timeout=10.0)

        if goal.command.position <0.01:
            gripper_command.command = 0
        elif goal.command.position >-0.01:
            gripper_command.command = 1
        else:
            gripper_command.command = 2
        
        
        res = self.gripper_client.call(gripper_command)
        # publish feedback

        while True:
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                rospy.loginfo('Gripper Controller: Preempted.')
                return
            if res.result is True:
                break
        self.server.set_succeeded()
        rospy.loginfo("\033[1;36mgripper controller complete action successful!\033[0m")
        
if __name__=='__main__':
    try:
        rospy.loginfo("\033[1;32mStart gripper_controller!\033[0m")
        GripperActionController()
    except rospy.ROSInterruptException:
        rospy.loginfo('Hasta la Vista...')

