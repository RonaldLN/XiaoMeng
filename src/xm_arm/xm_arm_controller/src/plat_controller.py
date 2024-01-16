#!/usr/bin/env python
#coding=utf-8
'''
Author: jacy
Date: 2020-09-03 17:57:38
LastEditTime: 2020-09-04 10:09:25
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: /src/plat_controller.py
'''


import rospy, actionlib
import _thread
import roslib
from control_msgs.msg import GripperCommandAction
from std_msgs.msg import Float64
from math import asin
from xm_msgs.srv import *
#from xm_msgs.msg import *

class PlatActionController:
    """ The actual action callbacks. """
    def __init__(self):
        rospy.init_node('plat_controller')
	#删除客户端命名空间的/mobile_base
        self.plat_client = rospy.ServiceProxy('/plat_command',xm_Plat)
        self.server = actionlib.SimpleActionServer('plat_controller/follow_joint_trajectory', GripperCommandAction, execute_cb=self.actionCb, auto_start=False)
        self.server.start()
        rospy.loginfo("\033[1;32mPlat Controller Start!\033[0m")
        rospy.spin()

    def actionCb(self, goal):
        
        """ Take an input command of width to open plat. """
        rospy.loginfo("\033[1;33mplat controller get the goal\033[0m")
        rospy.loginfo('Plat controller action goal recieved:%f' % goal.command.position)
        # send command to Plat
        plat_command = xm_PlatRequest()
        self.plat_client.wait_for_service(timeout=10.0)

        plat_command.height = goal.command.position
        res = self.plat_client.call(plat_command)
        # publish feedback

        while True:
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                rospy.loginfo('Lat Controller: Preempted.')
                return
            if res.result is True:
                break
        self.server.set_succeeded()
        rospy.loginfo("\033[1;36mplat controller complete action successful!\033[0m")
        
if __name__=='__main__':
    try:
        rospy.loginfo("\033[1;32mStart plat_controller!\033[0m")
        PlatActionController()
    except rospy.ROSInterruptException:
        rospy.loginfo('Hasta la Vista...')
