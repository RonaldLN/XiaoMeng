#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import *
from smach_ros import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
'''
Author: jacy
Date: 2020-09-11 21:23:24
LastEditTime: 2020-09-13 11:26:11
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: /undefined/home/jacy/gazebo_test_ws/src/xm_smach/xm_smach/smach_lib/common/tool.py
'''
class Wait(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=["succeeded", 'error'],
                       input_keys=['rec'])
        self.rec = 5
    def execute(self, userdata):
        try:
            self.rec = userdata.rec
        except:
            rospy.logerr('no param specified')
            return 'error'
        else:
            rospy.sleep(self.rec)
            return "succeeded"
