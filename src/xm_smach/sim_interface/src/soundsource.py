#!/usr/bin/env python
#coding=utf-8
'''
Author: jacy
Date: 2020-09-04 21:49:25
LastEditTime: 2020-09-15 15:07:33
LastEditors: Please set LastEditors
Description: get position
FilePath: /undefined/home/jacy/gazebo_test_ws/src/xm_smach/sim_interface/src/xm_arm_sim_hw.py
'''
import rospy, sys
import roslib
from xm_msgs.srv import *
from xm_msgs.msg import *
import random

class GetSoundSource:
    def __init__(self):
        
        rospy.init_node('soundsource', anonymous = True)

        rospy.Service('/mobile_base/SoundSource', SoundSource, self.callback)

        rospy.loginfo("soundsource has work")
        rospy.spin()

    def callback(self,req):
        
        res = SoundSourceResponse()
        res.angle = random.randint(0,360)
        rospy.loginfo("the angle of the sounce is :"+str(res.angle))
        return res

if __name__ == '__main__':
    GetSoundSource()
    
    
