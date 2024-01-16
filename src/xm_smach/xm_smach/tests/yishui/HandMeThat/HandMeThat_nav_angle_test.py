#!/usr/bin/env python
# encoding:utf8
from os import WIFSTOPPED
import os
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_special.gpsr import *
from smach_special.garbage import *
from smach_special.HandMeThat_lib import *
from smach_compose.compose import *
from smach_common.common import *
from xm_smach.target_gpsr import gpsr_target

from geometry_msgs.msg import *
import math
import subprocess
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse



'''
Author: yishui
Date: 2022-09-13
LastEditTime: 2022-09-13 18:17:05
LastEditors: yishui
Description: Codes for 2022中国机器人大赛通用项目
FilePath: ~/catkin_ws/src/xm_smach/xm_smach/tests/yishui/HandMeThat
'''


class HandMeThat():
    def __init__(self):
        rospy.init_node("Gpsr_Smach")
        rospy.on_shutdown(self.shutdown)
        rospy.logerr("Welcome to GPSR!")
        # empty pid file
        dir_path = '/home/xm/vision_pid/'
        for file in os.listdir(dir_path):
            with open(os.path.join(dir_path, file), 'w') as f:
                f.write('')
        print('Empty PID Files Success!')
        self.smach_bool = False


         # 顶层状态机
        self.xm_HandMeTHat = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'])
        with self.xm_HandMeTHat:

            self.xm_HandMeTHat.userdata.degree = 1.5

            StateMachine.add('TURN_DEGREE',
                    TurnDegree(),
                    transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'error': 'error'})

        intro_server = IntrospectionServer(
            'xm_HandMeThat', self.xm_HandMeTHat, '/XM_ROOT')
        intro_server.start()
        out = self.xm_HandMeTHat.execute()
        intro_server.stop()

    def shutdown(self):
        if self.smach_bool == True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

    # use for concurrence
if __name__ == "__main__":
    try:
        HandMeThat()
    except:
        rospy.logerr(e)
        # print("7777777")
