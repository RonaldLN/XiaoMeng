#!/usr/bin/env python
# encoding:utf8
import os
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_special.gpsr import *
from smach_special.garbage import *
from smach_special.HandMeThat_lib3 import *
from smach_compose.compose import *
from smach_common.common import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
import std_srvs.srv 



class HandMeThat():
    def __init__(self):
        rospy.init_node("Gpsr_Smach")
        rospy.on_shutdown(self.shutdown)
        rospy.logerr("Welcome to HandMeThat!")
        # empty pid file
        dir_path = '/home/xm/vision_pid/'
        for file in os.listdir(dir_path):
            with open(os.path.join(dir_path, file), 'w') as f:
                f.write('')
        print('Empty PID Files Success!')
        self.smach_bool = False



        #-------Enter_Room-------#
        # 任务开始，自动进入到房间指定点
        self.xm_EnterRoom = StateMachine(outcomes=['succeeded', 'aborted', 'error'],
                                         input_keys=['start_waypoint'])
        with self.xm_EnterRoom:
            StateMachine.add('NAV',
                             NavStack(),  # 移动到livingroom
                             transitions={'succeeded': 'succeeded',
                                          'aborted': 'NAV', 'error': 'error'},
                             remapping={'pos_xm': 'start_waypoint'})


        #顶层状态机
        self.xm_HandMeTHat = StateMachine(outcomes=['succeeded', 'aborted', 'error'])
        with self.xm_HandMeTHat:
        
            # 第一步到达指定位置
            self.xm_HandMeTHat.userdata.start_waypoint = gpsr_target['start_pos2']['pos']

            # gettopoint1
            self.xm_HandMeTHat.userdata.gettopoint2 = gpsr_target['gettopoint4']['pos']

            self.xm_HandMeTHat.userdata.room_object_pos1=gpsr_target['room_object_pos4']['pos']

            # 到达指定位置
            StateMachine.add('ENTERROOM',
                             self.xm_EnterRoom,
                             transitions={'succeeded': 'GO1', 'aborted': 'ENTERROOM', 'error': 'error'}) 
            # gettopoint1
            StateMachine.add('GO1',
                             NavStack(),
                             transitions={'succeeded': 'NAV_OBJECT', 'aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'gettopoint2'})

            StateMachine.add('NAV_OBJECT',
                             NavStack(),
                             transitions={'succeeded': 'NAV_OBJECT',
                                          'aborted': 'NAV_OBJECT', 'error': 'error'},
                             remapping={'pos_xm': 'room_object_pos1'})

        intro_server = IntrospectionServer('xm_HandMeThat', self.xm_HandMeTHat, '/XM_ROOT')
        intro_server.start()
        out = self.xm_HandMeTHat.execute()
        intro_server.stop()
        if out == 'succeeded':
            self.smach_bool = True

    def shutdown(self):
        if self.smach_bool == True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')
               
if __name__ == "__main__":
    try:
        HandMeThat()
    except:
        rospy.logerr(e)