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
            # 离开的位置
            self.xm_HandMeTHat.userdata.waypoint = gpsr_target['exit_pos']['pos']
            # 每个人的大概位置
            self.xm_HandMeTHat.userdata.people_pos = list()
            # 需要取的物品
            self.xm_HandMeTHat.userdata.target = list()
            self.xm_HandMeTHat.userdata.action = list()
            self.xm_HandMeTHat.userdata.people_name = list()
            # 每个人的大概位置
            self.xm_HandMeTHat.userdata.people_pos = list()
            # 人和物品的映射关系
            self.xm_HandMeTHat.userdata.name_pos_dict = {}
            # 物品和人的映射关系
            self.xm_HandMeTHat.userdata.pos_name_dict = {}
            # 每个人的精确位置
            self.xm_HandMeTHat.userdata.people_correct_pos = list()
            # 已经成功运送的物品个数/当前以及
            self.xm_HandMeTHat.userdata.current_turn_num = 0
            # 总任务个数
            self.xm_HandMeTHat.userdata.turn = 3

            # gettopoint1
            self.xm_HandMeTHat.userdata.gettopoint1 = gpsr_target['gettopoint3']['pos']
            # gettopoint2
            self.xm_HandMeTHat.userdata.gettopoint2 = gpsr_target['gettopoint4']['pos']

            self.xm_HandMeTHat.userdata.room_object_pos1=gpsr_target['room_object_pos4']['pos']
            self.xm_HandMeTHat.userdata.room_object_pos2=gpsr_target['room_object_pos5']['pos']
            self.xm_HandMeTHat.userdata.room_object_pos3=gpsr_target['room_object_pos6']['pos']


            # 到达指定位置
            StateMachine.add('ENTERROOM',
                             self.xm_EnterRoom,
                             transitions={'succeeded': 'GO1', 'aborted': 'ENTERROOM', 'error': 'error'}) 
            # 离开
            StateMachine.add('GO1',
                             NavStack(),
                             transitions={'succeeded': 'GO2', 'aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'gettopoint1'})

            StateMachine.add('GO2',
                             NavStack(),
                             transitions={'succeeded': 'GO_in1', 'aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'gettopoint2'})

            StateMachine.add('GO_in1',
                             NavStack(),
                             transitions={'succeeded': 'GO_in2', 'aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'room_object_pos1'})            
            
            StateMachine.add('GO_in2',
                             NavStack(),
                             transitions={'succeeded': 'GO_in3', 'aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'room_object_pos2'})

            StateMachine.add('GO_in3',
                             NavStack(),
                             transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'room_object_pos3'})

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