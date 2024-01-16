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


        #------go to another room ,find objects and give them to the people---------#
        self.xm_Give_Object = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'],
            input_keys=["people_pos", 'target', 'action',
                        'people_correct_pos', 'turn', 'current_turn_num', 'people_name', 'name_pos_dict'],
            output_keys=["people_pos", 'target', 'action', 'people_correct_pos', 'turn', 'current_turn_num', 'people_name'])
        with self.xm_Give_Object:
            self.xm_Give_Object.userdata.pos_gone_bool = [0 for x in range(3)]

            self.xm_Give_Object.userdata.room_object_pos = Pose()
            self.xm_Give_Object.userdata.room_name = ''

            self.xm_Give_Object.userdata.room_object_pos1=gpsr_target['room_object_pos1']['pos']
            self.xm_Give_Object.userdata.room_object_pos2=gpsr_target['room_object_pos2']['pos']
            # 不同入口值不一样,A口为1,B口为2
            self.xm_Give_Object.userdata.room_pos_num=1

            self.xm_Give_Object.userdata.arm_give_waypoints = [0,-0.65,-1.41,-0.85]


            self.xm_Give_Object.userdata.object_current_pos = Pose()

            self.xm_Give_Object.userdata.current_people_pos = Pose()
            self.xm_Give_Object.userdata.current_people_name = ''
            self.xm_Give_Object.userdata.current_object_name = ''

            self.xm_Give_Object.userdata.degree=-0.5 #右转80度
            self.xm_Give_Object.userdata.gripper_commond=0
            self.xm_Give_Object.userdata.gripper_commond_close=1
            self.xm_Give_Object.userdata.rec=3

            self.xm_Give_Object.userdata.find_object_count = 1

            self.xm_Give_Object.userdata.find_sentences= 'i find the '
            self.xm_Give_Object.userdata.can_not_find_sentences= 'i can not find the object'
            self.xm_Give_Object.userdata.camera_num=0

           
            StateMachine.add('TRY_FIND_OBJECT1',
                             TryFindObject1(),
                             transitions={'succeeded': 'FIND_SPEAK1','giveup':'CAN_NOT_FIND_SPEAK1', 'aborted': 'CAN_NOT_FIND_SPEAK1','next':'CAN_NOT_FIND_SPEAK1','error': 'error'})
            StateMachine.add('FIND_SPEAK1',
                             FindSpeak(),
                             transitions={'succeeded': 'TRY_FIND_OBJECT2', 'aborted': 'FIND_SPEAK1','error': 'error'})
            StateMachine.add('CAN_NOT_FIND_SPEAK1',
                             SpeakText(),
                             transitions={'succeeded': 'TRY_FIND_OBJECT2', 'aborted': 'CAN_NOT_FIND_SPEAK1','error': 'error'},
                             remapping={'sentences':'can_not_find_sentences'})
            
            StateMachine.add('TRY_FIND_OBJECT2',
                             TryFindObject2(),
                             transitions={'succeeded': 'FIND_SPEAK2','giveup':'CAN_NOT_FIND_SPEAK2', 'aborted': 'TRY_FIND_OBJECT2','next':'TRY_FIND_OBJECT2','error': 'error'})
            StateMachine.add('FIND_SPEAK2',
                             FindSpeak(),
                             transitions={'succeeded': 'TRY_FIND_OBJECT3', 'aborted': 'FIND_SPEAK2','error': 'error'})
            StateMachine.add('CAN_NOT_FIND_SPEAK2',
                             SpeakText(),
                             transitions={'succeeded': 'TRY_FIND_OBJECT3', 'aborted': 'CAN_NOT_FIND_SPEAK2','error': 'error'},
                             remapping={'sentences':'can_not_find_sentences'})
            
            StateMachine.add('TRY_FIND_OBJECT3',
                             TryFindObject3(),
                             transitions={'succeeded': 'FIND_SPEAK3','giveup':'CAN_NOT_FIND_SPEAK3', 'aborted': 'TRY_FIND_OBJECT3','next':'TRY_FIND_OBJECT3','error': 'error'})
            StateMachine.add('FIND_SPEAK3',
                             FindSpeak(),
                             transitions={'succeeded': 'succeeded', 'aborted': 'FIND_SPEAK3','error': 'error'})
            StateMachine.add('CAN_NOT_FIND_SPEAK3',
                             SpeakText(),
                             transitions={'succeeded': 'succeeded', 'aborted': 'CAN_NOT_FIND_SPEAK3','error': 'error'},
                             remapping={'sentences':'can_not_find_sentences'})

        #顶层状态机
        self.xm_HandMeTHat = StateMachine(outcomes=['succeeded', 'aborted', 'error'])
        with self.xm_HandMeTHat:
        
            # 第一步到达指定位置
            self.xm_HandMeTHat.userdata.start_waypoint = gpsr_target['start_pos']['pos']
            # 离开的位置
            self.xm_HandMeTHat.userdata.waypoint = gpsr_target['exit_pos']['pos']
            # 每个人的大概位置
            self.xm_HandMeTHat.userdata.people_pos = list()
            # 需要取的物品
            self.xm_HandMeTHat.userdata.target = ["water", "sprite", "chip"]
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

            # 给每个人相应物品
            StateMachine.add('GIVE_OBJECT',
                             self.xm_Give_Object,
                             transitions={'succeeded': 'GO_OUT', 'aborted': 'GIVE_OBJECT', 'error': 'error'})
            # 离开
            StateMachine.add('GO_OUT',
                             NavStack(),
                             transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'waypoint'})

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