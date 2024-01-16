#!/usr/bin/env python
# encoding:utf8
from os import WIFSTOPPED
import os
from turtle import xcor
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_special.gpsr import * 
from smach_compose.compose import * 
from smach_common.common import * 
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

'''
Author: yishui
Date: 2022-06-28
LastEditTime: 2022-06-28 11:59:27
LastEditors: yishui
Description: Codes for GPSR
'''


class Gpsr():
    def __init__(self):
        rospy.init_node("Gpsr_Smach")
        rospy.logerr("Welcome to GPSR!")
        # empty pid file
        dir_path = '/home/xm/vision_pid/'
        for file in os.listdir(dir_path):
            with open(os.path.join(dir_path,file),'w') as f:
                f.write('')
        print('Empty PID Files Success!')
        
        #-------PICK_UP-------#
        #RUNNODE_IMG-->GETNAME-->GET_POS-->NAV-->FIND-->POS_JUS-->NAV-->RUNNODE_IMG-->FIND-->POS_JUS-->PICK-->SPEAK
        #运行图像节点，获取物体名字和大物体位置后，nav到大物体位置，然后用图像找物体，然后用pos_jus得出适宜抓取的坐标，nav后继续循环一次
        #在此之后，进行抓取
        #如果上面的图像没有找到物体，则通过speak进行错误反馈
        self.xm_Pick_up = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['target','current_task'])    
        with self.xm_Pick_up: 
            self.xm_Pick_up.userdata.name =''
            self.xm_Pick_up.userdata.target_mode =0
            self.xm_Pick_up.userdata.objmode = -1
            self.xm_Pick_up.userdata.go_counter = 2
            self.xm_Pick_up.userdata.distance = 0.5 #change this element from 0.7 to 0.2
            self.xm_Pick_up.userdata.table_depth = 0.04
            self.xm_Pick_up.userdata.traget_size = [0.04, 0.065, 0.105]
            self.xm_Pick_up.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
            self.xm_Pick_up.userdata.take_back_pose = [ [0,0.026,3.14,0,3.14,0] ]
            self.xm_Pick_up.userdata.mode_1 =1
            self.xm_Pick_up.userdata.object_pos = PointStamped()
            
            #StateMachine.add('PICK_POS',
            #                     ArmTrajectory(),#轨迹分析
            #                     transitions={'succeeded': 'FIND_1', 'error': 'error'},
            #                     remapping={'arm_waypoints':'pick_pose'})                                   
            
            StateMachine.add('FIND_1',
                                GetObjectPosition(),#区分物体，识别物体
                                transitions={'succeeded': 'GETPICKPOSE', 'error': 'error'})

            StateMachine.add('GETPICKPOSE',
                                GetPickPos(),
                                transitions ={'succeeded':'NAVTOPICKPOS','error':'error'})       
            
            StateMachine.add('NAVTOPICKPOS',
                                NavStack(),
                                transitions ={'succeeded':'FIND_2','aborted':'NAVTOPICKPOS','error':'error'},
                                remapping ={"pos_xm":'pick_pos'})

            StateMachine.add('FIND_2',
                                GetObjectPosition(),
                                transitions={'succeeded': 'TARGET_POINT_JUSFY', 'error': 'error'})   
                                                               
            StateMachine.add('TARGET_POINT_JUSFY',
                                TargerPosReload(),
                                transitions={'succeeded': 'PICK', 'error': 'error'})

            StateMachine.add('PICK', 
                                ArmStack(),
                                transitions={'succeeded': 'NAV_BACK','error': 'error'})
            self.xm_Pick_up.userdata.distance = -0.2
            StateMachine.add('NAV_BACK', 
                                GoAhead(),
                                transitions={'succeeded': 'succeeded','error': 'error'},
                                remapping={'move_len':'distance'})
            

         #顶层状态机
        self.xm_GPSR = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.xm_GPSR:
            
            self.xm_GPSR.userdata.target = ["spring"]
            self.xm_GPSR.userdata.action = ["pick"]
            self.xm_GPSR.userdata.task_num = 0
            self.xm_GPSR.userdata.answer = "this is the answer"
            # current_task目前运行到的任务序号，从-1开始每次判断下一步做什么时，将其值+1
            # 任务实质上是从0开始，0为任务一，2为任务三，3为结束
            self.xm_GPSR.userdata.current_task = -1 
            self.xm_GPSR.userdata.current_turn = -1 
            self.xm_GPSR.userdata.turn = 1              #the max_num of the mission
            self.xm_GPSR.userdata.pos_xm_door = gpsr_target['livingroom']['pos'] # 门的位置
            self.xm_GPSR.userdata.waypoint = gpsr_target['dining_room_exit']['pos']
            self.xm_GPSR.userdata.sentences = 'give me the mission please' #for the Speak() state
            self.xm_GPSR.userdata.gripper_release = 0.0
            self.xm_GPSR.userdata.give_file = "give_me_the_mission.wav"
            self.xm_GPSR.userdata.sentences_release = "i will release the gripper,please grap it"
            self.xm_GPSR.userdata.rec = 5.0
            self.xm_GPSR.userdata.character = ''
            self.xm_GPSR.userdata.gesture = ''
            self.xm_GPSR.userdata.count_num = 0
            self.xm_GPSR.userdata.objectName = ''

            StateMachine.add('PICK_UP',
                                self.xm_Pick_up,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'succeeded','aborted':'PICK_UP'})

        intro_server = IntrospectionServer('xm_gpsr',self.xm_GPSR,'/XM_ROOT')
        intro_server.start()
        out = self.xm_GPSR.execute()
        intro_server.stop()


if __name__ == "__main__":
    try:
        Gpsr()
    except:
        rospy.logerr(e)
        #print("7777777")
