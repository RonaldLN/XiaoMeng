#!/usr/bin/env python
# encoding:utf8
from os import WIFSTOPPED
import os
from signal import set_wakeup_fd
from telnetlib import SE
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
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

'''
Author: yishui
Date: 2022-11-25
LastEditTime: 2022-11-25 7:28:40
LastEditors: yishui
Description: Codes for 2022中国机器人大赛通用项目 修改版3
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
        #-------Enter_Room-------#
        # 任务开始，首先要识别是否门已经打开，如果门已经打开，自动进入房间
        # 首先进入的房间一定是Livingroom
        self.xm_EnterRoom = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'],
            input_keys=['start_waypoint'])
        with self.xm_EnterRoom:

            self.xm_EnterRoom.userdata.rec = 3.0
            # StateMachine.add('WAIT',
            #                  Wait(),  # 等待三秒
            #                  remapping={'rec': 'rec'},
            #                  transitions={'succeeded': 'NAV', 'error': 'error'})

            StateMachine.add('NAV',
                             NavStack(),  # 移动到livingroom
                             transitions={'succeeded': 'succeeded',
                                          'aborted': 'NAV', 'error': 'error'},
                             remapping={'pos_xm': 'start_waypoint'})

        #-------PICK_UP-------#
        # RUNNODE_IMG-->GETNAME-->GET_POS-->NAV-->FIND-->POS_JUS-->NAV-->RUNNODE_IMG-->FIND-->POS_JUS-->PICK-->SPEAK
        # 运行图像节点，获取物体名字和大物体位置后，nav到大物体位置，然后用图像找物体，然后用pos_jus得出适宜抓取的坐标，nav后继续循环一次
        # 在此之后，进行抓取
        # 如果上面的图像没有找到物体，则通过speak进行错误反馈
        # target 为一个物品名称的字符串
        self.xm_Pick_up = StateMachine(outcomes=['succeeded', 'aborted', 'error'],
                                       input_keys=['target','target_camera_point'])
        with self.xm_Pick_up:
            self.xm_Pick_up.userdata.name ='person'
            self.xm_Pick_up.userdata.distance = 0.9  # change this element from 0.7 to 0.2
            # 桌子高度？
            self.xm_Pick_up.userdata.table_depth = 0.04
            # self.xm_Pick_up.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
            # self.xm_Pick_up.userdata.target = ['orange water']
            self.xm_Pick_up.userdata.pick_pos = Pose()
            self.xm_Pick_up.userdata.up_down=1
            # self.xm_Pick_up.userdata.target_camera_point = Pose()
            # StateMachine.add('PICK_POS',
                                # ArmTrajectory(),#轨迹分析
            #                     transitions={'succeeded': 'FIND_1', 'error': 'error'},
            #                     remapping={'arm_waypoints':'pick_pose'})

            # StateMachine.add('FIND_1',
                            #  GetObjectPosition(),  # 区分物体，识别物体
            #                  transitions={
            #                      'succeeded': 'GETPICKPOSE', 'error': 'error'})

            
            StateMachine.add('GETPICKPOSE',
                             GetPickPosBack(),
                             transitions={'succeeded': 'NAVTOPICKPOS', 'error': 'error'})
            
            StateMachine.add('NAVTOPICKPOS',
                             NavStack(),
                             transitions={'succeeded': 'TARGET_POINT_JUSFY',
                                          'aborted': 'NAVTOPICKPOS', 'error': 'error'},
                             remapping={"pos_xm": 'pick_pos'})

            StateMachine.add('FIND_2',
                             GetObjectPosition(),
                             transitions={'succeeded': 'TARGET_POINT_JUSFY', 'error': 'error'},
                             remapping={'target':'name'})
            
            StateMachine.add('TARGET_POINT_JUSFY',
                             TargerPosReload(),
                             transitions={'succeeded': 'PICK', 'error': 'error'})

            StateMachine.add('PICK',
                             ArmStack(),
                             transitions={'succeeded': 'NAV_BACK', 'error': 'error'})
            self.xm_Pick_up.userdata.distance = -0.1
            StateMachine.add('NAV_BACK',
                             GoAhead(),
                             transitions={'succeeded': 'succeeded', 'error': 'error'},
                             remapping={'move_len': 'distance'})

        #------find the people and get the all task---------#
        self.xm_Receive_Task = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'],
            input_keys=["people_pos", 'target', 'action', 'people_correct_pos', 'turn', 'people_name', 'name_pos_dict', 'pos_name_dict'],
            output_keys=["people_pos", 'target', 'action', 'people_correct_pos', 'turn', 'people_name', 'name_pos_dict', 'pos_name_dict'])
        with self.xm_Receive_Task:
            self.xm_Receive_Task.userdata.degree = -1.6  # 大约0.5是向左25度。1.5是向左75度。具体得导航确定一下
            self.xm_Receive_Task.userdata.start_sentences = 'I am service robot, what can I do for you'
            self.xm_Receive_Task.userdata.answer_sentences = 'The object you need is '
            self.xm_Receive_Task.userdata.remember_sentences = 'i have remember you'
            # 此地点需要看到所有的客人
            # self.xm_Receive_Task.userdata.all_guest_pos = gpsr_target['']['pos']
            # self.xm_Receive_Task.userdata.guest1_pos = gpsr_target['guest1_pos']['pos']
            # self.xm_Receive_Task.userdata.guest2_pos = gpsr_target['guest12_pos']['pos']
            # self.xm_Receive_Task.userdata.guest3_pos = gpsr_target['guest3_pos']['pos']

            # 目前正在寻找的客人编号-1
            self.xm_Receive_Task.userdata.present_guest_turn = 0
            #  目前正在寻找的客人坐标
            self.xm_Receive_Task.userdata.present_guest_pos = Pose()

            # 人脸识别并返回其代号
            # 返回所有人的位置
            # StateMachine.add('FIND_ALL_PEOPLE',
            #                  GetAllPersonPosition(),
            #                  transitions={'succeeded': 'CHECK_CURRENT_PEOPLE',
            #                               'aborted': 'FIND_ALL_PEOPLE', 'error': 'FIND_ALL_PEOPLE'})
            StateMachine.add('CHECK_CURRENT_PEOPLE',
                             CheckCurrentPerson(),
                             transitions={'succeeded': 'NAV_PEOPLE', 'finish': 'succeeded'})
            StateMachine.add('NAV_PEOPLE',
                             NavStack(),
                             transitions={'succeeded': 'SPEAK_GUEST',
                                          'aborted': 'NAV_PEOPLE', 'error': 'NAV_PEOPLE'},
                             remapping={'pos_xm': 'present_guest_pos'})
            # StateMachine.add('REMEMBER_PEOPLE',
            #                  GetTaskH(),
            #                  transitions={'succeeded': 'SPEAK_GUEST',
            #                               'aborted': 'REMEMBER_PEOPLE', 'error': 'REMEMBER_PEOPLE'})
            StateMachine.add('SPEAK_GUEST',
                             SpeakText(),
                             transitions={'succeeded': 'RECEIVE_TASKS',
                                          'aborted': 'SPEAK_GUEST', 'error': 'error'},
                             remapping={'sentences': 'start_sentences'})
            StateMachine.add('RECEIVE_TASKS',
                             GetTaskH(),
                             transitions={'succeeded': 'SPEAK_OBJECT',
                                          'aborted': 'RECEIVE_TASKS', 'error': 'error'})
            StateMachine.add('SPEAK_OBJECT',
                             SpeakAnswer1(),
                             transitions={'succeeded': 'SPEAK_REMEMBER',
                                          'aborted': 'SPEAK_OBJECT', 'error': 'error'},
                             remapping={'sentences': 'answer_sentences', 'object': 'target'})
            StateMachine.add('GET_PEOPLE_NAME',
                            #  FaceRecognition1(),
                             FaceRegister1(),
                             transitions={'succeeded': 'SPEAK_REMEMBER',
                                          'aborted': 'GET_PEOPLE_NAME', 'error': 'error'})
            StateMachine.add('SPEAK_REMEMBER',
                             SpeakText(),
                             transitions={'succeeded': 'succeeded',
                                          'aborted': 'SPEAK_REMEMBER', 'error': 'error'},
                             remapping={'sentences': 'remember_sentences'})

        #------go to another room ,find objects and give them to the people---------#
        self.xm_Give_Object = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'],
            input_keys=["people_pos", 'target', 'action',
                        'people_correct_pos', 'turn', 'current_turn_num', 'people_name', 'name_pos_dict'],
            output_keys=["people_pos", 'target', 'action', 'people_correct_pos', 'turn', 'current_turn_num', 'people_name'])
        with self.xm_Give_Object:
            self.xm_Give_Object.userdata.pos_gone_bool = [0 for x in range(3)]
            # self.xm_Give_Object.userdata.room_num = 2  # 该房间可以停的位置个数
            # self.xm_Give_Object.userdata.room_pos = list()  # 该房间可以停的位置列表
            # self.xm_Give_Object.userdata.current_turn = 0 # 当前抓的物品的序号
            # self.xm_Give_Object.userdata.current_people_pos = Pose()
            # self.xm_Give_Object.userdata.current_room_num = 1 # 当前家具的编号
            # self.xm_Give_Object.userdata.current_guest_num = 1 # 当前爪子抓客人的编号,从0开始
            # 物品所在的房间
            # self.xm_Give_Object.userdata.room_pos = Pose()
            self.xm_Give_Object.userdata.room_object_pos = Pose()
            self.xm_Give_Object.userdata.room_name = ''

            self.xm_Give_Object.userdata.room_object_pos1=gpsr_target['room_object_pos1']['pos']
            self.xm_Give_Object.userdata.room_object_pos2=gpsr_target['room_object_pos2']['pos']
            self.xm_Give_Object.userdata.room_pos_num=1


            self.xm_Give_Object.userdata.arm_give_waypoints = [0,-0.65,-1.41,-0.85]
            # self.xm_Give_Object.userdata.living_room_pos = gpsr_target['livingroom']['pos']
            # self.xm_Give_Object.userdata.dining_room_pos = gpsr_target['diningroom']['pos']
            # self.xm_Give_Object.userdata.kitchen_room_pos = gpsr_target['kitchen']['pos']
            # self.xm_Give_Object.userdata.bedroom_pos = gpsr_target['bedroom']['pos']

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


            # 当前回合里去过的地方置为1，每次成功递出后记得重置
            # self.xm_Give_Object.userdata.current_pos_gone_bool=[0 for x in range(3)]

            # self.xm_Give_Object.userdata.arm_give_waypoints = [[0, 0.026, 3.14, 0]]

            # StateMachine.add('GET_ROOM',
                            #  GetRoom(),
            #                  transitions={'succeeded': 'NAV',
            #                               'aborted': 'GET_ROOM', 'error': 'GET_ROOM'})
            # StateMachine.add('NAV',
            #                  NavStack(),
            #                  transitions={'succeeded': 'FIND_ALL_OBJECT',
            #                               'aborted': 'NAV', 'error': 'error'},
            #                  remapping={'pos_xm': 'room_pos'})
            # 选择该房间应该去的位置
            # 位置变量名由room_name+‘_object_pos1(2,3)’组成
            StateMachine.add('FIND_OBJECT_POS',
                             FindObjectPos(),
                             transitions={'succeeded': 'NAV_OBJECT',
                                          'aborted': 'FIND_OBJECT_POS', 'error': 'FIND_OBJECT_POS'})
            StateMachine.add('NAV_OBJECT',
                             NavStack(),
                             transitions={'succeeded': 'TRY_FIND_OBJECT',
                                          'aborted': 'NAV_OBJECT', 'error': 'error'},
                             remapping={'pos_xm': 'room_object_pos'})
            StateMachine.add('TRY_FIND_OBJECT',
                             TryFindObject1(),
                             transitions={'succeeded': 'FIND_SPEAK','giveup':'CAN_NOT_FIND_SPEAK', 'aborted': 'TRY_FIND_OBJECT','next':'FIND_OBJECT_POS','error': 'error'})
            StateMachine.add('CAN_NOT_FIND_SPEAK',
                             SpeakText(),
                             transitions={'succeeded': 'WAIT', 'aborted': 'CAN_NOT_FIND_SPEAK','error': 'error'},
                             remapping={'sentences':'can_not_find_sentences'})
            StateMachine.add('WAIT',
                             Wait(),  # 等待三秒
                             remapping={'rec': 'rec'},
                             transitions={'succeeded': 'GRIPPER_CLOSE', 'error': 'error'})
            StateMachine.add('GRIPPER_CLOSE',
                             GripperCommond(),
                             transitions={'succeeded': 'FIND_GUEST','error': 'error'},
                             remapping={'commond':'gripper_commond_close'})
            StateMachine.add('FIND_SPEAK',
                             FindSpeak(),
                             transitions={'succeeded': 'PICK_UP', 'aborted': 'FIND_SPEAK','error': 'error'})           
            StateMachine.add('PICK_UP',
                             self.xm_Pick_up,
                             transitions={
                                 'succeeded': 'FIND_GUEST', 'aborted': 'PICK_UP','error': 'error'},
                             remapping={'target': 'current_object_name'})

            # # 一眼看到所有物品，并选择传过去的那一个进行识别，传回一个距离物体两米远的位置
            # # 确定此回合中current_object_name的值
            # StateMachine.add('FIND_ALL_OBJECT',
            #                  FindAllObject(),
            #                  transitions={'succeeded': 'CORRECT_OBJECT_POS', 'aborted': 'TURN_DEGREE', 'error': 'error'})
            # # StateMachine.add('FIND_OBJECT_COUNT',
            # #                  FindObjectCount(),
            # #                  transitions={'succeeded': 'CORRECT_OBJECT_POS', 'aborted': 'TURN_DEGREE', 'error': 'error'})
            # StateMachine.add('TURN_DEGREE',
            #                  TurnDegree(),
            #                  transitions={'succeeded': 'FIND_ALL_OBJECT', 'aborted': 'TURN_DEGREE', 'error': 'error'})           
            # # 确定物品大概的位置，走到距其2米左右位置
            # StateMachine.add('CORRECT_OBJECT_POS',
            #                  GetPickPos1(),
            #                  transitions={'succeeded': 'NAV_PICK_OBJECT','aborted':'CORRECT_OBJECT_POS', 'error': 'error'})
            # StateMachine.add('NAV_PICK_OBJECT',
            #                  NavStack(),
            #                  transitions={'succeeded': 'PICK_UP',
            #                               'aborted': 'NAV_PICK_OBJECT', 'error': 'error'},
            #                  remapping={'pos_xm': 'object_current_pos'})


            StateMachine.add('FIND_GUEST',
                             FindGuest(),
                             transitions={'succeeded': 'NAV_BACK_HALL', 'error': 'error'})

            StateMachine.add('NAV_BACK_HALL',
                             NavStack(),  # 返回物品对应的客人位置?
                             transitions={
                                 'succeeded': 'COMPARE_PEOPLE_NAME', 'aborted': 'NAV_BACK_HALL', 'error': 'error'},
                             remapping={'pos_xm': 'current_people_pos'})
            # 人脸识别是否可以与当前客人，如果正确识别返回succeeded，如果不正确aborted返回FIND_GUEST_AGAIN,重新选一个客人
            StateMachine.add('COMPARE_PEOPLE_NAME',
                             FaceRecognition(),
                             transitions={'succeeded': 'GIVE_OBJECT',
                                          'aborted': 'FIND_GUEST_AGAIN', 'error': 'error'})
            # 重新选择位置，修改为下一个位置，如果是最后一个位置，直接递给他
            StateMachine.add('FIND_GUEST_AGAIN',
                             FindGuestAgain(),
                             transitions={'succeeded': 'NAV_BACK_HALL','finish': 'GIVE_OBJECT', 'error': 'error'})
            StateMachine.add('GIVE_OBJECT',
                             ArmTrajectory(),  # 机械臂摆一个递给人东西的动作
                             transitions={
                                 'succeeded': 'SPEAK_RELEASE', 'error': 'error'},
                             remapping={'arm_waypoints': 'arm_give_waypoints'})

            StateMachine.add('SPEAK_RELEASE',
                             SpeakText(),  # 提示人抓取物品
                             transitions={'succeeded': 'OPEN_GRIPPER',
                                          'aborted': 'SPEAK_RELEASE', 'error': 'error'},
                             remapping={'sentences': 'sentences_release'})
            StateMachine.add('OPEN_GRIPPER',
                             GripperCommond(),  
                             transitions={'succeeded': 'CHECK_CURRENT_OBJECT_NUM','error': 'error'},
                             remapping={'commond':'gripper_commond'})           
            #
            StateMachine.add('CHECK_CURRENT_OBJECT_NUM',
                             CheckCurrentObjectNum(),  # 检查下一个应该找的物品
                             transitions={'succeeded': 'NAV_OBJECT', 'finish': 'succeeded', 'error': 'error'})


         # 顶层状态机
        self.xm_HandMeTHat = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'])
        with self.xm_HandMeTHat:
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
            # self.xm_HandMeTHat.userdata.answer = "this is the answer"
            # 总任务个数
            self.xm_HandMeTHat.userdata.turn = 3

            # self.xm_HandMeTHat.userdata.pos_xm_door = gpsr_target['open_door_pose']['pos']
            self.xm_HandMeTHat.userdata.waypoint = gpsr_target['exit_pos']['pos']
            self.xm_HandMeTHat.userdata.start_waypoint = gpsr_target['start_pos']['pos']
            # self.xm_HandMeTHat.userdata.start_waypoint = gpsr_target['livingroom']['pos']
            # for the Speak() state
            # self.xm_HandMeTHat.userdata.sentences = 'give me the mission please'
            self.xm_HandMeTHat.userdata.gripper_release = 0.0
            self.xm_HandMeTHat.userdata.give_file = "give_me_the_mission.wav"
            self.xm_HandMeTHat.userdata.sentences_release = "i will release the gripper,please grap it"
            self.xm_HandMeTHat.userdata.rec = 5.0
            # StateMachine.add('OPEN_DOOR_DETECT',
                            #  OpenDoorDetect(),  # 检测门是否打开
            #                  transitions={'succeeded': 'ENTERROOM', 'aborted': 'OPEN_DOOR_DETECT', 'error': 'error'})
            # 到达指定位置
            StateMachine.add('ENTERROOM',
                             self.xm_EnterRoom,
                             transitions={'succeeded': 'FIND_ALL_PEOPLE', 'aborted': 'ENTERROOM', 'error': 'error'})
            # StateMachine.add('FIND_ALL_PEOPLE',
            #                  FindAllPeople(),  # 识别到三个人的位置，计算出全局位置并返回
            #                  transitions={'succeeded': 'RECEIVE_TASKS', 'aborted': 'FIND_ALL_PEOPLE', 'error': 'error'})
            StateMachine.add('FIND_ALL_PEOPLE',
                             FindAllPeople2(),  # 识别到三个人的位置，计算出全局位置并返回
                             transitions={'succeeded': 'RECEIVE_TASKS', 'aborted': 'FIND_ALL_PEOPLE', 'error': 'error'})

            StateMachine.add('RECEIVE_TASKS',
                             self.xm_Receive_Task,
                             transitions={'succeeded': 'GIVE_OBJECT',
                                          'aborted': 'RECEIVE_TASKS', 'error': 'error'})

            StateMachine.add('GIVE_OBJECT',
                             self.xm_Give_Object,
                             transitions={'succeeded': 'GO_OUT',
                                          'aborted': 'GIVE_OBJECT', 'error': 'error'})

            StateMachine.add('GO_OUT',
                             NavStack(),
                             transitions={'succeeded': 'succeeded',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'waypoint'}
                             )

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


    # use for concurrence
if __name__ == "__main__":
    try:
        HandMeThat()
    except:
        rospy.logerr(e)
        # print("7777777")
