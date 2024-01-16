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
            
            # 目前正在寻找的客人编号-1
            self.xm_Receive_Task.userdata.present_guest_turn = 0
            # 目前正在寻找的客人坐标
            self.xm_Receive_Task.userdata.present_guest_pos = Pose()

            # 人脸识别并返回其代号
            # 返回所有人的位置
            StateMachine.add('CHECK_CURRENT_PEOPLE',
                             CheckCurrentPerson(),
                             transitions={'succeeded': 'NAV_PEOPLE', 'finish': 'succeeded'})
            StateMachine.add('NAV_PEOPLE',
                             NavStack(),
                             transitions={'succeeded': 'SPEAK_GUEST',
                                          'aborted': 'NAV_PEOPLE', 'error': 'NAV_PEOPLE'},
                             remapping={'pos_xm': 'present_guest_pos'})
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
                             SpeakAnswer2(),
                             transitions={'succeeded1': 'GET_PEOPLE_NAME1','succeeded2': 'GET_PEOPLE_NAME2','succeeded3': 'GET_PEOPLE_NAME3',
                                          'aborted': 'SPEAK_OBJECT', 'error': 'error'},
                             remapping={'sentences': 'answer_sentences', 'object': 'target'})
            StateMachine.add('GET_PEOPLE_NAME1',
                             FaceRegister1(),
                             transitions={'succeeded': 'SPEAK_REMEMBER',
                                          'aborted': 'GET_PEOPLE_NAME1', 'error': 'error'})
            StateMachine.add('GET_PEOPLE_NAME2',
                             FaceRegister2(),
                             transitions={'succeeded': 'SPEAK_REMEMBER',
                                          'aborted': 'GET_PEOPLE_NAME2', 'error': 'error'})
            StateMachine.add('GET_PEOPLE_NAME3',
                             FaceRegister3(),
                             transitions={'succeeded': 'SPEAK_REMEMBER',
                                          'aborted': 'GET_PEOPLE_NAME3', 'error': 'error'})
            StateMachine.add('SPEAK_REMEMBER',
                             SpeakText(),
                             transitions={'succeeded': 'CHECK_CURRENT_PEOPLE',
                                          'aborted': 'SPEAK_REMEMBER', 'error': 'error'},
                             remapping={'sentences': 'remember_sentences'})

        #-------PICK_UP-------#
        # RUNNODE_IMG-->GETNAME-->GET_POS-->NAV-->FIND-->POS_JUS-->NAV-->RUNNODE_IMG-->FIND-->POS_JUS-->PICK-->SPEAK
        # 运行图像节点，获取物体名字和大物体位置后，nav到大物体位置，然后用图像找物体，然后用pos_jus得出适宜抓取的坐标，nav后继续循环一次
        # 在此之后，进行抓取
        # 如果上面的图像没有找到物体，则通过speak进行错误反馈
        # target 为一个物品名称的字符串
        self.xm_Pick_up = StateMachine(outcomes=['succeeded', 'aborted', 'error'],
                                       input_keys=['target','target_camera_point'])
        with self.xm_Pick_up:
            # self.xm_Pick_up.userdata.name = 'perso'
            self.xm_Pick_up.userdata.distance = 0.9  # change this element from 0.7 to 0.2
            # 桌子高度？
            self.xm_Pick_up.userdata.table_depth = 0.04
            # self.xm_Pick_up.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
            # self.xm_Pick_up.userdata.target = ['orange water']
            self.xm_Pick_up.userdata.pick_pos = Pose()
            self.xm_Pick_up.userdata.up_down=1
            # self.xm_Pick_up.userdata.target_camera_point = Pose()
            # StateMachine.add('PICK_POS',
            #                     ArmTrajectory(),#轨迹分析
            #                     transitions={'succeeded': 'FIND_1', 'error': 'error'},
            #                     remapping={'arm_waypoints':'pick_pose'})

            # StateMachine.add('FIND_1',
            #                  GetObjectPosition(),  # 区分物体，识别物体
            #                  transitions={
            #                      'succeeded': 'GETPICKPOSE', 'error': 'error'})
            # 返回抓取的位置坐标
            StateMachine.add('GETPICKPOSE',
                             GetPickPosBack(),
                             transitions={'succeeded': 'NAVTOPICKPOS', 'error': 'error'})
            # 移动到坐标
            StateMachine.add('NAVTOPICKPOS',
                             NavStack(),
                             transitions={'succeeded': 'TARGET_POINT_JUSFY',
                                          'aborted': 'NAVTOPICKPOS', 'error': 'error'},
                             remapping={"pos_xm": 'pick_pos'})

            StateMachine.add('FIND_2',
                             GetObjectPosition(),
                             transitions={'succeeded': 'TARGET_POINT_JUSFY', 'error': 'error'},
                             remapping={'target':'name'})
            # 修正图像传过来的物体坐标等信息
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
            self.xm_Give_Object.userdata.room_object_pos3=gpsr_target['room_object_pos3']['pos']
            # 不同入口值不一样,A口为1,B口为2
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

            # 选择该房间应该去的位置
            # 位置变量名由room_name+‘_object_pos1(2,3)’组成

            # 给需要到达的地点,从不同入口进入不一样,A口为1,B口为2
            StateMachine.add('FIND_OBJECT_POS',
                             FindObjectPos(),
                             transitions={'succeeded': 'NAV_OBJECT1',
                                          'aborted': 'FIND_OBJECT_POS', 'error': 'FIND_OBJECT_POS'})
            # 到位置处
            StateMachine.add('NAV_OBJECT1',
                             NavStack(),
                             transitions={'succeeded': 'TRY_FIND_OBJECT1',
                                          'aborted': 'NAV_OBJECT1', 'error': 'error'},
                             remapping={'pos_xm': 'room_object_pos1'})

            # 摄像头找
            StateMachine.add('TRY_FIND_OBJECT1',
                             TryFindObject1(),
                             transitions={'succeeded': 'FIND_SPEAK1','giveup':'CAN_NOT_FIND_SPEAK1', 'aborted': 'CAN_NOT_FIND_SPEAK1','next':'CAN_NOT_FIND_SPEAK1','error': 'error'})
            StateMachine.add('FIND_SPEAK1',
                             FindSpeak(),
                             transitions={'succeeded': 'TRY_FIND_OBJECT2', 'aborted': 'FIND_SPEAK1','error': 'error'})
            StateMachine.add('CAN_NOT_FIND_SPEAK1',
                             SpeakText(),
                             transitions={'succeeded': 'NAV_OBJECT2', 'aborted': 'CAN_NOT_FIND_SPEAK1','error': 'error'},
                             remapping={'sentences':'can_not_find_sentences'})
            StateMachine.add('NAV_OBJECT2',
                             NavStack(),
                             transitions={'succeeded': 'TRY_FIND_OBJECT1.5',
                                          'aborted': 'NAV_OBJECT2', 'error': 'error'},
                             remapping={'pos_xm': 'room_object_pos2'})
            StateMachine.add('TRY_FIND_OBJECT1.5',
                             TryFindObject1(),
                             transitions={'succeeded': 'FIND_SPEAK1','giveup':'CAN_NOT_FIND_SPEAK2', 'aborted': 'CAN_NOT_FIND_SPEAK2','next':'CAN_NOT_FIND_SPEAK2','error': 'error'})
            StateMachine.add('CAN_NOT_FIND_SPEAK2',
                             SpeakText(),
                             transitions={'succeeded': 'NAV_OBJECT3', 'aborted': 'CAN_NOT_FIND_SPEAK2','error': 'error'},
                             remapping={'sentences':'can_not_find_sentences'})
            
            StateMachine.add('NAV_OBJECT3',
                             NavStack(),
                             transitions={'succeeded': 'TRY_FIND_OBJECT1',
                                          'aborted': 'NAV_OBJECT3', 'error': 'error'},
                             remapping={'pos_xm': 'room_object_pos3'})

            StateMachine.add('TRY_FIND_OBJECT2',
                             TryFindObject2(),
                             transitions={'succeeded': 'FIND_SPEAK2','giveup':'FIND_SPEAK2', 'aborted': 'TRY_FIND_OBJECT2','next':'TRY_FIND_OBJECT2','error': 'error'})
            StateMachine.add('FIND_SPEAK2',
                             FindSpeak(),
                             transitions={'succeeded': 'TRY_FIND_OBJECT3', 'aborted': 'FIND_SPEAK2','error': 'error'})
            # StateMachine.add('CAN_NOT_FIND_SPEAK2',
            #                  SpeakText(),
            #                  transitions={'succeeded': 'TRY_FIND_OBJECT3', 'aborted': 'CAN_NOT_FIND_SPEAK2','error': 'error'},
            #                  remapping={'sentences':'can_not_find_sentences'})
            
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
            # gettopoint1
            self.xm_HandMeTHat.userdata.gettopoint1 = gpsr_target['gettopoint1']['pos']
            # gettopoint2
            self.xm_HandMeTHat.userdata.gettopoint2 = gpsr_target['gettopoint2']['pos']

            # 每个人的大概位置
            self.xm_HandMeTHat.userdata.people_pos = list()
            # 需要取的物品
            self.xm_HandMeTHat.userdata.target = list()
            # self.xm_HandMeTHat.userdata.target = ["chip", "orange juice", "cookie"]
            
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

            # # 到达指定位置
            StateMachine.add('ENTERROOM',
                             self.xm_EnterRoom,
                             transitions={'succeeded': 'GRTTOPOINT1', 'aborted': 'ENTERROOM', 'error': 'error'}) 
            # 1
            StateMachine.add('GRTTOPOINT1',
                             NavStack(),
                             transitions={'succeeded': 'FIND_ALL_PEOPLE', 'aborted': 'GRTTOPOINT1', 'error': 'error'},
                             remapping={"pos_xm": 'gettopoint1'})
            # 识别到三个人的位置，计算出全局位置并返回
            StateMachine.add('FIND_ALL_PEOPLE',
                             FindAllPeople2(),
                             transitions={'succeeded': 'RECEIVE_TASKS', 'aborted': 'GRTTOPOINT2', 'error': 'error'})      
            # 2
            StateMachine.add('GRTTOPOINT2',
                             NavStack(),
                             transitions={'succeeded': 'FIND_ALL_PEOPLE', 'aborted': 'GRTTOPOINT2', 'error': 'error'},
                             remapping={"pos_xm": 'gettopoint2'}) 
            
            # # 语音问物品,人脸注册
            StateMachine.add('RECEIVE_TASKS',
                             self.xm_Receive_Task,
                             transitions={'succeeded': 'GIVE_OBJECT',
                                          'aborted': 'RECEIVE_TASKS', 'error': 'error'})
            
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