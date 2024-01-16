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
Date: 2022-09-12
LastEditTime: 2022-09-12 14:02:52
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
        #-------Enter_Room-------#
        # 任务开始，首先要识别是否门已经打开，如果门已经打开，自动进入房间
        # 首先进入的房间一定是Livingroom
        self.xm_EnterRoom = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'])
        with self.xm_EnterRoom:

            self.xm_EnterRoom.userdata.rec = 3.0
            # self.xm_EnterRoom.userdata.start_waypoint = gpsr_target['livingroom']['pos']
            StateMachine.add('WAIT',
                             Wait(),  # 等待三秒
                             remapping={'rec': 'rec'},
                             transitions={'succeeded': 'NAV', 'error': 'error'})

            StateMachine.add('NAV',
                             NavStack(),  # 移动到livingroom
                             transitions={'succeeded': 'succeeded',
                                          'aborted': 'NAV', 'error': 'error'},
                             remapping={'pos_xm': 'start_waypoint'})

        #-------FIND_PEOPLE_AND_GO-------#
        #
        # 如果可以看到人，就直接直接跟过去，判断距离，然后问话。看不到人的话，就和follow一样运行
        # 跟人用follow的节点，打开摄像头，并发状态机，如果距离太近就结束这个状态，开始问话
        # self.findPeopleAndGo =StateMachine(outcomes=['succeeded', 'aborted','error'],
        #                                input_keys=[''])





        #-------PICK_UP-------#
        # RUNNODE_IMG-->GETNAME-->GET_POS-->NAV-->FIND-->POS_JUS-->NAV-->RUNNODE_IMG-->FIND-->POS_JUS-->PICK-->SPEAK
        # 运行图像节点，获取物体名字和大物体位置后，nav到大物体位置，然后用图像找物体，然后用pos_jus得出适宜抓取的坐标，nav后继续循环一次
        # 在此之后，进行抓取
        # 如果上面的图像没有找到物体，则通过speak进行错误反馈
        self.xm_Pick_up = StateMachine(outcomes=['succeeded', 'aborted','finished', 'error'],
                                       input_keys=['target','current_turn_num'])
        with self.xm_Pick_up:
            self.xm_Pick_up.userdata.name = ''
            self.xm_Pick_up.userdata.target_mode = 0
            self.xm_Pick_up.userdata.objmode = -1
            self.xm_Pick_up.userdata.go_counter = 2
            self.xm_Pick_up.userdata.distance = 0.5  # change this element from 0.7 to 0.2
            self.xm_Pick_up.userdata.table_depth = 0.04
            self.xm_Pick_up.userdata.traget_size = [0.04, 0.065, 0.105]
            self.xm_Pick_up.userdata.pick_pose = [[0, -1.5, 3.14, 0]]
            self.xm_Pick_up.userdata.take_back_pose = [[0, 0.026, 3.14, 0]]
            self.xm_Pick_up.userdata.mode_1 = 1
            self.xm_Pick_up.userdata.gripper_commond = 0
            self.xm_Pick_up.userdata.object_pos = PointStamped()
            self.xm_Pick_up.userdata.find_sentences='I find it '

            # StateMachine.add('PICK_POS',
            #                  ArmTrajectory(),  # 轨迹分析
            #                  transitions={
            #                      'succeeded': 'FIND_1', 'error': 'error'},
            #                  remapping={'arm_waypoints': 'pick_pose'})

            StateMachine.add('FIND_1',
                             GetTargetPosition1(),  # 区分物体，识别物体
                             transitions={'succeeded': 'NAVTOPICKPOS','aborted':'FIND_1','finished':'finished','error': 'error'})
            # StateMachine.add('GETPICKPOSE',
            #                  GetPickPos(),
            #                  transitions={'succeeded': 'NAVTOPICKPOS', 'error': 'error'})

            StateMachine.add('NAVTOPICKPOS',
                             NavStack(),
                             transitions={
                                 'succeeded': 'SPEAK_FIND', 'aborted': 'NAVTOPICKPOS', 'error': 'error'},
                             remapping={"pos_xm": 'pick_pos'})

            StateMachine.add('SPEAK_FIND',
                             SpeakText(),
                             transitions={
                                 'succeeded': 'FIND_2', 'aborted': 'SPEAK_FIND', 'error': 'error'},
                             remapping={"sentences": 'find_sentences'})

            StateMachine.add('FIND_2',
                             GetTargetPosition(),
                             transitions={'succeeded': 'PLACE', 'error': 'error'})

            # StateMachine.add('TARGET_POINT_JUSFY',
            #                  TargerPosReload(),
            #                  transitions={'succeeded': 'PICK', 'error': 'error'})

            # StateMachine.add('PICK',
            #                  ArmStack(),
            #                  transitions={'succeeded': 'NAV_BACK', 'error': 'error'})
            StateMachine.add('PLACE',
                             GripperCommond(),  # 控制爪子（打开）
                             transitions={'succeeded': 'NAV_BACK','error': 'error'},
                             remapping={'commond':'gripper_commond'})
            self.xm_Pick_up.userdata.distance = -0.2

            StateMachine.add('NAV_BACK',
                             GoAhead(),
                             transitions={
                                 'succeeded': 'succeeded', 'error': 'error'},
                             remapping={'move_len': 'distance'})

            # TODO arm back pose
            # StateMachine.add('BACK_POS',
            #                  ArmTrajectory(),
            #                  transitions={
            #                      'succeeded': 'succeeded', 'error': 'error'},
            #                  remapping={'arm_waypoints': 'take_back_pose'})

        # #-------Put_down---------#
        # self.xm_Put_down = StateMachine(outcomes =['succeeded','aborted','error'],
        #                             input_keys = ['target'])
        # with self.xm_Put_down:    
        #     self.xm_Put_down.userdata.commond =0
        #     self.xm_Put_down.userdata.put_down_pos = gpsr_target['put_down']['pos']
        #     self.xm_Put_down.userdata.put_pose = [ [0,-1.5,3.14,0] ]
        #     self.xm_Put_down.userdata.name =''
        #     self.xm_Put_down.userdata.target_mode =0
        #     self.xm_Put_down.userdata.objmode = -1
        #     self.xm_Put_down.userdata.go_counter = 2
        #     self.xm_Put_down.userdata.distance = 0.5 #change this element from 0.7 to 0.2
        #     self.xm_Put_down.userdata.table_depth = 0.04
        #     self.xm_Put_down.userdata.traget_size = [0.04, 0.065, 0.105]
        #     self.xm_Put_down.userdata.pick_pose = [ [0,-1.5,3.14,0] ]
        #     self.xm_Put_down.userdata.take_back_pose = [ [0,0.026,3.14,0] ]
        #     self.xm_Put_down.userdata.mode_1 =1
        #     self.xm_Put_down.userdata.object_pos = PointStamped()

            
        #     StateMachine.add('FIND_PUT_PLACE_1',
        #                         GetPutPosition(),
        #                         transitions={'succeeded':'TARGET_POINT_JUSFY','aborted':'FIND_PUT_PLACE_1','error':'error'},)
            
        #     # StateMachine.add('GETPUTPOSE',
        #     #                     GetPutPos(),
        #     #                     transitions ={'succeeded':'NAVTOPICKPOS','error':'error'}) 
            
        #     # StateMachine.add('NAVTOPICKPOS',
        #     #                     NavStack(),
        #     #                     transitions ={'succeeded':'FIND_2','aborted':'NAVTOPICKPOS','error':'error'},
        #     #                     remapping ={"pos_xm":'put_pos'})
        #     # StateMachine.add('FIND_PUT_PLACE_2',
        #     #                     GetTargetPosition(),
        #     #                     transitions={'succeeded': 'TARGET_POINT_JUSFY', 'error': 'error'})   
                                                               
        #     StateMachine.add('TARGET_POINT_JUSFY',
        #                         TargerPosReload(),
        #                         transitions={'succeeded': 'PUT_DOWN', 'error': 'error'})

        #     StateMachine.add('PUT_DOWN',
        #                         PutDown(),#控制爪子（打开）
        #                         transitions ={'succeeded':'succeeded','aborted':'PUT_DOWN','error':'error'})


        #------find the people and get the all task---------#
        self.xm_Receive_Task = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'],
            input_keys=["people_pos",'target', 'action', 'answer', 'turn'],
            output_keys=["people_pos",'target', 'action', 'answer', 'turn'])
        with self.xm_Receive_Task:
            self.xm_Receive_Task.userdata.degree = -1.3# 大约0.5是向左25度。1.5是向左75度。具体得导航确定一下
            self.xm_Receive_Task.userdata.detect2_turn =2
            self.xm_Receive_Task.userdata.start_sentences = 'I am service robot, what can I do for you'
            self.xm_Receive_Task.userdata.answer_sentences = 'The object you need is '
            #self.xm_Receive_Task.userdata.people_point=Point()

            # StateMachine.add('FIND_PEOPLE_AND_GO',
            #                 self.findPeopleAndGo,
            #                 transitions={'succeeded': 'FIND_PEOPLE_AND_GO', 'aborted': 'TURN_DEGREE', 'error': 'error'})

            StateMachine.add('FIND_PEOPLE',
                             GetPersonPosition(),
                             transitions={'succeeded': 'POSITION_TRANSFORM', 'aborted': 'TURN_DEGREE', 'error': 'error'})
            StateMachine.add('POSITION_TRANSFORM',
                             PositionTransform(),
                             transitions={'succeeded': 'GO_PEOPLE', 'error': 'error'})
            
            # StateMachine.add('TRANSFORM_CAMERA_POINT',
            #                  TransformCameraPoint(),
            #                  transitions={'succeeded': 'GO_PEOPLE', 'aborted': 'TURN_DEGREE', 'error': 'error'})
            StateMachine.add('GO_PEOPLE',
                             NavStackSpeaial(),
                             transitions={'succeeded': 'FIND_PEOPLE_AGAIN',
                                          'aborted': 'GO_PEOPLE', 'error': 'error'},
                             remapping={'pos_xm': 'people_pos'})
            StateMachine.add('TURN_DEGREE',
                             TurnDegree(),
                             transitions={'succeeded': 'FIND_PEOPLE', 'aborted': 'TURN_DEGREE', 'error': 'error'})
            # StateMachine.add('FIND_PEOPLE_AGAIN',
            #                  GetPersonPosition2(),
            #                  transitions={'succeeded': 'SPEAK_GUEST',
            #                               'aborted': 'FIND_PEOPLE_AGAIN', 'error': 'FIND_PEOPLE_AGAIN'},
            #                  remapping={'sentences': 'start_sentences'})    
            StateMachine.add('FIND_PEOPLE_AGAIN',
                             PositionJudge(),
                             transitions={'succeeded': 'SPEAK_GUEST',
                                          'aborted': 'FIND_PEOPLE_AGAIN', 'error': 'GO_PEOPLE'})    
            # StateMachine.add('POSITION_TRANSFORM2',
            #                  PositionTransform(),
            #                  transitions={'succeeded': 'SPEAK_GUEST', 'error': 'error'})             
            StateMachine.add('SPEAK_GUEST',
                             SpeakText(),
                             transitions={'succeeded': 'RECEIVE_TASKS',
                                          'aborted': 'SPEAK_GUEST', 'error': 'error'},
                             remapping={'sentences': 'start_sentences'})
            StateMachine.add('RECEIVE_TASKS',
                             GetTaskH(),
                             transitions={'succeeded': 'ANSWER',
                                          'aborted': 'RECEIVE_TASKS', 'error': 'error'}
                             )
            StateMachine.add('ANSWER',
                             SpeakAnswer1(),
                             transitions={'succeeded': 'CHECK_RECEIVE_TASKS',
                                          'aborted': 'ANSWER', 'error': 'error'},
                             remapping={'sentences': 'answer_sentences', 'object': 'target'})
            StateMachine.add('CHECK_RECEIVE_TASKS',
                             Check_Receive_Task(),
                             transitions={'succeeded': 'succeeded',
                                          'continue': 'FIND_PEOPLE', 'error': 'error'}
                             )

        #------go to another room ,find objects and give them to the people---------#
        self.xm_Give_Object = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'],
            input_keys=["people_pos",'target', 'action', 'answer', 'turn','current_turn_num'],
            output_keys=["people_pos",'target', 'action', 'answer', 'turn','current_turn_num'])
        with self.xm_Give_Object:
            self.xm_Give_Object.userdata.room_num = 2  # 该房间可以停的位置个数
            self.xm_Give_Object.userdata.room_pos = list()  # 该房间可以停的位置列表
            # self.xm_Give_Object.userdata.current_turn = 0 # 当前抓的物品的序号
            self.xm_Give_Object.userdata.current_people_pos = Pose()
            self.xm_Give_Object.userdata.current_room_num = 1 # 当前家具的编号
            self.xm_Give_Object.userdata.current_guest_num = 1 # 当前爪子抓客人的编号,从0开始
            # self.xm_Give_Object.userdata.room_pos = gpsr_target['livingroom']['pos']
            self.xm_Give_Object.userdata.arm_give_waypoints = [[0, 0.026, 3.14, 0]]

            # 依次停止的位置，如果一个位置已经被搜寻完，则换下一个位置
            # self.xm_Give_Object.userdata.search_pos1 = gpsr_target['kitchen_counter']['pos']
            # self.xm_Give_Object.userdata.search_pos2 = gpsr_target['kitchen_dishwasher']['pos']
            # self.xm_Give_Object.userdata.search_pos3 = gpsr_target['livingroom']['pos']
            # self.xm_Give_Object.userdata.search_pos4 = gpsr_target['livingroom']['pos']

            # StateMachine.add('SORT_ROOM_POS',
            #                  SortRoomPos(),
            #                  transitions={'succeeded': 'NAV',
            #                               'aborted': 'SORT_ROOM_POS', 'error': 'error'})

            StateMachine.add('NAV',
                             NavStack(),
                             transitions={'succeeded': 'PICK_UP',
                                          'aborted': 'NAV', 'error': 'error'},
                             remapping={'pos_xm': 'search_pos'+str(self.xm_Give_Object.userdata.current_room_num)})

            StateMachine.add('PICK_UP',
                             self.xm_Pick_up,
                             transitions={'succeeded': 'FIND_GUEST', 'aborted': 'PICK_UP','finished':'NAV', 'error': 'error'})

            StateMachine.add('FIND_GUEST',
                                FindGuest(),
                                transitions={'succeeded':'NAV_BACK_HALL','aborted':'FIND_GUEST','error':'error'}
                               )

            StateMachine.add('NAV_BACK_HALL',
                                NavStackSpeaial2(),#返回物品对应的客人位置
                                transitions={'succeeded':'GIVE_OBJECT','aborted':'NAV_BACK_HALL','error':'error'},
                                remapping = {'pos_xm':'current_people_pos'})
            StateMachine.add('GIVE_OBJECT',
                                ArmTrajectory(),#机械臂摆一个递给人东西的动作
                                transitions={'succeeded':'SPEAK_RELEASE','error':'error'},
                                remapping={'arm_waypoints':'arm_give_waypoints'})

            StateMachine.add('SPEAK_RELEASE',
                                SpeakText(),#提示人抓取物品
                                transitions={'succeeded':'CHECK_GIVE_OBJECT','aborted':'GIVE_OBJECT','error':'error'},
                                remapping = {'sentences':'sentences_release'})


            # 是否需要加一个等待人取走物品的状态,或是让机械臂检测?

            # StateMachine.add('PUT_DOWN',
            #                     self.xm_Put_down,
            #                     transitions={'succeeded':'NAV_BACK_HALL','aborted':'PUT_DOWN'})             
            
            # 此时应该回到物品对应的人旁边
            # StateMachine.add('NAV_BACK_HALL',
            #                  NavStack(),
            #                  transitions={'succeeded': 'GIVE_OBJECT',
            #                               'aborted': 'RECEIVE_TASKS', 'error': 'error'},
            #                  remapping={'pos_xm': 'room_pos'})

            # 每次抓取放回后作一次判断，看是否达到次数 
            StateMachine.add('CHECK_GIVE_OBJECT',
                                CheckGiveObject(),# 检测运送了几次物品
                                transitions={'succeeded':'succeeded','aborted':'NAV','error':'error'})



         # 顶层状态机
        self.xm_HandMeTHat = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'])
        with self.xm_HandMeTHat:

            self.xm_HandMeTHat.userdata.target = list()
            self.xm_HandMeTHat.userdata.action = list()
            self.xm_HandMeTHat.userdata.people_pos = list()
            self.xm_HandMeTHat.userdata.current_turn_num = 0# 已经成功运送的物品个数
            self.xm_HandMeTHat.userdata.answer = "this is the answer"
            self.xm_HandMeTHat.userdata.turn = 1
            # self.xm_HandMeTHat.userdata.pos_xm_door = gpsr_target['open_door_pose']['pos']
            self.xm_HandMeTHat.userdata.waypoint = gpsr_target['exit_pos']['pos']
            # self.xm_HandMeTHat.userdata.start_waypoint = gpsr_target['livingroom']['pos']
            # for the Speak() state
            # self.xm_HandMeTHat.userdata.sentences = 'give me the mission please'
            self.xm_HandMeTHat.userdata.gripper_release = 0.0
            self.xm_HandMeTHat.userdata.give_file = "give_me_the_mission.wav"
            self.xm_HandMeTHat.userdata.sentences_release = "i will release the gripper,please grap it"
            self.xm_HandMeTHat.userdata.rec = 5.0
            StateMachine.add('ENTERROOM',
                             self.xm_EnterRoom,
                             transitions={'succeeded': 'RECEIVE_TASKS', 'aborted': 'ENTERROOM', 'error': 'error'})
            transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'})

            # StateMachine.add('RECEIVE_TASKS',
            #                  self.xm_Receive_Task,
            #                  transitions={'succeeded': 'GIVE_OBJECT',
            #                               'aborted': 'RECEIVE_TASKS', 'error': 'error'})

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
