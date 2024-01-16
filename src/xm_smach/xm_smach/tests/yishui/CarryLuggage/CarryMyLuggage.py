#!/usr/bin/env python
# encoding:utf8

from os import abort
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_special.CarryMyLuggage_lib import *
from smach_special.gpsr import GetTarget, Wait_trace, RunNode, FindPeople, CloseCamera
from smach_special.gpsr import PersonOrPosition, GetPos, FindObject
from smach_compose.compose import *
from smach_common.common import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

'''
Author: yishui
Date: 2022-08-25
LastEditTime: 2022-08-25 15:30:47
LastEditors: yishui
Description: Codes for CarryLuggage
FilePath: ~/catkin_ws/src/xm_smach/xm_smach/tests/yishui/CarryLuggage
'''

'''
导航到客厅中预定开始地点->
获取要拿起的袋子->
抓取->
找到并面对操作员->
交互告诉操作员开始跟随->
跟随操作员（记忆）->
将袋子递交操作员->
等待感谢->
返回竞机场->
获取终止信号->
丢失叫操作员
'''


class Talk(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'])
        self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)

    def execute(self, userdata):
        try:
            getattr(userdata, 'sentences')
        except:
            rospy.logerr('No param specified ')
            return 'error'
        else:

            rospy.loginfo('.................Speak ^_^..........\n')
            rospy.logwarn(userdata.sentences)
            try:
                res = self.speak_client.call(command=2)
                self.string_ = res.target[0]

            except:
                rospy.logerr('get wrong when transfrom the sentences')
                return 'error'
            else:
                try:
                    # subprocess - Subprocesses with accessible I/O streams
                    # 创建一个子进程,父进程等待子进程完成,返回退出信息
                    # 让xm在子进程中说出userdata.sentences的语句，不影响父进程
                    self.speak_client.wait_for_service(timeout=10.0)
                    self.speak_client.call(self.string_)

                    rospy.sleep(2.0)

                    speech_bool = self.speak_client.call(self.string_)
                    if speech_bool.flag == 1:
                        subprocess.call(["play", "tts_sample.wav"])
                    elif speech_bool.flag == 0:
                        subprocess.call("espeak -vf5 -s 75 '%(a)s'" %
                                        {'a': str(self.string_)}, shell=True)
                    else:
                        rospy.logerr('the response error')
                        return 'error'
                except:
                    return 'aborted'
                else:
                    return 'succeeded'


class Carry():
    def __init__(self):
        rospy.init_node("Gpsr_Smach")
        rospy.on_shutdown(self.shutdown)
        rospy.logerr("Welcome to Carry!")
        self.smach_bool = False

        #--------GO--------#
        self.xm_Nav = StateMachine(outcomes=['succeeded', 'aborted', 'error'],
                                   input_keys=['target', 'current_task'])
        with self.xm_Nav:
            self.xm_Nav.userdata.pose_xm = Pose()
            self.xm_Nav.userdata.turn_pose = Pose()
            self.xm_Nav.userdata.target_mode = 1

            StateMachine.add('GETTARGET',
                             GetTarget(),  # 修改(获取)userdata.current_target
                             transitions={'succeeded': 'GO',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping={'target': 'target', 'current_task': 'current_task',
                                        'current_target': 'pos_xm', 'mode': 'target_mode'}  # 相当于将current_target赋值给Pos_xm
                             )
            StateMachine.add('GO',
                             NavStack(),  # 以pos_xm为目标移动
                             transitions={'succeeded': 'SPEAK',
                                          'aborted': 'GO', 'error': 'error'},
                             remapping={'pos_xm': 'pos_xm'}
                             )

            self.xm_Nav.userdata.sentences = 'I have arrived here'

            StateMachine.add('SPEAK',
                             Speak(),  # 使用语音的服务，开一个子进程，播放音频
                             transitions={
                                 'succeeded': 'succeeded', 'error': 'error'},
                             remapping={'sentences': 'sentences'}
                             )
        #----FOLLOW-----#
        # xm_Follow包括两个并发状态，FOLLOW和STOP
        # STOP用于语音获取stop信号
        # FOLLOW包装了FindPeople().find_people_监视器和meta_nav
        # 当找到人需要移动时，状态转移到meta_nav
        # meta_nav后,再继续进入监视状态
        # meta_nav是一个并发状态机，包装了NAV和WAIT
        # WAIT用于记录时间引发time_over
        # nav成功或中止后，与time_over一起进入继续找人状态

        self.xm_Follow = Concurrence(outcomes=['succeeded', 'aborted'],
                                     default_outcome='aborted',
                                     outcome_map={'succeeded': {'STOP': 'stop'},
                                                  'aborted': {'FOLLOW': 'aborted'},
                                                  'aborted': {'FOLLOW': 'preempted'}},
                                     child_termination_cb=self.child_cb)
        with self.xm_Follow:

            self.meta_follow = StateMachine(
                ['succeeded', 'aborted', 'preempted'])

            with self.meta_follow:

                self.meta_follow.userdata.pos_xm = Pose()
                self.meta_follow.userdata.rec = 0.2

                StateMachine.add('FIND',
                                 LegTracker0().tracker,  # 摄像头给订阅的话题发消息来跟踪人，在follow中
                                 transitions={
                                     'invalid': 'WAIT', 'valid': 'FIND', 'preempted': 'preempted'},

                                 remapping={'pos_xm': 'pos_xm'})
                StateMachine.add('WAIT',
                                 Wait(),  # 位于tool中，给rec赋值，沉睡0.2s
                                 transitions={'succeeded': 'META_NAV', 'error': 'META_NAV'})

                self.meta_nav = Concurrence(outcomes=['time_over', 'get_pos', 'aborted'],
                                            default_outcome='aborted',
                                            outcome_map={'time_over': {'WAIT': 'succeeded'},
                                                         'get_pos': {'NAV': 'succeeded'},
                                                         'aborted': {'NAV': 'aborted'}},
                                            child_termination_cb=self.nav_child_cb,
                                            input_keys=['pos_xm'])
                with self.meta_nav:
                    Concurrence.add('NAV', NavStack0(), remapping={
                                    'pos_xm': 'pos_xm'})  # 跟随人
                    Concurrence.add('WAIT', Wait_trace())  # 以一定频率发移动指令
                StateMachine.add('META_NAV',
                                 self.meta_nav,
                                 transitions={'get_pos': 'FIND', 'time_over': 'FIND', 'aborted': 'FIND'})
            Concurrence.add('FOLLOW', self.meta_follow)
            Concurrence.add('STOP', CheckStop())
            # Concurrence.add('RUNNODE',RunNode())#开启跟随人的图像节点

        #-------PICK_UP-------#
        # RUNNODE_IMG-->GETNAME-->GET_POS-->NAV-->FIND-->POS_JUS-->NAV-->RUNNODE_IMG-->FIND-->POS_JUS-->PICK-->SPEAK
        # 运行图像节点，获取物体名字和大物体位置后，nav到大物体位置，然后用图像找物体，然后用pos_jus得出适宜抓取的坐标，nav后继续循环一次
        # 在此之后，进行抓取
        # 如果上面的图像没有找到物体，则通过speak进行错误反馈
        self.xm_Pick_up = StateMachine(outcomes=['succeeded', 'aborted', 'error'],
                                       input_keys=['target', 'current_task'])
        with self.xm_Pick_up:
            self.xm_Pick_up.userdata.name = ''
            self.xm_Pick_up.userdata.target_mode = 0
            self.xm_Pick_up.userdata.objmode = -1
            self.xm_Pick_up.userdata.go_counter = 2
            # self.xm_Pick_up.userdata.distance = 0.5+9 #change this element from 0.7 to 0.2
            self.xm_Pick_up.userdata.distance = 0.5
            self.xm_Pick_up.userdata.table_depth = 0.04
            self.xm_Pick_up.userdata.traget_size = [0.04, 0.065, 0.105]
            self.xm_Pick_up.userdata.pick_pose = [[0, -1.5, 3.14, 0]]
            self.xm_Pick_up.userdata.take_back_pose = [[0, 0.026, 3.14, 0]]
            self.xm_Pick_up.userdata.mode_1 = 1
            self.xm_Pick_up.userdata.object_pos = PointStamped()

            # StateMachine.add('PICK_POS',
            #                     ArmTrajectory(),#轨迹分析
            #                     transitions={'succeeded': 'FIND_1', 'error': 'error'},
            #                     remapping={'arm_waypoints':'pick_pose'})

            StateMachine.add('FIND_1',
                             GetObjectPosition(),  # 区分物体，识别物体
                             transitions={'succeeded': 'GETPICKPOSE', 'error': 'error'})

            StateMachine.add('GETPICKPOSE',
                             GetPickPos(),
                             transitions={'succeeded': 'NAVTOPICKPOS', 'error': 'error'})

            StateMachine.add('NAVTOPICKPOS',
                             NavStack(),
                             transitions={
                                 'succeeded': 'FIND_2', 'aborted': 'NAVTOPICKPOS', 'error': 'error'},
                             remapping={"pos_xm": 'pick_pos'})

            StateMachine.add('FIND_2',
                             GetObjectPosition(),
                             transitions={'succeeded': 'TARGET_POINT_JUSFY', 'error': 'error'})

            StateMachine.add('TARGET_POINT_JUSFY',
                             TargerPosReload(),
                             transitions={'succeeded': 'PICK', 'error': 'error'})

            StateMachine.add('PICK',
                             ArmStack(),
                             transitions={'succeeded': 'NAV_BACK', 'error': 'error'})
            self.xm_Pick_up.userdata.distance = -0.2
            StateMachine.add('NAV_BACK',
                             GoAhead(),
                             transitions={
                                 'succeeded': 'succeeded', 'error': 'error'},
                             remapping={'move_len': 'distance'})

            # TODO arm back pose
            # StateMachine.add('BACK_POS',
            #                    ArmTrajectory(),
            #                    transitions={'succeeded': 'succeeded', 'error': 'error'},
            #                    remapping={'arm_waypoints':'take_back_pose'})

        #-------Put_down---------#

        self.xm_Put_down = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'])
        with self.xm_Put_down:
            self.xm_Put_down.userdata.commond = 0
            self.xm_Put_down.userdata.arm_waypoints=[[0.2,0,0.5,0]]
            StateMachine.add('ARM_MOVE',
                             ArmTrajectory(),  # 控制爪子（打开）
                             transitions={'succeeded': 'PLACE','error': 'error'})
            StateMachine.add('PLACE',
                             GripperCommond(),  # 控制爪子（打开）
                             transitions={'succeeded': 'succeeded','error': 'error'})

        # 顶层状态机
        self.xm_Carry = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'])
        with self.xm_Carry:

            self.xm_Carry.userdata.target = list()
            self.xm_Carry.userdata.action = list()
            self.xm_Carry.userdata.task_num = 0
            self.xm_Carry.userdata.answer = "this is the answer"
            self.xm_Carry.userdata.current_task = -1
            self.xm_Carry.userdata.current_turn = -1
            # xm开始的预定位置
            self.xm_Carry.userdata.start_pos = gpsr_target['open_door_pose']['pos']
            self.xm_Carry.userdata.sentences = 'which luggage you want'  # for the Speak() state
            # for the Speak() state
            self.xm_Carry.userdata.start_follow = 'Now I will follow you'
            self.xm_Carry.userdata.finish = 'I have finish the task'
            # 'Please take your luggage out of my claw'
            self.xm_Carry.userdata.gesture = ''
            # 1为爪子合上，0为爪子张开
            self.xm_Carry.userdata.gripper_close = 1
            self.xm_Carry.userdata.gripper_open = 0
            # CarryMyLuggage_lib的GetTask()方法
            StateMachine.add('RECEIVE_TASKS',
                             GetTask(),
                             transitions={'succeeded': 'GET_NEXT_TASK',
                                          'aborted': 'RECEIVE_TASKS', 'error': 'error'},
                             remapping={'target': 'target', 'action': 'action',
                                        'task_num': 'task_num', 'answer': 'answer'})

            StateMachine.add('GET_NEXT_TASK',
                             NextDo(),
                             transitions={'succeeded': 'BACK_DOOR',
                                          'aborted': 'aborted',
                                          'error': 'error',
                                          'go': 'GO',
                                          'follow': 'CLOSE_GRIPPER',  # open the camera first
                                          'get': 'PICK_UP',
                                          'release': 'PUT_DOWN'},
                             remapping={'action': 'action',
                                        'current_task': 'current_task',
                                        'task_num': 'task_num'}
                             )
            StateMachine.add('BACK_DOOR',
                             NavStack(),
                             transitions={'succeeded': 'succeeded',
                                          'aborted': 'BACK_DOOR', 'error': 'error'},
                             remapping={'pos_xm': 'start_pos'}
                             )

            StateMachine.add('GO_OUT',
                             NavStack(),
                             transitions={'succeeded': 'succeeded',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'waypoint'}
                             )

            # add all the task smach
            StateMachine.add('GO',
                             self.xm_Nav,
                             remapping={'target': 'target',
                                        'current_task': 'current_task'},
                             transitions={'succeeded': 'RECEIVE_TASKS', 'aborted': 'GO'})
            StateMachine.add('CLOSE_GRIPPER',
                             GripperCommond(),
                             transitions={'succeeded': 'SPEAK_START',
                                          'error': 'CLOSE_GRIPPER'},
                             remapping={'commond': 'gripper_close'})
            StateMachine.add('SPEAK_START',  # 念一句话
                             Speak(),
                             transitions={'succeeded': 'RUNNODE',
                                          'aborted': 'SPEAK_START'},
                             remapping={'sentences': 'start_follow'})
            StateMachine.add('RUNNODE',
                             RunNode(),
                             transitions={'succeeded': 'FOLLOW', 'aborted': 'RUNNODE'})
            StateMachine.add('FOLLOW',
                             self.xm_Follow,
                             transitions={'succeeded': 'SPEAK_FINISH', 'aborted': 'FOLLOW'})
            StateMachine.add('SPEAK_FINISH',  # 念一句话
                             Speak(),
                             transitions={'succeeded': 'RECEIVE_TASKS',
                                          'aborted': 'SPEAK_FINISH'},
                             remapping={'sentences': 'finish'})
            StateMachine.add('PICK_UP',
                             self.xm_Pick_up,
                             remapping={'target': 'target',
                                        'current_task': 'current_task'},
                             transitions={'succeeded':'RECEIVE_TASKS','aborted': 'PICK_UP'})
            StateMachine.add('PUT_DOWN',
                             self.xm_Put_down,
                             transitions={'succeeded': 'RELEASE_NAV', 'aborted': 'PUT_DOWN'})
            StateMachine.add('RELEASE_NAV',
                             NavStack(),
                             transitions={'succeeded': 'succeeded',
                                          'aborted': 'RELEASE_NAV', 'error': 'RELEASE_NAV'},
                             remapping={'pos_xm': 'start_pos'})
            # StateMachine.add('SPEAK_RELEASE',
            #                  Speak(),
            #                  transitions={
            #                      'succeeded': 'WAIT_RELEASE', 'error': 'error'},
            #                  remapping={'sentences': 'sentences_release'})
            # StateMachine.add('WAIT_RELEASE',
            #                  Wait(),
            #                  transitions={'succeeded': 'RELEASE', 'error': 'RELEASE'})

        intro_server = IntrospectionServer(
            'xm_Carry', self.xm_Carry, '/XM_ROOT')
        intro_server.start()
        out = self.xm_Carry.execute()
        intro_server.stop()

    def shutdown(self):
        if self.smach_bool == True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

    # return True down
    def child_cb(self, outcome_map):
        if outcome_map['STOP'] == 'stop':
            rospy.logwarn(
                '---------get the signal of stop,stop tracing---------')
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill', '-9', pid], shell=True)
            return True
        elif outcome_map['STOP'] == 'aborted':
            rospy.logerr('the stop state meet error!')
            return True

        if outcome_map['FOLLOW']:
            rospy.logerr('the follow state meet error!')
            return True
        return False

    def nav_child_cb(self, outcome_map):
        if outcome_map['WAIT'] == 'succeeded':
            rospy.logwarn('get the pos again')
            return True
        elif outcome_map['NAV'] == 'succeeded':
            return True
        elif outcome_map['NAV'] == 'aborted':
            return True
        else:
            print(outcome_map)
            return False

    # use for concurrence
if __name__ == "__main__":
    try:
        Carry()
    except:
        rospy.logerr(e)
