#!/usr/bin/env python
# encoding:utf8
import rospy
from smach_ros import IntrospectionServer
from smach_special.gpsr import *
from smach_compose.compose import *
from smach_common.common import *
from smach_special.shopping_lib import *
from xm_smach.target_gpsr import *
from smach import State, StateMachine, Concurrence, CBState
from math import pi
from geometry_msgs.msg import *
import subprocess
from new_lib.special import ShoppingNextTask, ShoppingNextTask_Test
from new_lib.basic_pick import ArmCmdForTf
from new_lib.basic_pick import ArmCmd as new_ArmCmd
from new_lib.special import PosJustfy as new_PosJustfy
from new_lib.basic_vision import FindObject as FindObject2

# from xm_smach.xm_smach.smach_lib.smach_special.shopping_lib import GetPickPosBack_shopping_putdown, GetPutPosition, Pos_Find, PresentTask, TurnDegree_Shop
from smach_special.shopping_lib import GetPickPosBack_shopping_putdown, GetPutPosition, Pos_Find, PresentTask, TurnDegree_Shop


'''
Author: yishui
Date: 2022-10-02
LastEditTime: 2022-10-02 22:42:11
LastEditors: yishui
Description: Codes for 中国机器人大赛超市项目
FilePath: ~/catkin_ws/src/xm_smach/xm_smach/tests/yishui/Shopping
'''


def get_pid(name):
    pid = ''
    with open("/home/xm/vision_pid/{}.txt".format(name)) as f:
        pid = f.read()
    return pid


class Shopping():
    def __init__(self):
        rospy.init_node('Shopping')
        self.smach_bool = False
        rospy.on_shutdown(self.shutdown)

        # how to get stop signal?
        self.trace = Concurrence(outcomes=['remeber', 'stop', 'aborted'],
                                 default_outcome='stop',
                                 outcome_map={'remeber': {'STOP': 'remeber'},
                                              'stop': {'STOP': 'stop'},
                                              'aborted': {'FOLLOW': 'aborted'}},
                                 child_termination_cb=self.trace_child_cb,
                                 input_keys=['PT_LIST', 'mission', 'task'],
                                 output_keys=['PT_LIST', 'mission', 'task'])
        with self.trace:
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
                                 Wait(),  # 位于tool中，给rec赋值，沉睡0s
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
                                    'pos_xm': 'pos_xm'})
                    Concurrence.add('WAIT', Wait_trace())
                StateMachine.add('META_NAV',
                                 self.meta_nav,
                                 transitions={'get_pos': 'FIND', 'time_over': 'FIND', 'aborted': 'FIND'})
            Concurrence.add('FOLLOW', self.meta_follow)
            Concurrence.add('STOP', CheckStop2(),
                            remapping={'PT_LIST': 'PT_LIST', 'mission': 'mission'})

        #-------Put_down---------#
        self.put_down = StateMachine(outcomes=['succeeded', 'aborted', 'error'])
        with self.put_down:
            self.put_down.userdata.commond = 0
            # self.put_down.userdata.put_down_pos = gpsr_target['put_down']['pos']
            # self.put_down.userdata.put_pose = [[0, -1.5, 3.14, 0]]
            self.put_down.userdata.name = ''
            self.put_down.userdata.distance = 0.5  # change this element from 0.7 to 0.2
            self.put_down.userdata.table_depth = 0.04
            self.put_down.userdata.traget_size = [0.04, 0.065, 0.105]
            # self.put_down.userdata.pick_pose = [[0, -1.5, 3.14, 0]]
            # self.put_down.userdata.take_back_pose = [[0, 0.026, 3.14, 0]]
            self.put_down.userdata.mode_1 = 1
            # self.put_down.userdata.object_pos = PointStamped()
            self.put_down.userdata.target = 'basket'

            StateMachine.add('FIND_PUT_PLACE_1',
                             GetPutPosition(),
                             transitions={'succeeded': 'GETPUTPOSE', 'aborted': 'FIND_PUT_PLACE_1', 'error': 'error'},)

            StateMachine.add('GETPUTPOSE',
                             GetPickPosBack_shopping_putdown(),
                             transitions={'succeeded': 'PUT_DOWN', 'error': 'error'})

            # StateMachine.add('NAVTOPICKPOS1',
            #                  NavStack(),
            #                  transitions={
            #                      'succeeded': 'PUT_DOWN', 'aborted': 'NAVTOPICKPOS1', 'error': 'error'},
            #                  remapping={"pos_xm": 'pick_pos'})

            # StateMachine.add('TARGET_POINT_JUSFY',
            #                  TargerPosReload(),
            #                  transitions={'succeeded': 'PUT_DOWN', 'error': 'error'})

            StateMachine.add('PUT_DOWN',
                             GripperCommond(),  # 控制爪子（打开）
                             transitions={'succeeded': 'succeeded', 'error': 'error'})

        self.GetTask = StateMachine(outcomes=['succeeded', 'aborted', 'error'],
                                    output_keys=[
                                        'task', 'task_pisition', 'current_turn',],
                                    input_keys=['task_pisition', 'PT_LIST', 'task', 'current_turn','order'])
        with self.GetTask:
            self.GetTask.userdata.num_list = list()
            self.GetTask.userdata.person_position = Pose()
            self.GetTask.userdata.pose = Pose()
            self.GetTask.userdata.find_sentences = "I found the "
            self.GetTask.userdata.name = ''
            self.GetTask.userdata.distance = 0.5  # change this element from 0.7 to 0.2
            self.GetTask.userdata.pick_pose = [[0, -1.5, 3.14, 0]]
            self.GetTask.userdata.present_pos = Pose()
            self.GetTask.userdata.present_task = ""
            self.GetTask.userdata.pick_sentences = ""
            self.GetTask.userdata.catch = "i catch that"
            self.GetTask.userdata.help_sentences = 'Please help me pick up my things'
            self.GetTask.userdata.commond_open=0
            self.GetTask.userdata.commond_close=1

            StateMachine.add('OPEN_GRIPPER',
                             GripperCommond(),  # 控制爪子（打开）
                             transitions={'succeeded': 'OBJECT_DETECT', 'error': 'error'},
                             remapping={'commond':'commond_open'})
            StateMachine.add("OBJECT_DETECT", 
                             TurnDegree_Shop(),
                             transitions={'succeeded': 'PRESENT_TASK', 'aborted': 'OBJECT_DETECT', 'error': 'error'})
            StateMachine.add("PRESENT_TASK", 
                             PresentTask(),
                             transitions={'succeeded': 'SPEAK_OBJECT', 'error': 'error'})

            StateMachine.add("SPEAK_OBJECT", 
                             SpeakText(),
                             transitions={
                                 'succeeded': 'NAVTOPICKPOS', 'aborted': 'SPEAK_OBJECT', 'error': 'error'},
                             remapping={'sentences': 'pick_sentences'})

            StateMachine.add('NAVTOPICKPOS',
                             SpeakText(),
                             transitions={'succeeded': 'CLOSE_GRIPPER',
                                          'aborted': 'NAVTOPICKPOS', 'error': 'error'},
                             remapping={"sentences": 'help_sentences'})
            StateMachine.add('CLOSE_GRIPPER',
                             GripperCommond(),  
                             transitions={'succeeded': 'POS_FIND', 'error': 'error'},
                             remapping={'commond':'commond_close'})
            StateMachine.add("POS_FIND", 
                             Pos_Find(),
                             transitions={'succeeded': 'NAV', 'error': 'error'})
            StateMachine.add("NAV", 
                             NavStack(),
                             transitions={'succeeded': 'succeeded',
                                          'aborted': 'NAV', 'error': 'error'},
                             remapping={'pos_xm': 'present_pos'})


        #主
        self.shopping = StateMachine(outcomes=['succeeded', 'aborted', 'error'])
        with self.shopping:

            self.shopping.userdata.PT_LIST = {}  # dict {object:Pose()}
            # checkStop2后，mission1的内容与PT_LIST一致
            self.shopping.userdata.mission1 = {}
            # 需要捡起的物品，按照引导顺序排列
            self.shopping.userdata.task = []
            # 转向的顺序，123，左边是1，中间是2，右边是3
            self.shopping.userdata.order=['left','med','right']
            
            self.shopping.userdata.rec = 5.
            # 需要捡起的物品数量
            self.shopping.userdata.turn = 3
            self.shopping.userdata.current_turn = 0
            self.shopping.userdata.start_follow = 'Now I will follow you.'
            # 存储物品的位置，每次只存最近一次识别的物品位置
            self.shopping.userdata.task_pisition = Pose()
            self.shopping.userdata.cashier_pos = Pose()
            # 当前任务的下标数
            self.shopping.userdata.target = 0
            self.shopping.userdata.done = 'object had put down'
            self.shopping.userdata.go_shopping = 'i will go shopping'
            # StateMachine.add('ENTERROOM',
            #                     self.xm_EnterRoom,
            #                     transitions={'succeeded':'START','aborted':'aborted'})
            # 说follow_me
            # StateMachine.add('START',
            #                  GetSignal(),
            #                  transitions={'succeeded': 'RUNNODE', 'aborted': 'START'})
            #开摄像头
            StateMachine.add('RUNNODE',
                             RunNode(),
                             transitions={'succeeded': 'SPEAK_BEGIN_FOLLOW', 'aborted': 'RUNNODE'})
            # transitions={'succeeded':'TRACE','aborted':'RUNNODE'})
            #speak:'Now I will follow you to.'
            StateMachine.add('SPEAK_BEGIN_FOLLOW',
                             SpeakText(),
                             transitions={'succeeded': 'TRACE',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping={'sentences': 'start_follow'}
                             )
            #跟随人,获取3个物品信息，位置
            StateMachine.add('TRACE',
                             self.trace,
                             transitions={
                                 'remeber': 'TRACE', 'stop': 'SPEAK_GO', 'aborted': 'aborted'},
                             remapping={'PT_LIST': 'PT_LIST', 'mission': 'mission1'})
            # speak：'i will go shopping'
            StateMachine.add('SPEAK_GO',
                             SpeakText(),
                             transitions={'succeeded': 'GET_TASK',
                                          'aborted': 'SPEAK_GO', 'error': 'error'},
                             remapping={'sentences': 'go_shopping'}
                             )
            # 写一个提示引导者离开的状态
            StateMachine.add('GET_TASK',
                             self.GetTask,
                             transitions={'succeeded': 'PUT_DOWN',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping={'task': 'task', 'PT_LIST': 'PT_LIST'})
            StateMachine.add('PUT_DOWN',
                             self.put_down,
                             transitions={'succeeded': 'CHECK_TURN',
                                          'aborted': 'aborted', 'error': 'error'})
            # StateMachine.add('PUT_DOWN',
            #                  SpeakText(),
            #                  transitions={'succeeded': 'CHECK_TURN',
            #                               'aborted': 'aborted', 'error': 'error'},
            #  remapping={'sentences': 'done'})
            StateMachine.add('CHECK_TURN',
                             CheckTurn(),
                             transitions={'succeeded': 'succeeded',
                                          'continue': 'NAV_BACK', 'error': 'error'})
            StateMachine.add('NAV_BACK',
                             NavStack(),
                             transitions={'succeeded': 'GET_TASK',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping={'pos_xm': 'cashier_pos'})
        intro_server = IntrospectionServer('shopping', self.shopping, 'SM_ROOT')
        intro_server.start()
        out = self.shopping.execute()
        intro_server.stop()
        if out == 'succeeded':
            self.smach_bool = True

    def shutdown(self):
        if self.smach_bool == False:
            rospy.logwarn('smach execute failed')
        else:
            rospy.logwarn('smach execute successfully')

    def trace_child_cb(self, outcome_map):
        if outcome_map['STOP'] == 'stop':
            rospy.logwarn('get the stop signal, stop tracing ........')
            pid = get_pid("people_tracking")
            print('kill pid -> {}'.format(pid))
            print('kill pid -> {}'.format(pid))
            print('kill pid -> {}'.format(pid))
            print('kill pid -> {}'.format(pid))
            subprocess.Popen(['kill', '-9', pid], shell=False)
            # with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
            #     f.write('')
            print("sleepiiiiiiiiing!!!!!!!!!!!")
            return True
        elif outcome_map['STOP'] == 'remeber':
            rospy.logwarn('ready to remeber the position')
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

    def PickDistance(self, userdata):
        try:
            if (userdata.name == 'ice_tea' or userdata.name == 'water' or userdata.name == 'tea'):
                userdata.distance = 0.85
            elif userdata.name == 'milk' or userdata.name == 'redbull':
                userdata.distance = 0.8

            else:
                userdata.distance = 0.85
            return 'succeeded'

        except Exception, e:
            rospy.logerr(e)
            return 'error'

    def Arm_psJus(self, userdata):
        try:
            if userdata.name == 'ice_tea':
                userdata.pick_pos.point.x += -0.05
                userdata.pick_pos.point.y += 0
                userdata.pick_pos.point.z += 0.04
            elif userdata.name == 'tea':
                userdata.pick_pos.point.x += -0.05
                userdata.pick_pos.point.y += -0.02
                userdata.pick_pos.point.z += 0.04
            elif userdata.name == 'cola' or userdata.name == 'herbal_tea' or userdata.name == 'fanta' or userdata.name == 'porridge' or userdata.name == 'redbull':
                userdata.pick_pos.point.z -= 0.05
            elif userdata.name == 'grape_juice' or userdata.name == 'milk_tea' or userdata.name == 'water':
                userdata.pick_pos.point.z += 0.04
            rospy.logerr(userdata.pick_pos)

            return 'succeeded'

        except Exception, e:
            rospy.logerr(e)
            return 'error'

    def ahead_justice(self, userdata):
        try:
            self.cmd_vel = rospy.Publisher(
                '/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)
            angular_speed = 0.1
            self.turn = Twist()
            self.turn.linear.x = 0.1
            self.turn.linear.y = 0.0
            self.turn.linear.z = 0.0
            self.turn.angular.x = 0.0
            self.turn.angular.y = 0.0
            self.turn.angular.z = 0.0

            goal_angle = 0.1
            angular_duration = goal_angle/angular_speed

            rate = 50
            r = rospy.Rate(rate)
            ticks = int(goal_angle*rate)+5
            for i in range(ticks):
                self.cmd_vel.publish(self.turn)
                r.sleep()

            return 'succeeded'

        except Exception, e:
            rospy.logerr(e)
            return 'aborted'


if __name__ == "__main__":
    try:
        Shopping()

    except Exception, e:
        rospy.logerr(e)
