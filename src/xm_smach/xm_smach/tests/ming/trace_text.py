#!/usr/bin/env python
# encoding:utf8
import rospy
from smach_ros import IntrospectionServer
from smach import StateMachine
from geometry_msgs.msg import *
from smach_special.HandMeThat_lib3 import *
from smach_special.shopping_lib import *
from smach_common.common import *
from smach_compose.compose import *

'''
Author: ming
Date: 2023-7-16
LastEditTime: 2023-7-16
LastEditors: ming
Description: Codes for 中国机器人大赛跟随测试
FilePath: ~/catkin_ws/src/xm_smach/xm_smach/tests/ming
'''

class Trace_texting():
    def __init__(self):
        rospy.init_node('Trace_texting')
        self.smach_bool = False
        rospy.on_shutdown(self.shutdown)

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

        self.trace_texting = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'])
        with self.trace_texting:
            
            self.trace_texting.userdata.start_follow = 'Now I will follow you'
            self.trace_texting.userdata.begin = 'Now begin'

            StateMachine.add('SPEAK',
                             SpeakText(),
                             transitions={'succeeded': 'RUNNODE',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping={'sentences': 'begin'}
                             )
            #开摄像头
            StateMachine.add('RUNNODE',
                             RunNode(),
                             transitions={'succeeded': 'SPEAK_BEGIN_FOLLOW', 'aborted': 'RUNNODE'})
            # transitions={'succeeded':'TRACE','aborted':'RUNNODE'})
            #speak:'Now I will follow you.'
            StateMachine.add('SPEAK_BEGIN_FOLLOW',
                             SpeakText(),
                             transitions={'succeeded': 'TRACE',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping={'sentences': 'start_follow'}
                             )
            StateMachine.add('TRACE',
                             self.trace,
                             transitions={
                                 'remeber': 'TRACE', 'stop': 'succeeded', 'aborted': 'aborted'},
                             remapping={'PT_LIST': 'PT_LIST', 'mission': 'mission1'})

        intro_server = IntrospectionServer('trace_texting', self.trace_texting, 'SM_ROOT')
        intro_server.start()
        out = self.trace_texting.execute()
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

if __name__ == "__main__":
    try:
        Trace_texting()

    except Exception, e:
        rospy.logerr(e)