#!/usr/bin/env python
# encoding:utf8
import rospy
from smach_ros import IntrospectionServer
from smach import StateMachine
from geometry_msgs.msg import *
from smach_special.HandMeThat_lib3 import *
from smach_special.shopping_lib import *
from smach_common.common import *

'''
Author: ming
Date: 2023-7-11
LastEditTime: 2023-7-11
LastEditors: ming
Description: Codes for 中国机器人大赛语音全部测试
FilePath: ~/catkin_ws/src/xm_smach/xm_smach/tests/ming
'''

class Speech_texting():
    def __init__(self):
        rospy.init_node('Speech_texting')
        self.smach_bool = False
        rospy.on_shutdown(self.shutdown)

        #顶层
        self.speech_texting = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'])
        with self.speech_texting:
            # 需要取的物品
            self.speech_texting.userdata.target = list()
            self.speech_texting.userdata.action = list()
            self.speech_texting.userdata.people_name = list()

            self.speech_texting.userdata.command_one = 'Now I will text the frist command shopping.'
            self.speech_texting.userdata.command_five = 'Now I will text the fifth command hand me that.'
            self.speech_texting.userdata.answer_sentences = 'The object you need is '
            
            # StateMachine.add('COMMAND_ONE',
            #                  SpeakText(),
            #                  transitions={'succeeded': 'START', 'aborted': 'aborted', 'error': 'error'},
            #                  remapping={'sentences': 'command_one'}
            #                  )
            # #测试shopping                 
            # StateMachine.add('START',
            #                  GetSignal(),
            #                  transitions={'succeeded': 'COMMAND_FIVE', 'aborted': 'START'}
            #                  )
            StateMachine.add('COMMAND_FIVE',
                             SpeakText(),
                             transitions={'succeeded': 'RECEIVE_TASKS',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping={'sentences': 'command_five'}
                             )
            #测试hand me that                 
            StateMachine.add('RECEIVE_TASKS',
                             GetTaskH(),
                             transitions={'succeeded': 'SPEAK_OBJECT',
                                          'aborted': 'RECEIVE_TASKS', 'error': 'error'}
                             )
            StateMachine.add('SPEAK_OBJECT',
                             SpeakAnswer1(),
                             transitions={'succeeded': 'COMMAND_FIVE',
                                          'aborted': 'SPEAK_OBJECT', 'error': 'error'},
                             remapping={'sentences': 'answer_sentences', 'object': 'target'})

        intro_server = IntrospectionServer('speech_texting', self.speech_texting, 'SM_ROOT')
        intro_server.start()
        out = self.speech_texting.execute()
        intro_server.stop()
        if out == 'succeeded':
            self.smach_bool = True

    def shutdown(self):
        if self.smach_bool == False:
            rospy.logwarn('smach execute failed')
        else:
            rospy.logwarn('smach execute successfully')

if __name__ == "__main__":
    try:
        Speech_texting()

    except Exception, e:
        rospy.logerr(e)