#!/usr/bin/env python
# encoding:utf8
from pyparsing import CaselessKeyword
from xm_smach.target_gpsr import gpsr_target
import rospy
import tf
import actionlib
from math import *
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatus 
from time import sleep
from std_msgs.msg import String,Int32,Bool,Header
from std_srvs.srv import *
from xm_msgs.srv import *
from xm_msgs.msg import *
import subprocess
import math  
from speech.srv import *

# simple state used for get meaning from the speech node
# command mean invoke the speech node to return the meaning from the order
# 用于从对话中得到含义的简单状态
# 命令行是启动speech node来返回命令的含义
# 此处gesture仅为占位，无实际用处
class GetTask(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=['succeeded','aborted','error'],
                        io_keys=['target','action','task_num','answer','gesture'])
                        
        # self.speech_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        #rospy.wait_for_service('speech_core')
        self.speech_client = rospy.ServiceProxy('speech_core', speech_to_smach)
                
    def execute(self,userdata):
        try:
            getattr(userdata, 'target')
            getattr(userdata, 'action')            
            getattr(userdata, 'task_num')
            getattr(userdata, 'answer')
            getattr(userdata, 'gesture')
        except:
            rospy.logerr('No param specified')
            return 'error'
        try:
            self.speech_client.wait_for_service(timeout=10)
        #command = 1用于gpsr
            response =self.speech_client.call(command = 1,text = 'www')
            print(response.num)
            print(response.action)
            print(response.object)
            print(response.gesture)
        except:
            rospy.logerr('wrong in call the service')
            return 'error'
        # if response.action[0] == 'stop':
        #     rospy.logerr('response wrong!')
        #     return 'aborted'

        # num任务个数
        # action动作序列go find get
        # target目标对象 drinks
        # answer回复的语音文本 一句话
        # gesture （机械臂）姿态 rasie their right arm 等
        if response.num > 0:
            userdata.task_num = response.num
            userdata.action = response.action
            userdata.target = response.object
            userdata.answer = response.answer
            userdata.gesture = response.gesture
            rospy.logwarn(userdata.task_num)
            rospy.logwarn(userdata.action)
            rospy.logwarn(userdata.target)
            rospy.logwarn(userdata.answer)
            rospy.logwarn(userdata.gesture)
            # if userdata.action[1] == 'grasp':
                # something interest here
                # userdata.action[0] = 'nav_grasp'
                # userdata.action.append('place')
                # userdata.task_num += 1
            return 'succeeded'
        else:
            return 'aborted'



class NextDo(State):###跳转action
    def __init__(self):
        State.__init__(self, 
                        #nav_grasp is something interesting
                    outcomes=['succeeded','aborted','go','follow','get','release','error'],
                    input_keys =['action','task_num'],
                    io_keys =['current_task'])
        
    def execute(self, userdata):
        try:
            action = userdata.action
            current_task = userdata.current_task
            task_num = userdata.task_num
        except:
            rospy.logerr('No param specified')
            return 'error'
        userdata.current_task+=1
            
        current_action =  action[0]

        #-------test for find---------#
        # if current_action != 'find':
        #     return 'aborted'
        # else: return 'find'

        if current_action == 'go':
            return 'go'
        elif current_action == 'follow':
            return 'follow'
        elif current_action == 'pick' or current_action == 'get' or current_action == 'take':
            return 'get'
        elif current_action == 'deliver' or current_action=='release':
            return 'release'
        else:
            # no avaiable action find
            # userdata.current_task_out -1
            userdata.current_task -=1
            return 'aborted'
  
