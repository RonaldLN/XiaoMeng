#! /usr/bin/env python
#encoding:utf8


import rospy
from smach import State, UserData, StateMachine
from smach_ros import SimpleActionState, ServiceState, MonitorState
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
from time import sleep
from math import *
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import subprocess
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from std_msgs.msg import Bool, Header
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import os
from rospy import ROSException


class SpeakSentence(State):
    def __init__(self):
        State.__init__(self , outcomes = ['succeeded' , 'aborted' , 'error'],
                            input_keys = ['sentences'])

        self.speak_client = rospy.ServiceProxy('tts' , xm_Speech_tts)

    
    def execute(self , userdata):

        try:
            getattr(userdata , 'sentences')

        except Exception , e:
            rospy.logerr('Bad sentences')
            rospy.logerr(e)
            return 'error'

        
        try:
            self.speak_client.wait_for_service( timeout = 10)
            self.sentences = str(userdata.sentences)

            rospy.logwarn('-----------xm will speak-----------')
            rospy.logwarn(self.sentences)

            self.flag = self.speak_client.call(self.sentences)

            rospy.sleep(2.0)

            speech_bool = self.speak_client.call(self.sentences)
            if speech_bool.flag == 1:
                subprocess.call(["play", "tts_sample.wav"])
            elif speech_bool.flag == 0:
                subprocess.call("espeak -vf5 -s 75 '%(a)s'" %
                                {'a': str(self.sentences)}, shell=True)
            else:
                rospy.logerr('the response error')
                return 'error'

        except ROSException,e:
            rospy.logerr(e)
            subprocess.call("espeak -vf5 -s 75 '%(a)s'" %
                                {'a': str(self.sentences)}, shell=True)
            return 'aborted'
        except Exception,e:
            rospy.logerr(e)
            return error
        else:
            return 'succeeded'



class GeneralAnswer(State):
    def __init__(self):
        State.__init__(self , outcomes = ['succeeded' , 'aborted' , 'error'], 
                            input_keys = ['people_condition' ,'command'],
                            output_keys = ['targets','actions','task_num'])

        self.answer_client = rospy.ServiceProxy('xm_speech_meaning' , xm_Speech_meaning)
        self.speak_client = rospy.ServiceProxy('tts' , xm_Speech_tts)
    def execute(self , userdata):
        try:
            getattr(userdata , 'people_condition')
            getattr(userdata , 'command')

        except Exception ,e:
            rospy.logerr('param error')
            rospy.logerr(e)

            return 'error'

        try:
            self.answer_client.wait_for_service(10.0)
            self.cmd_rec = userdata.command
            self.ans = self.answer_client.call(command = self.cmd_rec )
            
            rospy.logwarn(self.ans)

            if(self.cmd_rec == 2):
                userdata.targets= self.ans.target
                userdata.actions = self.ans.action
                userdata.task_num = len(self.ans.action)

                return 'succeeded'
            
            self.speak_sentence = ''
            rospy.logwarn('ansnum = '+ str(self.ans.num))
            if(self.ans.num>0):
                #it depends on people_recnogstion
		        
                if self.ans.num == 2:
                    self.speak_sentence = str(self.peo_con['Male'])
                elif self.ans.num == 3:
                    self.speak_sentence= str(self.peo_con['Female'])
                elif self.ans.num == 4:
                    self.speak_sentence= str(self.peo_con['All'])
                else:
                    self.speak_sentence = 'please say again'
                
                speech_bool = self.speak_client.call(self.speak_sentence)
                if speech_bool.flag == 1:
                    subprocess.call(["play", "tts_sample.wav"])
                elif speech_bool.flag == 0:
                    subprocess.call("espeak -vf5 -s 75 '%(a)s'" %
                                {'a': str(self.speak_sentence)}, shell=True)


            
            
            return 'succeeded'
        except rospy.ROSException , e:
            rospy.logerr(e)
            return 'aborted'
        except Exception,e:
            rospy.logerr(e)
            return 'error'
        

class CheckStop(State):
    def __init__(self):
        State.__init__(self,
                        outcomes = ['stop','aborted'])
    
        self.target_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)

    def execute(self,userdata):
        try:
            while not rospy.is_shutdown():
                self.target_client.wait_for_service(timeout = 10)
                self.response = self.target_client.call(command = 5)
                self.action = self.response.action
                if self.action[0] == 'stop':
                    return 'stop'
                else:
                    continue
        except Exception,e:
            rospy.logerr('xm meet wrong when get the target')
            return 'aborted'
                
            



            

        
