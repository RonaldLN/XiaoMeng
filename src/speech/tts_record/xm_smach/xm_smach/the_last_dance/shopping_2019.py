#! /usr/bin/env python
#encoding:utf8

import rospy
from smach_ros import IntrospectionServer
from xm_smach.common_lib import *
from xm_smach.shopping_lib import * 
from xm_smach.target_gpsr import *
from smach import State, StateMachine, Concurrence
from math import pi
from geometry_msgs.msg import *
import subprocess



class Shopping():
    def __init__(self):

        rospy.init_node('Shopping')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('gogoogo')
        self.smach_bool = False

        self.get_command = StateMachine(outcomes =['succeeded','aborted'])
        with self.get_command:
            StateMachine.add('GET_FOLLOW',
                                GetSignal(),
                                transitions={'succeeded':'succeeded','aborted':'aborted'})

        self.FollowAndLabel = StateMachine(outcomes = ['succeeded','aborted','error'],
                                            io_keys = ['ObjPos'])
        with self.FollowAndLabel:
            self.FollowAndLabel.userdata.ObjName = ''
            StateMachine.add('FOLLOW' , self.follow , 
                                transitions={'Label':'ASK_NAME' ,  'STOP':'LABLE_POS' ,
                                            'aborted':'aborted' , 'error':'error' })
            StateMachine.add('ASK_NAME' , ShoppingGetObjName(),
                                transitions={'succeeded':'succeeded',
                                            'aborted':'aborted',
                                            'error':'error'},
                                remapping={'ObjName':'ObjName'}) 
            StateMachine.add('LABLE_POS' , LablePos() , transitions={'continue':'SPEAK' , 'finish':'CLOSE_CAM' ,'error':'error'},
                                remapping={'ObjName':'ObjName',
                                            'ObjPos':'ObjPos'})

            self.FollowAndLabel.userdata.con_speak = 'Show me the next one'  

            StateMachine.add('SPEAK' , Speak() , transitions={'succeeded':'FOLLOW',
                                                                'aborted':'FOLLOW',
                                                                'error':'error'})     
            StateMachine.add('CLOSE_CAM' , )     