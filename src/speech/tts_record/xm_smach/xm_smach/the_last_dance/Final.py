#! /usr/bin/env python
# encoding:utf8

import rospy
from smach import StateMachine,Concurrence,CBState
from smach_ros import IntrospectionServer
from new_lib.basic_move import *
from new_lib.basic_vision import *
from new_lib.basic_voice import *
from new_lib.special import *
from new_lib.basic_pick import *
from geometry_msgs.msg import *
from xm_smach.target_gpsr import gpsr_target
from xm_smach.common_lib import Place2,Pick2
import math
import subprocess
from xm_smach.pick_turn import PickTurn, IsTurn
import time


class Final():
    def __init__(self):
        rospy.init_node('FinalSmach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('gogoogo')
        self.smach_bool = False

        self.handleTask = StateMachine(outcomes = ['succeeded' , 'aborted' , 'error'])

        with self.handleTask:
            self.handleTask.userdata.diningroom_pos = gpsr_target['diningroom']['pos']
            self.handleTask.userdata.sofa_pos = Pose(Point(3.567 , 0.566 , 0),Quaternion(0,0,0.717 , 0.697))
            self.handleTask.userdata.girl_pos = Pose(Point(4.575 , 0.966 , 0 ) , Quaternion(0,0,0.717 , 0.697))
            
            self.handleTask.userdata.sentence1 = 'Please command me'
            self.handleTask.userdata.sentence2 = 'There are two people sitting on the couch.'
            self.handleTask.userdata.sentence3 = 'Here you are'
            self.handleTask.userdata.answer1 = 'Ok'
            self.handleTask.userdata.wait_len = 5.0
            self.handleTask.userdata.short_wait = 1.0
            self.handleTask.userdata.answer2 = 'Yes'
            # StateMachine.add('NAV_DIN1' , NavStack() , 
            #                                 transitions = {'succeeded':'SPEAK1' , 
            #                                                 'aborted':'NAV_DIN1' , 
            #                                                 'error':'error',
            #                                                 'preempted':'NAV_DIN1'},
            #                                 remapping = {'pos_xm':'diningroom_pos'})

            StateMachine.add('SPEAK1' , SpeakSentence() , 
                                        transitions = {'succeeded':'WAIT1',
                                                        'error':'error',
                                                        'aborted':'WAIT1'},
                                        remapping = {'sentences':'sentence1'})
            
            StateMachine.add('WAIT1' , Wait() , 
                                        transitions = {'succeeded':'ANS1',
                                                        'error':'error'},
                                        remapping = {'rec':'wait_len'})
            
            StateMachine.add('ANS1' , SpeakSentence() , 
                                        transitions = {'succeeded':'NAV_SOFA',
                                                        'error':'error',
                                                        'aborted':'NAV_SOFA'},
                                        remapping = {'sentences':'answer1'})
            
            StateMachine.add('NAV_SOFA' , NavStack(),
                                            transitions={'succeeded':'RUNNODE',
                                                            'aborted':'NAV_SOFA',
                                                            'error':'error',
                                                            'preempted':'NAV_SOFA'},
                                            remapping = {'pos_xm':'sofa_pos'})

            StateMachine.add('RUNNODE' , CBState(self.RunNode , outcomes = ['succeeded' , 'aborted']) ,
                                        transitions = {'succeeded':'NAV_DIN2',
                                                        'aborted':'NAV_DIN2'})
        
            StateMachine.add('NAV_DIN2' , NavStack() , 
                                            transitions = {'succeeded':'SPEAK2',
                                                            'error':'error',
                                                            'aborted':'NAV_DIN2',
                                                            'preempted':'NAV_DIN2'},
                                            remapping = {'pos_xm':'diningroom_pos'})
                                                
            
            StateMachine.add('SPEAK2' , SpeakSentence() , 
                                        transitions = {'succeeded':'WAIT3',
                                                        'error':'error',
                                                        'aborted':'WAIT3'},
                                        remapping = {'sentences':'sentence2'})

            StateMachine.add('WAIT3' , Wait() , 
                                        transitions = {'succeeded':'ANS2',
                                                        'error':'error'},
                                        remapping = {'rec':'wait_len'})
            
            StateMachine.add('ANS2' , SpeakSentence() , 
                                        transitions = {'succeeded':'WAIT4',
                                                        'error':'error',
                                                        'aborted':'WAIT4'},
                                        remapping = {'sentences':'answer2'})
            StateMachine.add('WAIT4' , Wait() , 
                                        transitions = {'succeeded':'ANS3',
                                                        'error':'error'},
                                        remapping = {'rec':'wait_len'})
            StateMachine.add('ANS3' , SpeakSentence() , 
                                        transitions = {'succeeded':'PICK',
                                                        'error':'error',
                                                        'aborted':'PICK'},
                                        remapping = {'sentences':'answer1'})

            StateMachine.add('PICK' , Pick2() , 
                                        transitions = {'succeeded':'NAV_GIRL',
                                                        'aborted':'NAV_GIRL'})
            
            StateMachine.add('NAV_GIRL' , NavStack() , 
                                            transitions = {'succeeded':'SPEAK3',
                                                            'error':'error',
                                                            'aborted':'NAV_GIRL',
                                                            'preempted':'NAV_GIRL'},
                                            remapping = {'pos_xm':'girl_pos'})
            
            StateMachine.add('SPEAK3' , SpeakSentence() , 
                                        transitions = {'succeeded':'PLACE',
                                                        'error':'error',
                                                        'aborted':'PLACE'},
                                        remapping = {'sentences':'sentence3'})
            

            StateMachine.add('PLACE' , Place2() , 
                                            transitions = {'succeeded':'succeeded',
                                                            'aborted':'aborted'})
            

            






        intro_server = IntrospectionServer('sm_gpsr',self.handleTask,'/SM_ROOT')
        intro_server.start()
        out_2 = self.handleTask.execute()
        intro_server.stop()
        self.smach_bool = True 
    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')
    

    def RunNode(self , userdata):
        try:
            subprocess.call('./Vision/Camera/take_photo_5s/build/take_photos_5s ' , shell=True)
            return 'succeeded'
        except Exception,e:
            rospy.logerr(e)
            return 'aborted'




if __name__ == '__main__':
    Final()