#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import *
from smach_ros import *

from smach_common.common import *
from smach_compose.compose import *
from smach_special.whoiswho import *

from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from copy import deepcopy

from xm_msgs.srv import *
from xm_msgs.msg import *

import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
'''
Who Is Who
1.进入房间:等待开门-》进入房间-》介绍自己
2.打开摄像头，获取三个人的位置
3.面对视野中第一个人，获取信息
'''

class WhoIsWho():
    def __init__(self):
        rospy.init_node('WhoIsWho_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to WhoIsWho!!!')
        self.smach_bool = False
        
        self.waypoints = list()
        self.waypoints.append(gpsr_target['kitchen']['pos'])     #找人的点
        self.waypoints.append(gpsr_target['shelf']['pos'])     #抓东西的点
        self.waypoints.append(gpsr_target['kitchen']['pos'])     #结束出门的点

        self.sm_EnterRoom = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_EnterRoom:
            self.sm_EnterRoom.userdata.sentences = 'I am robot xiaomeng'
            self.sm_EnterRoom.userdata.start_waypoint = self.waypoints[0]
            StateMachine.add('NAV',
                                NavStack(),
                                transitions={'succeeded':'SELF_INTRO','aborted':'NAV','error':'error'},
                                remapping = {'pos_xm':'start_waypoint'})
            
            StateMachine.add('SELF_INTRO',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'succeeded','aborted':'SELF_INTRO','error':'error'})
        
        self.sm_FaceDetect = StateMachine(outcomes = ['succeeded','aborted','error'],
                                            output_keys = ['people_position','num_list'])
        with self.sm_FaceDetect:
            self.sm_FaceDetect.userdata.people_position =list()
            self.sm_FaceDetect.userdata.name_id =-1
            self.sm_FaceDetect.userdata.num_list = list()
            self.sm_FaceDetect.userdata.sentences = 'please look at me'
            self.sm_FaceDetect.userdata.degree_l = 3.1415926535/8
            self.sm_FaceDetect.userdata.degree_r = -3.1415926535/8
            self.sm_FaceDetect.userdata.distance = 0.8
            StateMachine.add('SPEAK',
                                Speak(),
                                remapping = {'sentences':"sentences"},
                                transitions = {'succeeded':'GET_POSITION','aborted':'aborted','error':'error'})

            StateMachine.add('GET_POSITION',
                                FaceReco(),
                                remapping  ={'name_id':'name_id','position':'people_position','num_list':'num_list'},
                                transitions ={'succeeded':'succeeded',
                                              'again':'GET_POSITION',
                                              'aborted':'GET_POSITION',
                                              'error':'error',
                                              'turn_l':'TURN_L',
                                              'turn_r':'TURN_R',
                                              'train_error':'aborted'})                 
            StateMachine.add('TURN_L',
                                    TurnDegree(),
                                    transitions={'succeeded':'SPEAK','aborted':'aborted','error':'error'},
                                    remapping = {'degree':'degree_l'}
                                    )              
            StateMachine.add('TURN_R',
                                    TurnDegree(),
                                    transitions={'succeeded':'SPEAK','aborted':'aborted','error':'error'},
                                    remapping = {'degree':'degree_r'}
                                    )              
   

        self.sm_Remember = StateMachine(outcomes =['succeeded','aborted','error'],
                                        input_keys =['person_position'],
                                        output_keys =['name','target']
                                        )
        with self.sm_Remember:  
            self.sm_Remember.userdata.sentences = "what is your name and what do you want？"   
            StateMachine.add('NAV',
                                NavStack(),
                                remapping ={'pos_xm':'person_position'},
                                transitions ={'succeeded':'TALK','aborted':'NAV','error':'error'}
                                )
                                
            
            StateMachine.add('TALK',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'GETINFORMATION','aborted':'TALK','error':'error'})           
            StateMachine.add('GETINFORMATION',
                                NameAndThing(),
                                remapping ={'name':'name','target':'target'},
                                transitions ={'succeeded':'succeeded','aborted':'GETINFORMATION','error':'error'})


        self.sm_GiveBack = StateMachine(outcomes =['succeeded','aborted','error'],
                                            input_keys =['name_id','name_list','target_list'])# the name is a string
        with self.sm_GiveBack:
            self.sm_GiveBack.userdata.sentences_look = "please look at me"
            self.sm_GiveBack.userdata.degree_l = 3.1415926535/8
            self.sm_GiveBack.userdata.degree_r = -3.1415926535/8   
            self.sm_GiveBack.userdata.rec =5.0
            self.sm_GiveBack.userdata.distance = 0.7
            StateMachine.add('SPEAK',
                                Speak(),
                                remapping ={'sentences':'sentences_look'},
                                transitions ={'succeeded':'WAIT','aborted':'aborted','error':'error'})
            
            StateMachine.add('WAIT',
                                Wait(),
                                remapping ={'rec':'rec'},
                                transitions ={'succeeded':'FACE_RECO','error':'error'})  
            StateMachine.add('FACE_RECO',
                                FaceReco(),
                                remapping ={'position':'person_position','name_id':'name_id'},
                                transitions ={'succeeded':'NAV_GO',
                                              'again':'FACE_RECO',
                                              'aborted':'FACE_RECO',
                                              'error':'error',
                                              'turn_l':'TURN_L',
                                              'turn_r':'TURN_R',
                                              'train_error':'aborted'}
                                )
                
            StateMachine.add('TURN_L',
                                    TurnDegree(),
                                    transitions={'succeeded':'SPEAK_2','aborted':'aborted','error':'error'},
                                    remapping = {'degree':'degree_l'}
                                    )              
            StateMachine.add('TURN_R',
                                    TurnDegree(),
                                    transitions={'succeeded':'SPEAK_2','aborted':'aborted','error':'error'},
                                    remapping = {'degree':'degree_r'}
                                    )           
            StateMachine.add('SPEAK_2',
                                Speak(),
                                remapping ={'sentences':'sentences'},
                                transitions ={'succeeded':'FACE_RECO','aborted':'aborted','error':'error'})  
            StateMachine.add('NAV_GO',
                                NavStack(),
                                remapping ={'pos_xm':'person_position'},
                                transitions ={'succeeded':'GET_NAME','aborted':'NAV_GO','error':'error'})
            StateMachine.add("GET_NAME",
                                GenerateInformation(),
                                remapping ={'name_list':'name_list','name_id':'name_id','target_list':'target_list', 'sentences':'sentences'},
                                transitions ={'succeeded':'TALK','aborted':'aborted','error':'error'})
            
            StateMachine.add('TALK',
                                Speak(),
                                remapping ={'sentences':"sentences"},
                                transitions ={'succeeded':'succeeded','aborted':"succeeded",'error':'error'})
            #TODO RELEASE THE GRIPPER

        self.sm_GetTarget = StateMachine(outcomes =['succeeded','aborted','error'],
                                            input_keys =['target_name'])#the target is a string.....
        with self.sm_GetTarget:
            self.sm_GetTarget.userdata.distance = 0.7
            self.sm_GetTarget.userdata.nav_ps = self.waypoints[1]
            self.sm_GetTarget.userdata.table_depth = 0.04
            self.sm_GetTarget.userdata.traget_size = [0.04, 0.065, 0.105]
            self.sm_GetTarget.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
            self.sm_GetTarget.userdata.take_back_pose = [ [0,0.026,3.14,0,3.14,0] ]
            StateMachine.add('NAVTOOBSERVEPOS',
                                NavStack(),
                                transitions ={'succeeded':'PICK_POS','aborted':'NAVTOOBSERVEPOS','error':'error'},
                                remapping ={"pos_xm":'nav_ps'})
            
            StateMachine.add('PICK_POS',ArmTrajectory(),
                             transitions={'succeeded': 'FIND_1', 'error': 'error'},
                             remapping={'arm_waypoints':'pick_pose'})                                   
            
            StateMachine.add('FIND_1',GetObjectPosition(),
                             transitions={'succeeded': 'GETPICKPOSE', 'error': 'error'})

            StateMachine.add('GETPICKPOSE',
                                GetPickPos(),
                                transitions ={'succeeded':'NAVTOPICKPOS','error':'error'})       
            
            StateMachine.add('NAVTOPICKPOS',
                                NavStack(),
                                transitions ={'succeeded':'FIND_2','aborted':'NAVTOPICKPOS','error':'error'},
                                remapping ={"pos_xm":'pick_pos'})

            StateMachine.add('FIND_2',GetObjectPosition(),
                             transitions={'succeeded': 'TARGET_POINT_JUSFY', 'error': 'error'})   
                                                               
            StateMachine.add('TARGET_POINT_JUSFY',TargerPosReload(),
                             transitions={'succeeded': 'PICK', 'error': 'error'})

            StateMachine.add('PICK', ArmStack(),
                             transitions={'succeeded': 'NAV_BACK','error': 'error'})
            
            StateMachine.add('NAV_BACK', GoAhead(),
                             transitions={'succeeded': 'BACK_POS','error': 'error'},
                             remapping={'move_len':'back_distance'})
            
            #TODO arm back pose
            StateMachine.add('BACK_POS',ArmTrajectory(),
                             transitions={'succeeded': 'succeeded', 'error': 'error'},
                             remapping={'arm_waypoints':'take_back_pose'}) 

        #顶层状态机
        self.WHOISWHO = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.WHOISWHO:
            self.WHOISWHO.userdata.people_position =list()
            self.WHOISWHO.userdata.num_list = list()
            self.WHOISWHO.userdata.person_position = PointStamped()
            self.WHOISWHO.userdata.name_list =list()
            self.WHOISWHO.userdata.target_list =list()
            self.WHOISWHO.userdata.nav_ps1 = self.waypoints[1]
            self.WHOISWHO.userdata.nav_ps2 = self.waypoints[0]
            StateMachine.add('ENTERROOM',
                                self.sm_EnterRoom,
                                transitions ={'succeeded':'FACEDETECT','aborted':'aborted','error':'error'})

            StateMachine.add('FACEDETECT',
                                self.sm_FaceDetect,
                                remapping ={'people_position':'people_position','num_list':'num_list'},
                                transitions = {'succeeded':'GETPERSON','aborted':'aborted','error':'error'}
                                )
            #people_position:0 1 2
            #num_list = 2 1 0

            StateMachine.add('GETPERSON',
                                GetValue(),
                                remapping ={'element_list':'people_position','element':'person_position'},
                                transitions ={'succeeded':'REMEMBER','aborted':"aborted",'error':'error'}
                                )
            #people_position:2->1->0
            
            StateMachine.add('REMEMBER',
                                self.sm_Remember,
                                remapping ={'person_position':'person_position','name':'name','target':'target'},
                                transitions ={'succeeded':'NAMEINLIST','aborted':'aborted','error':'error'}
                                )
            StateMachine.add('NAMEINLIST',
                                NameInList(),
                                remapping ={'name':'name','target':'target','name_list':'name_list','target_list':'target_list'},
                                transitions ={'succeeded':'GETTARGET','aborted':'aborted','error':'error'}
                                )
            #name_list,target_list:0 1 2
            #num_list:2 1 0
            #到此为止，我们已经获取了三个人的名字和物体列表

            #开始识别
            StateMachine.add('GETTARGET',
                                GetValue(),
                                remapping ={'element_list':'target_list','element':'target'},
                                transitions ={'succeeded':'CATCHTARGET','aborted':'NAV_BYEBYE','error':'error'})
            StateMachine.add('CATCHTARGET',
                                self.sm_GetTarget,
                                transitions= {'succeeded':'NAV_ROOM','aborted':'NAV_ROOM','error':'error'},
                                remapping = {'target_name':'target'})
            

            StateMachine.add('NAV_ROOM',
                                NavStack(),
                                remapping ={'pos_xm':'nav_ps2'},
                                transitions ={'succeeded':'GETID','aborted':'GETID','error':'error'})

            self.WHOISWHO.userdata.name_id = 0   

            #GETID name_id :0 1 2    num_list:2 1 0

            StateMachine.add('GETID',
                                GetId(),
                                remapping ={'output_id':'name_id','input_list':'name_list','num_list':'num_list'},
                                transitions ={'succeeded':'GIVEBACK','aborted':'NAV_BYEBYE','error':'error'}
                                )
            #GIVE BACK :2 1 0  name_list target_list
            StateMachine.add('GIVEBACK',
                                self.sm_GiveBack,
                                remapping ={'name_id':'name_id','name_list':'name_list','target_list':'target_list'},
                                transitions ={'succeeded':'CHECKFINISH','aborted':'CHECKFINISH','error':'error'}
                                )

            StateMachine.add('CHECKFINISH',
                                CBState(self.checkfinish,outcomes=['finish','continue'],input_keys=['num_list']),
                                transitions={'finish':'NAV_BYEBYE',
                                             'continue':'NAV_ROOM'},
                                remapping={'num_list':'num_list'})
            self.WHOISWHO.userdata.nav_exit = self.waypoints[2]     
            StateMachine.add('NAV_BYEBYE',
                                NavStack(),
                                remapping ={'pos_xm':'nav_exit'},
                                transitions ={'succeeded':'succeeded','aborted':'NAV_BYEBYE','error':'error'})
                                                    

        intro_server = IntrospectionServer('WHOISWHO',self.WHOISWHO,'/WHOISWHO')
        intro_server.start()
        out = self.WHOISWHO.execute()
        intro_server.stop()

  
    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

    def checkfinish(self,ud):
        print ud.num_list
        if len(ud.num_list) == 0:
            return 'finish'
        else:
            return 'continue'

if __name__ == "__main__":
    try:
        WhoIsWho()
    except Exception,e:
        rospy.logerr(e)   
