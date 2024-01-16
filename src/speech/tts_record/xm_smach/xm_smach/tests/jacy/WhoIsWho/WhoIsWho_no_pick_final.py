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
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
'''
Who Is Who
1.进入房间:等待开门-》进入房间-》介绍自己
2.打开摄像头，获取三个人的位置
3.面对视野中第一个人，获取信息


'''

class DoorDetect(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded','aborted', 'error'])
        rospy.Subscriber('DoorState', Bool, self.door_cb)
        self.door_state = False
    def execute(self, userdata):
        while not self.door_state:
            pass
        return 'succeeded'
    def door_cb(self, msg):
        if msg.data == True:
            self.door_state = True
            # 门开着刷新建图，刷新后返回
            clear_client = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            req = EmptyRequest()
            res = clear_client.call()


class WhoIsWho():
    def __init__(self):
        rospy.init_node('WhoIsWho_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to WhoIsWho!!!')
        self.smach_bool = False
        
        self.waypoints = list()
        self.waypoints.append(gpsr_target['who_observe_1']['pos'])     #找人的点
        self.waypoints.append(gpsr_target['who_observe_3']['pos'])     #抓东西的点
        self.waypoints.append(gpsr_target['who_observe_2']['pos'])     #结束出门的点
        self.waypoints.append(gpsr_target['exit_pos']['pos'])          #结束出门的点

        self.sm_EnterRoom = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_EnterRoom:
            self.sm_EnterRoom.userdata.sentences = 'I am robot xiaomeng'
            self.sm_EnterRoom.userdata.start_waypoint = self.waypoints[0]
            self.sm_EnterRoom.userdata.rec = 3.0
            
            StateMachine.add('DOORDETECT',
                                DoorDetect(),
                                transitions={'succeeded':'WAIT','aborted':'DOORDETECT','error':'error'})
            StateMachine.add('WAIT',
                                Wait(),
                                remapping ={'rec':'rec'},
                                transitions ={'succeeded':'NAV','error':'error'})              
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
            self.sm_FaceDetect.userdata.filename = '/home/domistic/lookatme.wav'
            self.sm_FaceDetect.userdata.degree_l = 3.1415926535/8
            self.sm_FaceDetect.userdata.degree_r = -3.1415926535/8
            self.sm_FaceDetect.userdata.distance = 1.0
            StateMachine.add('SPEAK',
                                SpeakWAV(),
                                remapping = {'filename':"filename"},
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
            self.sm_Remember.userdata.filename = "/home/domistic/whoiswho.wav"   
            StateMachine.add('NAV',
                                NavStack(),
                                remapping ={'pos_xm':'person_position'},
                                transitions ={'succeeded':'TALK','aborted':'NAV','error':'error'}
                                )
                                
            
            StateMachine.add('TALK',
                                SpeakWAV(),
                                remapping ={'filename':'filename'},
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
            self.sm_GiveBack.userdata.distance = 1.0
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
                                              'train_error':'FACE_RECO'}
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


        #顶层状态机
        self.WHOISWHO = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.WHOISWHO:
            self.WHOISWHO.userdata.people_position =list()
            self.WHOISWHO.userdata.num_list = list()
            self.WHOISWHO.userdata.person_position = PointStamped()
            self.WHOISWHO.userdata.name_list =list()
            self.WHOISWHO.userdata.target_list =list()
            self.WHOISWHO.userdata.nav_ps1 = self.waypoints[0]
            self.WHOISWHO.userdata.nav_ps2 = self.waypoints[1]
            self.WHOISWHO.userdata.nav_ps3 = self.waypoints[2]
            self.WHOISWHO.userdata.nav_ps4 = self.waypoints[3]
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
                                transitions ={'succeeded':'REMEMBER','aborted':"NAV_ROOM",'error':'error'}
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
                                transitions ={'succeeded':'GETPERSON','aborted':'aborted','error':'error'}
                                )
            #name_list,target_list:0 1 2
            #num_list:2 1 0
            #到此为止，我们已经获取了三个人的名字和物体列表

            #开始识别
            StateMachine.add('NAV_ROOM',
                                NavStack(),
                                remapping ={'pos_xm':'nav_ps2'},
                                transitions ={'succeeded':'NAV_FIND','aborted':'NAV_ROOM','error':'error'})

            StateMachine.add('NAV_FIND',
                                NavStack(),
                                remapping ={'pos_xm':'nav_ps3'},
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
                                             'continue':'NAV_FIND'},
                                remapping={'num_list':'num_list'})   
            StateMachine.add('NAV_BYEBYE',
                                NavStack(),
                                remapping ={'pos_xm':'nav_ps4'},
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
