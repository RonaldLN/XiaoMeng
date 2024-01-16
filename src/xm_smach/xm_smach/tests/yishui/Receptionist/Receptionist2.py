#!/usr/bin/env python
# encoding:utf8


'''
Author: yishui
Date: 2022-11-25
LastEditTime: 2022-11-25 22:54:18
LastEditors: yishui
Description: Codes for Receptionist
FilePath: ~/catkin_ws/src/xm_smach/xm_smach/tests/yishui/Receptionist
'''


import rospy
from std_msgs.msg import String, Int32, Bool, Header
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_common.common import *
from smach_special.Receptionist2_lib import *
from tf.transformations import quaternion_matrix
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from copy import deepcopy


class Receptionist():
    def __init__(self):
        rospy.init_node('Receptionist_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logwarn('Welcome to Receptionist!!!')
        self.smach_bool = False
               
        #顶层状态机
        self.RECEPTIONIST = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.RECEPTIONIST:
            # 第一个人为主人，后两个为客人
            self.RECEPTIONIST.userdata.name_list = []
            self.RECEPTIONIST.userdata.drink_list = []
            self.RECEPTIONIST.userdata.face_name_dict ={'wrz':'tom'}
            # zjw,wyz,scl
            # self.RECEPTIONIST.userdata.pos_door = gpsr_target['open_door_pose']['pos']#开门的位置
            # self.RECEPTIONIST.userdata.pos_scan = gpsr_target['receptionist_pose']['pos']#观察的位置
            #self.RECEPTIONIST.userdata.pos_door = gpsr_target['test_pos']['pos']#开门的位置
            #self.RECEPTIONIST.userdata.pos_scan = gpsr_target['receptionist_pose_test']['pos']
            # self.RECEPTIONIST.userdata.point_pos = gpsr_target['point_pose']['pos']#指座位的位置

            self.RECEPTIONIST.userdata.pos_door = gpsr_target['door_pos']['pos']#开门的位置
            self.RECEPTIONIST.userdata.bedroom_pos = gpsr_target['bedroom_pos']['pos']
            self.RECEPTIONIST.userdata.char1_pos = gpsr_target['char1']['pos']
            self.RECEPTIONIST.userdata.char2_pos = gpsr_target['char2']['pos']

            self.RECEPTIONIST.userdata.med_pos = gpsr_target['med_pos']['pos']

            # self.RECEPTIONIST.userdata.sentences_ask_again = 'sorry, can you say it again'
            self.RECEPTIONIST.userdata.sentences_ask = 'welcome to you,please tell me your name and your favourite drink.'
            # self.RECEPTIONIST.userdata.sentences_look_at_me = 'please look at me'
            self.RECEPTIONIST.userdata.sentences_point = 'please have a seat.'
            self.RECEPTIONIST.userdata.sentences_open_door = 'please help me open the door.'
            self.RECEPTIONIST.userdata.sentences_sit='please sit in front of me '
            self.RECEPTIONIST.userdata.receptionist_sentence=''
            self.RECEPTIONIST.userdata.straight_angle = 0
            self.RECEPTIONIST.userdata.rec_wait = 3
            self.RECEPTIONIST.userdata.sentences_follow = 'please follow me.'
            self.RECEPTIONIST.userdata.arm_back_waypoints = [[0,-1.3,1.57,0]]
            self.RECEPTIONIST.userdata.des = ''

            StateMachine.add('NAVDOOR1',
                                NavStack(),
                                transitions={'succeeded':'SAY_HELLO1','aborted':'NAVDOOR1','error':'error'},
                                remapping ={'pos_xm':'pos_door'}
                                )
            # 问好,问他的需要
            StateMachine.add('SAY_HELLO1',
                                SpeakText(),
                                transitions={'succeeded':'GETINFORMATION1','aborted':'SAY_HELLO1','error':'error'},
                                remapping={'sentences':'sentences_ask'}
                                )
            StateMachine.add('GETINFORMATION1',
                                GetNameAndDrink(),
                                transitions={'succeeded':'FIND_GUEST1','aborted':'GETINFORMATION1','error':'error'}
                                )
            # 找到客人，添加一个人脸识别中人的姓名和指定姓名的映射
            StateMachine.add('FIND_GUEST1',
                                FaceRecognition1(),
                                transitions={'succeeded':'SAY_FOLLOW1','aborted':'FIND_GUEST1','error':'error'}
                                )
            StateMachine.add('SAY_FOLLOW1',
                                SpeakText(),
                                transitions={'succeeded':'NAV_GUEST1','aborted':'SAY_FOLLOW1','error':'error'},
                                remapping={'sentences':'sentences_follow'}
                                ) 
            # 走到一个位置，既可以看到主人也可以看到客人，并且此时指向座位1
            StateMachine.add('NAV_GUEST1',
                                NavStackReceptionist(),
                                transitions={'succeeded':'SAY_SIT1','aborted':'NAV_GUEST1','error':'error'},
                                remapping ={'pos_xm':'char1_pos'}
                                )
            StateMachine.add('SAY_SIT1',
                                SpeakText(),
                                transitions={'succeeded':'NAV_HOST1','aborted':'SAY_SIT1','error':'error'},
                                remapping={'sentences':'sentences_sit'}
                                )
            StateMachine.add('NAV_HOST1',
                                NavStack(),
                                transitions={'succeeded':'INTRODUCE_GUEST1','aborted':'NAV_HOST1','error':'error'},
                                remapping ={'pos_xm':'bedroom_pos'}
                                )
            StateMachine.add('INTRODUCE_GUEST1',
                                IntroduceGuest1(),
                                transitions={'succeeded':'BACK_DOOR','aborted':'INTRODUCE_GUEST1','error':'error'}
                                )
            StateMachine.add('BACK_DOOR',
                                NavStackReceptionist(),
                                transitions={'succeeded':'SAY_HELLO2','aborted':'NAVDOOR1','error':'error'},
                                remapping ={'pos_xm':'pos_door'}
                                )
            # 问好,问他的需要
            StateMachine.add('SAY_HELLO2',
                                SpeakText(),
                                transitions={'succeeded':'GETINFORMATION2','aborted':'SAY_HELLO2','error':'error'},
                                remapping={'sentences':'sentences_ask'}
                                )
            StateMachine.add('GETINFORMATION2',
                                GetNameAndDrink(),
                                transitions={'succeeded':'SAY_FOLLOW2','aborted':'GETINFORMATION2','error':'error'}
                                )
            StateMachine.add('SAY_FOLLOW2',
                                SpeakText(),
                                transitions={'succeeded':'NAV_GUEST2','aborted':'SAY_FOLLOW2','error':'error'},
                                remapping={'sentences':'sentences_follow'}
                                ) 
            # 走到一个位置，可以看到三个人，并且此时指向座位2
            StateMachine.add('NAV_GUEST2',
                                NavStackReceptionist(),
                                transitions={'succeeded':'SAY_SIT2','aborted':'NAV_GUEST2','error':'error'},
                                remapping ={'pos_xm':'char2_pos'}
                                )
            StateMachine.add('SAY_SIT2',
                                SpeakText(),
                                transitions={'succeeded':'INTRODUCE_GUEST3','aborted':'SAY_SIT2','error':'error'},
                                remapping={'sentences':'sentences_sit'}
                                )
            StateMachine.add('INTRODUCE_GUEST3',
                                IntroduceGuest1(),
                                transitions={'succeeded':'NAV_HOST2','aborted':'INTRODUCE_GUEST3','error':'error'}
                                )
            StateMachine.add('NAV_HOST2',
                                NavStack(),
                                transitions={'succeeded':'GUEST_OR_HOST','aborted':'NAV_HOST2','error':'error'},
                                remapping ={'pos_xm':'bedroom_pos'}
                                )
            StateMachine.add('GUEST_OR_HOST',
                                FaceRecognition2(),
                                transitions={'true':'INTRODUCE_GUEST2','false':'NAV_GUEST3','aborted':'GUEST_OR_HOST','error':'error'}
                                )
            # 此地不为主人
            StateMachine.add('NAV_GUEST3',
                                NavStack(),
                                transitions={'succeeded':'INTRODUCE_GUEST2','aborted':'NAV_GUEST3','error':'error'},
                                remapping ={'pos_xm':'char1_pos'}
                                )          
            # 此地为主人
            StateMachine.add('INTRODUCE_GUEST2',
                                IntroduceGuest2(),
                                transitions={'succeeded':'succeeded','aborted':'INTRODUCE_GUEST2','error':'error'}
                                )                


            # StateMachine.add('FIND_TWO_PEOPLE',
            #                  FindTwoPeople(),  # 识别到两个人的位置，计算出全局位置并返回
            #                  transitions={'succeeded': 'RECEIVE_TASKS', 'aborted': 'FIND_ALL_PEOPLE', 'error': 'error'})
            
            # StateMachine.add('FIND_TWO_PEOPLE',
            #                  FindTwoPeople(),  # 识别到两个人的位置，计算出全局位置并返回
            #                  transitions={'succeeded': 'RECEIVE_TASKS', 'aborted': 'FIND_ALL_PEOPLE', 'error': 'error'})

            # 向主人介绍客人信息
            # StateMachine.add('NAV_HOST',
            #                     NavStack(),
            #                     transitions={'succeeded':'ASK_LOOK','aborted':'NAVDOOR1','error':'error'},
            #                     remapping ={'pos_xm':'pos_door'}
            #                     )            
            # StateMachine.add('ASK_AGAIN',
            #                     Speak(),
            #                     transitions={'succeeded':'GETINFORMATION1','aborted':'ASK_AGAIN','error':'error'},
            #                     remapping ={'sentences':'sentences_ask_again'}
            #                     )    
            # StateMachine.add('TAKE_A_PHOTO',
            #                     TakeAPhoto(),
            #                     transitions={'succeeded':'GET_RECEPTION_WORLD1','aborted':'TAKE_A_PHOTO','error':'error'},
            #                     remapping={'des':'des'}
            #                     )               
            # StateMachine.add('GET_RECEPTION_WORLD1',
            #                     GetReceptionWorld(),
            #                     transitions={'succeeded':'RECEPTIONIST1','aborted':'aborted','error':'error'},
            #                     remapping ={'name_list':'name_list','drink_list':'drink_list'}
            #                     )
            # StateMachine.add('RECEPTIONIST1',
            #                     Speak(),
            #                     transitions={'succeeded':'SPEAK_FOLLOW1','aborted':'aborted','error':'error'},
            #                     remapping ={'sentences':'receptionist_sentence'}
            #                     )
            # StateMachine.add('TAKEBACKARM_AFTER_POINT1',
            #                     ArmTrajectory(),
            #                     transitions={'succeeded':'SPEAK_FOLLOW1','error':'error'},
            #                     remapping ={'arm_waypoints':'arm_back_waypoints'}
            #                     )   
            # StateMachine.add('SPEAK_FOLLOW1',
            #                     Speak(),
            #                     transitions={'succeeded':'NAVLIVINGROOM1','aborted':'NAVLIVINGROOM1','error':'error'},
            #                     remapping ={'sentences':'sentences_follow'}
            #                     ) 
            # StateMachine.add('NAVLIVINGROOM1',
            #                     NavStack(),
            #                     transitions={'succeeded':'ASK_LOOK2','aborted':'NAVLIVINGROOM1','error':'error'},
            #                     remapping ={'pos_xm':'pos_scan'}
            #                     )

            # StateMachine.add('ASK_LOOK2',
            #                     Speak(),
            #                     transitions={'succeeded':'POINT_HOST','aborted':'POINT_HOST','error':'error'},
            #                     remapping ={'sentences':'sentences_look_at_me'}
            #                     ) 

            # StateMachine.add('POINT_HOST',
            #                     self.xm_PointHost1,
            #                     transitions={'succeeded':'POINTSEAT1','aborted':'POINT_HOST','error':'error'} ,
            #                     remapping={'point_pos':'point_pos','name_list':'name_list','drink_list':'drink_list'})

            # StateMachine.add('POINTSEAT1',
            #                     self.xm_PointSeat,
            #                     transitions={'succeeded':'NAVDOOR2','aborted':'aborted','error':'error'} ,
            #                     remapping={'point_pos':'point_pos'})
            
            # StateMachine.add('NAVDOOR2',
            #                     NavStack(),
            #                     transitions={'succeeded':'GETINFORMATION2','aborted':'GETINFORMATION2','error':'error'},
            #                     remapping ={'pos_xm':'pos_door'}
            #                     )
            # StateMachine.add('SPEAK_OPEN_DOOR2',
            #                     Speak(),
            #                     transitions={'succeeded':'GETINFORMATION2','aborted':'GETINFORMATION2','error':'error'},
            #                     remapping ={'sentences':'sentences_open_door'}
            #                     )
            # StateMachine.add('ASK2',
            #                     Speak(),
            #                     transitions={'succeeded':'GETINFORMATION2','aborted':'GETINFORMATION2','error':'error'},
            #                     remapping ={'sentences':'sentences_ask'}
            #                     ) 
            # StateMachine.add('GETINFORMATION2',
            #                     GetNameAndDrink(),
            #                     transitions={'succeeded':'GET_RECEPTION_WORLD2','aborted':'GETINFORMATION2','error':'error'},
            #                     remapping ={'name_list':'name_list','drink_list':'drink_list'}
            #                     )
            # StateMachine.add('POINT_GUEST2',
            #                     Point_Guest(),
            #                     transitions={'succeeded':'GET_RECEPTION_WORLD2','aborted':'aborted','error':'error'},
            #                     remapping={'angle':'straight_angle'}
            #                     )
            # StateMachine.add('GET_RECEPTION_WORLD2',
            #                     GetReceptionWorld(),
            #                     transitions={'succeeded':'RECEPTIONIST2','aborted':'aborted','error':'error'},
            #                     remapping ={'name_list':'name_list','drink_list':'drink_list'}
            #                     )
            # StateMachine.add('RECEPTIONIST2',
            #                     Speak(),
            #                     transitions={'succeeded':'SPEAK_FOLLOW2','aborted':'aborted','error':'error'},
            #                     remapping ={'sentences':'receptionist_sentence'}
            #                     ) 
            # StateMachine.add('TAKEBACKARM_AFTER_POINT2',
            #                     ArmTrajectory(),
            #                     transitions={'succeeded':'SPEAK_FOLLOW2','error':'error'},
            #                     remapping ={'arm_waypoints':'arm_back_waypoints'}
            #                     ) 
            # StateMachine.add('SPEAK_FOLLOW2',
            #                     Speak(),
            #                     transitions={'succeeded':'NAVLIVINGROOM2','aborted':'aborted','error':'error'},
            #                     remapping ={'sentences':'sentences_follow'}
            #                     ) 
            # StateMachine.add('NAVLIVINGROOM2',
            #                     NavStack(),
            #                     transitions={'succeeded':'POINT_HOST_1','aborted':'POINTSEAT2','error':'error'},
            #                     remapping ={'pos_xm':'pos_scan'}
            #                     )
            # StateMachine.add('POINT_HOST_1',
            #                     self.xm_PointHost2,
            #                     transitions={'succeeded':'POINT_GUEST','aborted':'POINT_HOST_1','error':'error'} ,
            #                     remapping={'point_pos':'point_pos','name_list':'name_list','drink_list':'drink_list'})
            # StateMachine.add('POINT_GUEST',
            #                     self.xm_PointGuest,
            #                     transitions={'succeeded':'POINTSEAT2','aborted':'POINT_GUEST','error':'error'} ,
            #                     remapping={'point_pos':'point_pos','name_list':'name_list','drink_list':'drink_list','des':'des'})
            # StateMachine.add('POINTSEAT2',
            #                     self.xm_PointSeat,
            #                     transitions={'succeeded':'NAVDOOR3','aborted':'POINTSEAT2','error':'error'},
            #                     remapping={'point_pos':'point_pos'})  
            # StateMachine.add('NAVDOOR3',
            #                     NavStack(),
                                # transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                # remapping ={'pos_xm':'pos_door'}
                                # )                                              
        intro_server = IntrospectionServer('RECEPTIONIST',self.RECEPTIONIST,'/XM_ROOT')
        intro_server.start()
        out = self.RECEPTIONIST.execute()
        intro_server.stop()

    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.logwarn('smach succeeded')


if __name__ == "__main__":
    try:
        Receptionist()
    except Exception,e:
        rospy.logerr(e)   
