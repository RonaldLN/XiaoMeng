#!/usr/bin/env python
# encoding:utf8
from os import WIFSTOPPED
import os
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_special.gpsr import * 
from smach_compose.compose import * 
from smach_common.common import * 
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

'''
Author: yyb
Date: 2021-01-21 
LastEditTime: 2021-01-21 19:17:27
LastEditors: yyb
Description: Codes for GPSR
FilePath: /xm_smach/tests/yyb/GPSR
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

class Gpsr():
    def __init__(self):
        rospy.init_node("Gpsr_Smach")
        rospy.on_shutdown(self.shutdown)
        rospy.logerr("Welcome to GPSR!")
        # empty pid file
        dir_path = '/home/xm/vision_pid/'
        for file in os.listdir(dir_path):
            with open(os.path.join(dir_path,file),'w') as f:
                f.write('')
        print('Empty PID Files Success!')
        self.smach_bool = False
        
        #-------Enter_Room-------#
        #任务开始，首先要识别是否门已经打开，如果门已经打开，自动进入房间
        #首先进入的房间一定是Livingroom
        self.xm_EnterRoom = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.xm_EnterRoom:
            self.xm_EnterRoom.userdata.start_waypoint = gpsr_target['instruction_pos']['pos']
            self.xm_EnterRoom.userdata.rec = 3.0
            

            StateMachine.add('DOORDETECT',
                                DoorDetect(),
                                transitions={'succeeded':'WAIT','aborted':'DOORDETECT','error':'error'})
            
            StateMachine.add('WAIT',
                                Wait(),#等待三秒
                                remapping ={'rec':'rec'},
                                transitions ={'succeeded':'NAV','error':'error'})              
            
            StateMachine.add('NAV',
                                NavStack(),#移动到livingroom
                                transitions={'succeeded':'succeeded','aborted':'NAV','error':'error'},
                                remapping = {'pos_xm':'start_waypoint'})
        
        #---------Tell---------#
        self.xm_Tell = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys = ['target','current_task','answer','count_num','objectName'],
                                    output_keys=['objectName'])
        with self.xm_Tell:

            self.xm_Tell.userdata.people_condition = list()
            #self.xm_Tell.userdata.sentences = ""
            self.xm_Tell.userdata.target_mode = 0
            self.xm_Tell.userdata.sentences_num = "the number is " 
            self.xm_Tell.userdata.sentences_name = "the object you asked is "

            StateMachine.add('GETTARGET',
                                GetTarget(),
                                transitions = {'succeeded':'CHECK_NUM_OR_ANS','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task','current_target':'current_target','mode':'target_mode'}
                            )
            StateMachine.add('CHECK_NUM_OR_ANS',
                                    CheckNumOrANS(),
                                    transitions = {'answer':'ANSWER','num':'SPEAKNUM','name':"SPEAKNAME"},
                                    remapping = {'current_target':'current_target'}) 
            StateMachine.add('SPEAKNAME',
                                    SpeakName(),#避免userdata.sentences异常，可以通过将getattr加入到Talk状态机中来取代这一步
                                    transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                    remapping = {'sentences':'sentences_name','objectName':'objectName'})                           
            StateMachine.add('SPEAKNUM',
                                    SpeakNum(),#避免userdata.sentences异常，可以通过将getattr加入到Talk状态机中来取代这一步
                                    transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                    remapping = {'sentences':'sentences_num','count_num':"count_num"})
            StateMachine.add('ANSWER',
                                Speak(),
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping={'sentences':'answer'})

        #---------Guide-------------#
        self.xm_Guide = Concurrence(outcomes = ['succeeded','aborted'],
                                 default_outcome = 'aborted',
                                 outcome_map={'succeeded':{'NAV_GUIDE':'succeeded'},
                                              'aborted':{'SPEAK_FOLLOW':'aborted'},
                                              'aborted':{'SPEAK_FOLLOW':'error'}},
                                 child_termination_cb = self.child_cb_guide,
                                 input_keys = ['target','current_task'])   
        with self.xm_Guide:

            self.meta_nav_1 = StateMachine(outcomes = ['succeeded','aborted','error'],
                                        input_keys = ['target', 'current_task'])

            with self.meta_nav_1:

                self.meta_nav_1.userdata.pos_xm = Pose()
                self.meta_nav_1.userdata.turn_pose = Pose()
                self.meta_nav_1.userdata.target_mode = 1
                self.meta_nav_1.userdata.sentences = 'I have arrived here'

                StateMachine.add('GETTARGET',
                                GetTarget(),#修改(获取)userdata.current_target
                                transitions = {'succeeded':'GO','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task','current_target':'pos_xm','mode':'target_mode'}#相当于将current_target赋值给Pos_xm
                            )
                StateMachine.add('GO',
                                NavStack(),#以pos_xm为目标移动
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'pos_xm':'pos_xm'}
                            )

            self.guide_speak = StateMachine(outcomes = ['succeeded','aborted','error'],
                                        input_keys = ['target', 'current_task'])

            with self.guide_speak :
                
                self.guide_speak.userdata.sentences = 'Please follow me.'
                StateMachine.add('SPEAK',
                                    Speak_Guide(),#使用语音的服务，开一个子进程，播放音频
                                    transitions = {'succeeded':'succeeded','error':'error'},
                                    remapping = {'sentences':'sentences','target':'target','current_task':'current_task'}
                                ) 
                #rospy.sleep(1.0)   

            Concurrence.add('NAV_GUIDE',self.meta_nav_1)
            Concurrence.add('SPEAK_FOLLOW',self.guide_speak)

        #---------attend-----------#
        self.xm_Attend = StateMachine(outcomes = ['succeeded', 'aborted', 'error'],
                                input_keys = ['target','current_task'])
        with self.xm_Attend:
            self.xm_Attend.userdata.sentences = " may I get your attention please?"
            StateMachine.add('SPEAK',
                                SpeakAttend(),#使用语音的服务，开一个子进程，播放音频
                                transitions = {'succeeded':'succeeded','error':'error'},
                                remapping = {'sentences':'sentences','target':'target','current_task':'current_task'}
                            )
        #--------Introduce---------#
        self.xm_Introduce = StateMachine(outcomes = ['succeeded', 'aborted', 'error'],
                                input_keys = ['character','target','current_task'])
        with self.xm_Introduce:
            self.xm_Introduce.userdata.pos_xm  =Pose()
            self.xm_Introduce.userdata.sentences = ' I find you,and I will introduce that guy'
            self.xm_Introduce.userdata.rec = 2.0

            StateMachine.add('RUNNODE',
                                RunNode(),
                                transitions={'succeeded':'WAIT','aborted':'RUNNODE'})#On simulation , We need to skip RunNode ;So aborted -> Wait Or aborted->RunNode 
            #等待保证能够找到人
            StateMachine.add('WAIT',
                                Wait(),
                                transitions = {'succeeded':'GET_PEOPLE_POS','error':'error'},
                                remapping ={'rec':'rec'})
            #the data should be PointStamped() 
            
            StateMachine.add('GET_PEOPLE_POS',
                                FindPeople().find_people_,
                                transitions ={'invalid':'CLOSECAMERA','valid':'GET_PEOPLE_POS','preempted':'aborted'},
                                remapping = {'pos_xm':'pos_xm'}
                                )
            StateMachine.add('CLOSECAMERA',
                                     CloseCamera(),
                                     transitions ={'succeeded':'NAV_PEOPLE','aborted':'aborted'})
       
            StateMachine.add('NAV_PEOPLE',
                                NavStack(),
                                transitions = {'succeeded':'SPEAK','aborted':'NAV_PEOPLE','error':'error'},
                                remapping = {'pos_xm':'pos_xm'})
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions = {'succeeded':'SPEAK_CHARACTER','error':'error'},
                                remapping = {'sentences':'sentences'})
            StateMachine.add('SPEAK_CHARACTER',
                                Speak(),#使用语音的服务，开一个子进程，播放音频
                                transitions = {'succeeded':'succeeded','error':'error'},
                                remapping = {'sentences':'character'}
                            )

        
        #--------GO--------#
        self.xm_Nav = StateMachine(outcomes = ['succeeded', 'aborted', 'error'],
                                input_keys = ['target', 'current_task'])
        with self.xm_Nav:
            self.xm_Nav.userdata.pose_xm = Pose()
            self.xm_Nav.userdata.turn_pose = Pose()
            self.xm_Nav.userdata.target_mode = 1

            StateMachine.add('GETTARGET',
                                GetTarget(),#修改(获取)userdata.current_target
                                transitions = {'succeeded':'GO','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task','current_target':'pos_xm','mode':'target_mode'}#相当于将current_target赋值给Pos_xm
                            )
            StateMachine.add('GO',
                                NavStack(),#以pos_xm为目标移动
                                transitions = {'succeeded':'SPEAK','aborted':'SPEAK','error':'error'},
                                remapping = {'pos_xm':'pos_xm'}
                            )

            self.xm_Nav.userdata.sentences = 'I have arrived here'

            StateMachine.add('SPEAK',
                                Speak(),#使用语音的服务，开一个子进程，播放音频
                                transitions = {'succeeded':'succeeded','error':'error'},
                                remapping = {'sentences':'sentences'}
                            )
        #----FOLLOW-----#
        #xm_Follow包括两个并发状态，FOLLOW和STOP
        #STOP用于语音获取stop信号
        #FOLLOW包装了FindPeople().find_people_监视器和meta_nav
        #当找到人需要移动时，状态转移到meta_nav
        #meta_nav后,再继续进入监视状态
        #meta_nav是一个并发状态机，包装了NAV和WAIT
        #WAIT用于记录时间引发time_over
        #nav成功或中止后，与time_over一起进入继续找人状态
        
        self.xm_Follow = Concurrence(outcomes = ['succeeded','aborted'],
                                 default_outcome = 'aborted',
                                 outcome_map={'succeeded':{'STOP':'stop'},
                                              'aborted':{'FOLLOW':'aborted'},
                                              'aborted':{'FOLLOW':'preempted'}},
                                 child_termination_cb = self.child_cb)   
        with self.xm_Follow:

            self.meta_follow = StateMachine(['succeeded','aborted','preempted'])

            with self.meta_follow:

                self.meta_follow.userdata.pos_xm = Pose()
                self.meta_follow.userdata.rec = 0.2

                StateMachine.add('FIND',
                                    LegTracker0().tracker,#摄像头给订阅的话题发消息来跟踪人，在follow中
                                    transitions = {'invalid':'WAIT','valid':'FIND','preempted':'preempted'},
                            
                                    remapping={'pos_xm':'pos_xm'})
                StateMachine.add('WAIT' , 
                                    Wait(),#位于tool中，给rec赋值，沉睡0s
                                    transitions={'succeeded':'META_NAV' , 'error':'META_NAV'})
                
                self.meta_nav = Concurrence(outcomes=['time_over','get_pos','aborted'],
                                                default_outcome  = 'aborted',
                                                outcome_map={'time_over':{'WAIT':'succeeded'},
                                                             'get_pos':{'NAV':'succeeded'},
                                                             'aborted':{'NAV':'aborted'}},
                                                child_termination_cb=self.nav_child_cb,
                                                input_keys=['pos_xm'])
                with self.meta_nav:
                    print("8888888888888888888")
                    Concurrence.add('NAV',NavStack0(),remapping={'pos_xm':'pos_xm'})#跟随人
                    Concurrence.add('WAIT',Wait_trace())#以一定频率发移动指令
                StateMachine.add('META_NAV',
                                    self.meta_nav,
                                    transitions={'get_pos':'FIND','time_over':'FIND','aborted':'FIND'})
            Concurrence.add('FOLLOW',self.meta_follow)
            Concurrence.add('STOP',CheckStop())
            #Concurrence.add('RUNNODE',RunNode())#开启跟随人的图像节点            
        
        #----ANSWER----#
        #回答人的问题
        self.xm_Answer = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys = ['target','current_task'])
        with self.xm_Answer:

            self.xm_Answer.userdata.people_condition = list()
            self.xm_Answer.userdata.sentences = ""
            self.xm_Answer.userdata.target_mode = 0
            self.xm_Answer.userdata.sentences_ask_me = "please ask me"

            StateMachine.add('GETTARGET',
                                GetTarget(),
                                transitions = {'succeeded':'CHECKIF','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task','current_target':'current_target','mode':'target_mode'}
                            )
            StateMachine.add('CHECKIF',
                                    CheckIfTalk(),
                                    transitions = {'talk_answer':'SPEAKANSWER','ask':'SPEAK'},
                                    remapping = {'current_target':'current_target'})                            
            StateMachine.add('SPEAK',
                                    Speak(),#避免userdata.sentences异常，可以通过将getattr加入到Talk状态机中来取代这一步
                                    transitions = {'succeeded':'SPEAKANSWER','error':'error'},
                                    remapping = {'sentences':'sentences_ask_me'})

            #此处加一个语音识别，在识别服务中，语音可自己播放回复

            # StateMachine.add('TALK',
            #                         Talk(),
            #                         transitions = {'succeeded':'succeeded','error':'TALK'})
            StateMachine.add('SPEAKANSWER',
                                     SpeakAnswer(),#说出回答
                                     transitions = {'succeeded':'succeeded','error':'error'},
                                     remapping = {'sentences':'current_target'})

        #---------count---------#
        self.xm_Count = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['target','current_task','count_num','gesture'],
                                output_keys=['count_num'])
        
        with self.xm_Count:
            self.xm_Count.userdata.target_mode =0
            StateMachine.add('GETTARGET',
                                GetTarget(),
                                transitions = {'succeeded':'CHECK_NUM_OR_ANS','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task','current_target':'current_target','mode':'target_mode'}
                            )
            StateMachine.add('CHECK_NUM_OR_ANS',
                                    CheckPeopleOrObject(),
                                    transitions = {'people':'GET_PEOPLE_NUM','object':'GET_OBJECT_NUM'},
                                    remapping = {'current_target':'current_target'})
            StateMachine.add('GET_PEOPLE_NUM',
                                    GetPeopleNum(),
                                    transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                    remapping = {'count_num':'count_num','gesture':'gesture'}
                                    )
            StateMachine.add('GET_OBJECT_NUM',
                                    GetObjectNum(),
                                    transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                    remapping = {'count_num':'count_num','gesture':'gesture','current_task':'current_task'}
                                    )

        #----FIND----#
        #包含三个状态，由PERSON_OR_POS决定进入PERSON or POS
        #------<PERSON>-----#
        #PERSON：RUNNODE-->WAIT-->GET_PEOPLE_POS-->NAV-->SPEAK-->CLOSE_KINECT
        #运行图像节点，等待图像可用，然后利用find_people_监视器，nav到需要找的人面前，然后进行speak反馈，最后关闭摄像头
        #------<POS>------#
        #POS:GET_POS-->NAV-->GET_TARGET-->FIND_OBJECT-->GO-->FIND_OBJECT-->SPEAK
        #前一个任务的位置和需要find的小物体位置一样，得到位置后，nav到具体地点，然后获取物体名字，
        #打开摄像头寻找物体，获取具体信息后go，然后再找一个，speak反馈
        #------<CALLING>------#
        #找到打电话人的位置信息，移动后语音说明已找到，关闭摄像头
        self.xm_Find = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['target','current_task','character','gesture','objectName'],
                                output_keys=['character','objectName'])
        
        with self.xm_Find:
            #xm_Person--RUNNODE-->WAIT-->GET_PEOPLE_POS-->NAV-->SPEAK-->CLOSE_KINECT
            self.xm_Person = StateMachine(outcomes = ['succeeded','aborted','error'],
                                        input_keys=['character','gesture'],
                                        output_keys=['character','gesture'])
            with self.xm_Person:
                
                self.xm_Person.userdata.rec = 2.0
                #运行跟随人的图像节点
                # StateMachine.add('RUNNODE',
                #                     RunNode(),
                #                     transitions={'succeeded':'WAIT','aborted':'RUNNODE'})#On simulation , We need to skip RunNode ;So aborted -> Wait Or aborted->RunNode 
                #等待保证能够找到人
                
                StateMachine.add('WAIT',
                                    Wait(),
                                    transitions = {'succeeded':'GET_PEOPLE_POS','error':'error'},
                                    remapping ={'rec':'rec'})
                #the data should be PointStamped() 
                self.xm_Person.userdata.pos_xm  =Pose()
                
                #用FindPeople().find_people_监视器找人的位置
                StateMachine.add('GET_PEOPLE_POS',
                                    FindPeople_new(),
                                    transitions ={'succeeded':'NAV_PEOPLE','aborted':'GET_PEOPLE_POS','error':'error'},
                                    remapping = {'pos_xm':'pos_xm','character':'character','gesture':'gesture'}
                                    )  
             
                StateMachine.add('NAV_PEOPLE',
                                    NavStack(),
                                    transitions = {'succeeded':'SPEAK','aborted':'NAV_PEOPLE','error':'error'},
                                    remapping = {'pos_xm':'pos_xm'})
                self.xm_Person.userdata.sentences = 'I find you'
                StateMachine.add('SPEAK',
                                    Speak(),
                                    transitions = {'succeeded':'succeeded','error':'error'},
                                    remapping = {'sentences':'sentences'})

                # close the KinectV2
                # StateMachine.add('CLOSECAMERA',
                #                      CloseCamera(),
                #                      transitions ={'succeeded':'succeeded','aborted':'aborted'})
 

            #xm_Pos--GET_POS-->NAV-->GET_TARGET-->FIND_OBJECT-->GO-->FIND_OBJECT-->SPEAK-->CHECK_TURN-->TURN-->NAV
            self.xm_Pos = StateMachine(outcomes=['succeeded','aborted','error'],
                                    input_keys =['target','current_task','objectName'],
                                    output_keys=['objectName'])
            with self.xm_Pos:
                self.xm_Pos.userdata.pose = Pose()
                self.xm_Pos.userdata.mode_1 =1
                self.xm_Pos.userdata.nav_pos = gpsr_target['speaker']['pos']     
                #self.xm_Pos.userdata.objectName = ''
                self.xm_Pos.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
                self.xm_Pos.userdata.take_back_pose = [ [0,-1.4,3.14,0,0,0] ]
                self.xm_Pos.userdata.table_depth = 0.04
                self.xm_Pos.userdata.traget_size = [0.04, 0.065, 0.105]
                self.xm_Pos.userdata.distance = 0.3

                StateMachine.add('FIND_1',
                                GetObjectPosition_new(),#区分物体，识别物体
                                transitions={'succeeded': 'SPEAK', 'error': 'error'})

                # StateMachine.add('GETPICKPOSE',
                #                 GetPickPos(),
                #                 transitions ={'succeeded':'NAVTOPICKPOS','error':'error'})       
            
                # StateMachine.add('NAVTOPICKPOS',
                #                 NavStack(),
                #                 transitions ={'succeeded':'FIND_2','aborted':'NAVTOPICKPOS','error':'error'},
                #                 remapping ={"pos_xm":'pick_pos'})

                # StateMachine.add('FIND_2',
                #                 GetObjectPosition_new(),
                #                 transitions={'succeeded': 'TARGET_POINT_JUSFY', 'error': 'error'})   
                                                               
                # StateMachine.add('TARGET_POINT_JUSFY',
                #                 TargerPosReload(),
                #                 transitions={'succeeded': 'POINT', 'error': 'error'})

                # StateMachine.add('POINT', 
                #                 ArmStackPoint(),
                #                 transitions={'succeeded': 'SPEAK','error': 'error'})                                
                self.xm_Pos.userdata.sentences = 'I  have found '
                # self.xm_Pos.userdata.back_target = 'speaker'
                StateMachine.add('SPEAK',
                                    SpeakObjectName(),
                                    transitions = {'succeeded':'succeeded','error':'error'},
                                    remapping ={'sentences':'sentences','objectName':'objectName'})
                # StateMachine.add('BACK_POS',
                #                     ArmTrajectory(),
                #                     transitions={'succeeded': 'PLAT_COMMAND', 'error': 'error'},
                #                     remapping={'arm_waypoints':'take_back_pose'})
                # StateMachine.add('PLAT_COMMAND',
                #                     PlatCommond(),
                #                     transitions={'succeeded': 'succeeded', 'error': 'error'}
                #                     )
                                                        
            self.xm_Calling = StateMachine(outcomes=['succeeded','aborted','error'])
            with self.xm_Calling:
                self.xm_Calling.userdata.sentences = 'I have found you are calling.'
                StateMachine.add('GET_POS',
                                    GetCallingPos(),#获取打电话的人的位置信息
                                    remapping ={'pos_xm':'pos_xm'},
                                    transitions={'succeeded':'NAV_HEHE','aborted':'aborted','error':'error'})
                StateMachine.add('NAV_HEHE',
                                    NavStack(),
                                    remapping ={'pos_xm':'pos_xm'},
                                    transitions ={'succeeded':'SPEAK','aborted':'NAV_HEHE','error':'error'})
                
                StateMachine.add('SPEAK',
                                    Speak(),
                                    transitions = {'succeeded':'succeeded','error':'error'},
                                    remapping ={'sentences':'sentences'})

                # StateMachine.add('CLOSEKINECT',
                #                     CloseKinect(),
                #                     transitions ={'succeeded':'succeeded','aborted':'aborted'})

            
            
            #顶层状态机装配
            StateMachine.add('PERSON_OR_POS',
                                PersonOrPosition(),
                                transitions ={'person':'PERSON','position':'POS','calling':'CALLING'},
                                remapping = {'target':'target','current_task':'current_task'})
            StateMachine.add('PERSON',
                                self.xm_Person,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'character':'character','gesture':'gesture'})
            StateMachine.add('POS',
                                self.xm_Pos,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task'})
            StateMachine.add('CALLING',
                                self.xm_Calling,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})
        
        
        #-------PICK_UP-------#
        #RUNNODE_IMG-->GETNAME-->GET_POS-->NAV-->FIND-->POS_JUS-->NAV-->RUNNODE_IMG-->FIND-->POS_JUS-->PICK-->SPEAK
        #运行图像节点，获取物体名字和大物体位置后，nav到大物体位置，然后用图像找物体，然后用pos_jus得出适宜抓取的坐标，nav后继续循环一次
        #在此之后，进行抓取
        #如果上面的图像没有找到物体，则通过speak进行错误反馈
        self.xm_Pick_up = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['target','current_task'])    
        with self.xm_Pick_up: 
            self.xm_Pick_up.userdata.name =''
            self.xm_Pick_up.userdata.target_mode =0
            self.xm_Pick_up.userdata.objmode = -1
            self.xm_Pick_up.userdata.go_counter = 2
            self.xm_Pick_up.userdata.distance = 0.5 #change this element from 0.7 to 0.2
            self.xm_Pick_up.userdata.table_depth = 0.04
            self.xm_Pick_up.userdata.traget_size = [0.04, 0.065, 0.105]
            self.xm_Pick_up.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
            self.xm_Pick_up.userdata.take_back_pose = [ [0,0.026,3.14,0,3.14,0] ]
            self.xm_Pick_up.userdata.mode_1 =1
            self.xm_Pick_up.userdata.object_pos = PointStamped()
            
            #StateMachine.add('PICK_POS',
            #                     ArmTrajectory(),#轨迹分析
            #                     transitions={'succeeded': 'FIND_1', 'error': 'error'},
            #                     remapping={'arm_waypoints':'pick_pose'})                                   
            
            StateMachine.add('FIND_1',
                                GetObjectPosition(),#区分物体，识别物体
                                transitions={'succeeded': 'GETPICKPOSE', 'error': 'error'})

            StateMachine.add('GETPICKPOSE',
                                GetPickPos(),
                                transitions ={'succeeded':'NAVTOPICKPOS','error':'error'})       
            
            StateMachine.add('NAVTOPICKPOS',
                                NavStack(),
                                transitions ={'succeeded':'FIND_2','aborted':'NAVTOPICKPOS','error':'error'},
                                remapping ={"pos_xm":'pick_pos'})

            StateMachine.add('FIND_2',
                                GetObjectPosition(),
                                transitions={'succeeded': 'TARGET_POINT_JUSFY', 'error': 'error'})   
                                                               
            StateMachine.add('TARGET_POINT_JUSFY',
                                TargerPosReload(),
                                transitions={'succeeded': 'PICK', 'error': 'error'})

            StateMachine.add('PICK', 
                                ArmStack(),
                                transitions={'succeeded': 'NAV_BACK','error': 'error'})
            self.xm_Pick_up.userdata.distance = -0.2
            StateMachine.add('NAV_BACK', 
                                GoAhead(),
                                transitions={'succeeded': 'succeeded','error': 'error'},
                                remapping={'move_len':'distance'})
            
            #TODO arm back pose
            #StateMachine.add('BACK_POS',
            #                    ArmTrajectory(),
            #                    transitions={'succeeded': 'succeeded', 'error': 'error'},
            #                    remapping={'arm_waypoints':'take_back_pose'})

        #-------Put_down---------#
        self.xm_Put_down = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.xm_Put_down:    
            self.xm_Put_down.userdata.commond =0
            StateMachine.add('PLACE',
                                GripperCommond(),#控制爪子（打开）
                                transitions ={'succeeded':'succeeded','error':'error'})

         #顶层状态机
        self.xm_GPSR = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.xm_GPSR:

            self.xm_GPSR.userdata.target = list()
            self.xm_GPSR.userdata.action = list()
            self.xm_GPSR.userdata.task_num = 0
            self.xm_GPSR.userdata.answer = "this is the answer"
            self.xm_GPSR.userdata.current_task = -1
            self.xm_GPSR.userdata.current_turn = -1
            self.xm_GPSR.userdata.turn = 1              #the max_num of the mission
            self.xm_GPSR.userdata.pos_xm_door = gpsr_target['livingroom']['pos']
            self.xm_GPSR.userdata.waypoint = gpsr_target['dining_room_exit']['pos']
            self.xm_GPSR.userdata.sentences = 'give me the mission please' #for the Speak() state
            self.xm_GPSR.userdata.gripper_release = 0.0
            self.xm_GPSR.userdata.give_file = "give_me_the_mission.wav"
            self.xm_GPSR.userdata.sentences_release = "i will release the gripper,please grap it"
            self.xm_GPSR.userdata.rec = 5.0
            self.xm_GPSR.userdata.character = ''
            self.xm_GPSR.userdata.gesture = ''
            self.xm_GPSR.userdata.count_num = 0
            self.xm_GPSR.userdata.objectName = ''
            # StateMachine.add('ENTERROOM',
            #                      self.xm_EnterRoom,
            #                      transitions ={'succeeded':'RECEIVE_TASKS','aborted':'RECEIVE_TASKS','error':'error'})
            
            
            # StateMachine.add('SPEAK_RESTART',
            #                 SpeakWAV(),
            #                 transitions = {'succeeded':'RECEIVE_TASKS','aborted':'aborted','error':'error'},
            #                 remapping = {'filename':'give_file'}
            #                 )

            StateMachine.add('RECEIVE_TASKS',
                            GetTask(),
                            transitions = {'succeeded':'GET_NEXT_TASK','aborted':'RECEIVE_TASKS','error':'error'},
                            remapping = {'target':'target','action':'action','task_num':'task_num','answer':'answer','gesture':'gesture'}
                            )
            
            StateMachine.add('GET_NEXT_TASK',
                                NextDo(),
                                transitions = {'succeeded':'BACK_DOOR',
                                                'aborted':'GET_NEXT_TASK',
                                                'error':'error',
                                                'go':'GO',
                                                'find':'FIND',
                                                'answer':'ANSWER',
                                                'follow':'RUNNODE',#open the camera first
                                                'get':'PICK_UP',                                              
                                                'release':'RELEASE',
                                                'speak':'SPEAK_DES',
                                                'guide':'GUIDE',
                                                'introduce':'INTRODUCE',
                                                'count':'COUNT',
                                                'attend':'ATTEND'},
                                remapping = {'action':'action',
                                              'current_task':'current_task',
                                              'task_num':'task_num'}
                            )
            StateMachine.add('BACK_DOOR',
                                NavStack(),
                                transitions = {'succeeded':'CHECK_TURN','aborted':'BACK_DOOR','error':'error'},
                                remapping = {'pos_xm':'pos_xm_door'}
                            )

            # StateMachine.add('CHECK_TURN',
            #                     CheckTurn(),
            #                     transitions = {'succeeded':'GO_OUT','continue':'SPEAK_RESTART','error':'error'}
            #                 )
            StateMachine.add('CHECK_TURN',
                CheckTurn(),
                transitions = {'succeeded':'GO_OUT','continue':'RECEIVE_TASKS','error':'error'}
            )
            StateMachine.add('GO_OUT',
                                NavStack(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={"pos_xm":'waypoint'}
                            )
        
            #add all the task smach
            #-----------------#
            StateMachine.add('INTRODUCE',
                                self.xm_Introduce,
                                remapping ={'character':'character','target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'INTRODUCE'})
            StateMachine.add('ANSWER',
                                self.xm_Answer,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'ANSWER'})
            StateMachine.add('ATTEND',
                                self.xm_Attend,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'ATTEND'})
            StateMachine.add('GO',
                                self.xm_Nav,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'GO'})
            # StateMachine.add('CLOSE',
            #                     CloseKinect(),
            #                     transitions={'succeeded':'GET_NEXT_TASK','aborted':'GET_NEXT_TASK'})
            StateMachine.add('RUNNODE',
                                    RunNode(),
                                    transitions={'succeeded':'FOLLOW','aborted':'RUNNODE'})
            StateMachine.add('FIND',
                                self.xm_Find,
                                remapping ={'target':'target','current_task':'current_task','gesture':'gesture','character':'character'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'FIND'})
            StateMachine.add('FOLLOW',
                                self.xm_Follow,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'FOLLOW'})
            StateMachine.add('COUNT',
                                self.xm_Count,
                                remapping ={'target':'target','current_task':'current_task','count_num':'count_num','gesture':'gesture'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'FIND'})
            StateMachine.add('GUIDE',
                                self.xm_Guide,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'GUIDE'})
            StateMachine.add('PICK_UP',
                                self.xm_Pick_up,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'PICK_UP'})
            StateMachine.add('PUT_DOWN',
                                self.xm_Put_down,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'PUT_DOWN'}) 
            StateMachine.add('RELEASE_NAV',
                                NavStack(),
                                transitions={'succeeded':'SPEAK_RELEASE','aborted':'RELEASE','error':'RELEASE'},
                                remapping ={'pos_xm':'pos_xm_door'})
            StateMachine.add('SPEAK_RELEASE',
                                Speak(),
                                transitions = {'succeeded':'WAIT_RELEASE','error':'error'},
                                remapping ={'sentences':'sentences_release'})
            StateMachine.add('WAIT_RELEASE' , 
                                Wait(),
                                transitions={'succeeded':'RELEASE' , 'error':'RELEASE'})
            StateMachine.add('RELEASE',
                                GripperCommond(),
                                transitions={'succeeded':'GET_NEXT_TASK','error':'RELEASE'},
                                remapping={'gripper_release'})
            StateMachine.add('SPEAK_DES',
                                self.xm_Tell,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'GET_NEXT_TASK','error':'GET_NEXT_TASK'},
                                remapping={'answer':'answer','target':'target','current_task':'current_task','count_num':'count_num'})
        
        intro_server = IntrospectionServer('xm_gpsr',self.xm_GPSR,'/XM_ROOT')
        intro_server.start()
        out = self.xm_GPSR.execute()
        intro_server.stop()


    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

    def child_cb_guide(self,outcome_map):
        if outcome_map['NAV_GUIDE'] == 'succeeded':
            rospy.logwarn('---------I have arrive there---------')
            
            return True
        elif outcome_map['NAV_GUIDE'] == 'aborted':
            rospy.logerr('the stop state meet error!')
            return True
            
        if outcome_map['SPEAK_FOLLOW'] == 'aborted':
            rospy.logerr('the speak_follow state meet error!')
            return True

        return False

    #return True down
    def child_cb(self,outcome_map):
        if outcome_map['STOP'] == 'stop':
            rospy.logwarn('---------get the signal of stop,stop tracing---------')
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill','-9',pid],shell=True)
            return True
        elif outcome_map['STOP'] == 'aborted':
            rospy.logerr('the stop state meet error!')
            return True
            
        if outcome_map['FOLLOW']:
            rospy.logerr('the follow state meet error!')
            return True
        return False

    def nav_child_cb(self,outcome_map):
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
        
    # use for concurrence
if __name__ == "__main__":
    try:
        Gpsr()
    except:
        rospy.logerr(e)
        #print("7777777")
