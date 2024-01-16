#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from xm_smach.common_lib import *
from xm_smach.gpsr_lib import * 
from xm_smach.help_me_carry_lib import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from xm_smach.pick_turn import PickTurn , IsTurn

'''
导航到客厅中预定开始地点->获取要拿起的袋子->抓取->找到并面对操作员->交互告诉操作员开始跟随->跟随操作员（记忆）->将袋子递交操作员-》等待感谢-》返回竞机场
                                                                                ->获取终止信号 
                                                                                ->丢失叫操作员
'''


class CarryMyLuggage():
    def __init__(self):
        rospy.init_node('CarryMyLuggage_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to CarryMyLuggage!!!')
        self.smach_bool = False
        #add the function_smach here!
        #action状态机
        #---------------#
        #----GO-----#
        self.sm_Nav = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['target','current_task'])
        with self.sm_Nav:
            self.sm_Nav.userdata.pos_xm = Pose()
            self.sm_Nav.userdata.turn_pose = Pose()
            self.sm_Nav.userdata.target_mode = 1
            
            StateMachine.add('GETTARGET',
                                GetTarget(),
                                transitions = {'succeeded':'GO','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task','current_target':'pos_xm','mode':'target_mode'}
                            )
            StateMachine.add('GO',
                                NavStack(),
                                transitions = {'succeeded':'SPEAK','aborted':'GO','error':'error'},
                                remapping = {'pos_xm':'pos_xm'}
                            )
            self.sm_Nav.userdata.sentences = 'I have arrive here'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions = {'succeeded':'succeeded','error':'error'},
                                remapping = {'sentences':'sentences'}
                            )               
        #----FOLLOW-----#
        #sm_Follow包括两个并发状态，FOLLOW和STOP
        #STOP用于语音获取stop信号
        #FOLLOW包装了FindPeople().find_people_监视器和meta_nav
        #当找到人需要移动时，状态转移到meta_nav
        #meta_nav后,再继续进入监视状态
        #meta_nav是一个并发状态机，包装了NAV和WAIT
        #WAIT用于记录时间引发time_over
        #nav成功或中止后，与time_over一起进入继续找人状态
        self.sm_Follow = Concurrence(outcomes=['succeeded','aborted'],
                                        default_outcome = 'succeeded',
                                        outcome_map = {'succeeded':{'STOP':'stop'},
                                                        'aborted':{'FOLLOW':'aborted'}},
                                        child_termination_cb = self.child_cb)
        with self.sm_Follow:
            self.meta_follow = StateMachine(['succeeded','aborted','preempted'])
            with self.meta_follow:
                StateMachine.add('FIND',
                                    FindPeople().find_people_,
                                    transitions = {'invalid':'META_NAV','valid':'FIND','preempted':'preempted'},
                                    remapping = {'pos_xm':'pos_xm'}
                                )
                self.meta_nav = Concurrence(outcomes = ['time_over','get_pos','aborted'],
                                                default_outcome = 'aborted',
                                                outcome_map = {'time_over':{'WAIT':'succeeded'},
                                                               'get_pos':{'NAV':'succeeded'},
                                                               'aborted':{'NAV':'aborted'}},
                                                child_termination_cb=self.nav_child_cb,
                                                input_keys=['pos_xm']
                                            )
                with self.meta_nav:
                    Concurrence.add('NAV',NavStack(),remapping={'pos_xm':'pos_xm'})
                    Concurrence.add('WAIT',Wait_trace())
                StateMachine.add('META_NAV',
                                    self.meta_nav,
                                    transitions={'get_pos':'FIND','time_over':'FIND','aborted':'FIND'})
            Concurrence.add('FOLLOW',self.meta_follow)
            Concurrence.add('STOP',CheckStop())
            Concurrence.add('RUNNODE',RunNode())
        
        #----TALK----#
        #说出房间中人的状况
        self.sm_Talk = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_Talk:
            self.sm_Talk.userdata.people_condition = list()
            StateMachine.add('SPEAK',
                                Anwser(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted'})      
        #----FIND----#
        #包含三个状态，由PERSON_OR_POS决定进入PERSON or POS
        #------<PERSON>-----#
        #PERSON：RUNNODE-->WAIT-->GET_PEOPLE_POS-->NAV-->SPEAK-->CLOSE_KINECT
        #运行图像节点，等待图像可用，然后利用find_people_监视器，nav到需要找的人面前，然后进行speak反馈，最后关闭摄像头
        #------<POS>------#
        #POS:GET_POS-->NAV-->GET_TARGET-->FIND_OBJECT-->GO-->FIND_OBJECT-->SPEAK
        #前一个任务的位置和需要find的小物体位置一样，得到位置后，nav到具体地点，然后获取物体名字，
        #打开摄像头寻找物体，获取具体信息后go，然后再找一个，speak反馈
        self.sm_Find = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['target','current_task'])
        
        with self.sm_Find:
            #sm_Person--RUNNODE-->WAIT-->GET_PEOPLE_POS-->NAV-->SPEAK-->CLOSE_KINECT
            self.sm_Person = StateMachine(outcomes = ['succeeded','aborted','error'])
            with self.sm_Person:
                self.sm_Person.userdata.rec = 2.0
                #运行跟随人的图像节点
                StateMachine.add('RUNNODE',
                                    RunNode(),
                                    transitions={'succeeded':'WAIT','aborted':'RUNNODE'})#On simulation , We need to skip RunNode ;So aborted -> Wait Or aborted->RunNode 
                #等待保证能够找到人
                StateMachine.add('WAIT',
                                    Wait(),
                                    transitions = {'succeeded':'GET_PEOPLE_POS','error':'error'},
                                    remapping ={'rec':'rec'})
                #the data should be PointStamped() 
                self.sm_Person.userdata.pos_xm  =Pose()
                #用FindPeople().find_people_监视器找人的位置
                StateMachine.add('GET_PEOPLE_POS',
                                    FindPeople().find_people_,
                                    transitions ={'invalid':'NAV_PEOPLE','valid':'GET_PEOPLE_POS','preempted':'aborted'},
                                    remapping = {'pos_xm':'pos_xm'}
                                    )      
                StateMachine.add('NAV_PEOPLE',
                                    NavStack(),
                                    transitions = {'succeeded':'SPEAK','aborted':'NAV_PEOPLE','error':'error'},
                                    remapping = {'pos_xm':'pos_xm'})
                self.sm_Person.userdata.sentences = 'I find you'
                StateMachine.add('SPEAK',
                                    Speak(),
                                    transitions = {'succeeded':'CLOSEKINECT','error':'error'},
                                    remapping = {'sentences':'sentences'})

                # close the KinectV2
                StateMachine.add('CLOSEKINECT',
                                    CloseKinect(),
                                    transitions ={'succeeded':'succeeded','aborted':'aborted'})
 

            #sm_Pos--GET_POS-->NAV-->GET_TARGET-->FIND_OBJECT-->GO-->FIND_OBJECT-->SPEAK-->CHECK_TURN-->TURN-->NAV
            self.sm_Pos = StateMachine(outcomes=['succeeded','aborted','error'],
                                    input_keys =['target','current_task'])
            with self.sm_Pos:
                self.sm_Pos.userdata.pose = Pose()
                self.sm_Pos.userdata.mode_1 =1
                self.sm_Pos.userdata.nav_pos = gpsr_target['speaker']['pos']                                     
                StateMachine.add('GET_POS',
                                    GetPos(),
                                    remapping ={'target':'target','current_task':'current_task','pose':'pose','mode':'mode_1'},
                                    transitions={'succeeded':'NAV_HEHE','aborted':'aborted','error':'error'})

                StateMachine.add('NAV_HEHE',
                                    NavStack(),
                                    remapping ={'pos_xm':'pose'},
                                    transitions ={'succeeded':'GET_TARGET','aborted':'NAV_HEHE','error':'error'})

                self.sm_Pos.userdata.target_mode = 0
                self.sm_Pos.userdata.name = ''
                StateMachine.add('GET_TARGET',
                                    GetTarget(),
                                    remapping ={'target':'target','current_task':'current_task','mode':'target_mode','current_target':'name'},
                                    transitions ={'succeeded':'FIND_OBJECT','aborted':'aborted','error':'error'})
                
                self.sm_Pos.userdata.object_pos = PointStamped()
                StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                transitions ={'succeeded':'SPEAK','aborted':'SPEAK','error':'error'},
                                remapping ={'name':'name','object_pos':'object_pos'}) 
                
                StateMachine.add('GO' , 
                                    GoAhead(),
                                    transitions = {'succeeded':'FIND_OBJECT_1' , 'aborted':'FIND_OBJECT_1' , 'error':'error'})

                self.sm_Pos.userdata.object_pos = PointStamped()
                StateMachine.add('FIND_OBJECT_1',
                                FindObject(),
                                transitions ={'succeeded':'SPEAK','aborted':'SPEAK','error':'error'},
                                remapping ={'name':'name','object_pos':'object_pos'}) 

                self.sm_Pos.userdata.sentences = 'I find it'
                self.sm_Pos.userdata.back_target = 'speaker'
                StateMachine.add('SPEAK',
                                    Speak(),
                                    transitions = {'succeeded':'succeeded','error':'error'},
                                    remapping ={'sentences':'sentences'})

    
            #顶层状态机装配
            StateMachine.add('PERSON_OR_POS',
                                PersonOrPosition(),
                                transitions ={'person':'PERSON','position':'POS','error':'error'},
                                remapping = {'target':'target','current_task':'current_task'})
            StateMachine.add('PERSON',
                                self.sm_Person,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})
            StateMachine.add('POS',
                                self.sm_Pos,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task'})
        #-------PICK_UP-------#
        #RUNNODE_IMG-->GETNAME-->GET_POS-->NAV-->FIND-->POS_JUS-->NAV-->RUNNODE_IMG-->FIND-->POS_JUS-->PICK-->SPEAK
        #运行图像节点，获取物体名字和大物体位置后，nav到大物体位置，然后用图像找物体，然后用pos_jus得出适宜抓取的坐标，nav后继续循环一次
        #在此之后，进行抓取
        #如果上面的图像没有找到物体，则通过speak进行错误反馈
        self.sm_Pick_up = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['target','current_task'])    
        with self.sm_Pick_up: 
            self.sm_Pick_up.userdata.name =''
            self.sm_Pick_up.userdata.target_mode =0
            self.sm_Pick_up.userdata.objmode = -1
            self.sm_Pick_up.userdata.go_counter = 2

            StateMachine.add('RUNNODE_IMG',
                                RunNode_img(),
                                transitions = {'succeeded':'GETNAME','aborted':'RUNNODE_IMG'})
            StateMachine.add('GETNAME',
                                GetTarget(),
                                remapping ={'target':'target','current_task':'current_task','mode':'target_mode','current_target':'name'},
                                transitions={'succeeded':'GET_POS','aborted':'aborted','error':'error'})
            self.sm_Pick_up.userdata.mode_1 =1
            StateMachine.add('GET_POS',
                                GetPos(),
                                remapping ={'target':'target','current_task':'current_task','pose':'pose','mode':'mode_1'},
                                transitions={'succeeded':'NAV_HEHE','aborted':'aborted','error':'error'})
            StateMachine.add('NAV_HEHE',
                                NavStack(),
                                remapping ={'pos_xm':'pose'},
                                transitions ={'succeeded':'FIND_OBJECT','aborted':'NAV_HEHE','error':'error'})

            self.sm_Pick_up.userdata.object_pos = PointStamped()
            StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                transitions ={'succeeded':'POS_JUSTFY','aborted':'succeeded','error':'SPEAK'},
                                remapping ={'name':'name','object_pos':'object_pos','objmode':'objmode'})
            self.sm_Pick_up.userdata.pose = Pose()
            StateMachine.add('POS_JUSTFY',
                                PosJustfy(),
                                remapping={'object_pos':'object_pos','pose':'pose'},
                                transitions={'succeeded':'NAV_TO','aborted':'aborted','error':'error'})
            StateMachine.add('NAV_TO',
                                NavStack(),
                                transitions ={'succeeded':'RUNNODE_IMG2','aborted':'NAV_TO','error':'error'},
                                remapping ={"pos_xm":'pose'})
            StateMachine.add('RUNNODE_IMG2',
                                RunNode_img(),
                                transitions = {'succeeded':'FIND_AGAIN','aborted':'RUNNODE_IMG2'})                    
            StateMachine.add('FIND_AGAIN',
                                FindObject(),
                                transitions ={'succeeded':'PICK_JUS','aborted':'succeeded','error':'error'},
                                remapping ={'name':'name','object_pos':'object_pos','objmode':'objmode'})
            StateMachine.add('PICK_JUS' , 
                                PickJustfy(),
                                transitions = {'succeeded':'PICK','error':'error'},
                                remapping = {'name':'name',
                                            'object_pos':'object_pos'})
            self.sm_Pick_up.userdata.arm_mode_1 =1
            StateMachine.add('PICK',
                                ArmCmd(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={'arm_ps':'object_pos','mode':'arm_mode_1'})
            self.sm_Pick_up.userdata.sentences = 'xiao meng can not find the thing'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})

        

        #-------Pick_down---------#
        self.sm_Put_down = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_Put_down:
            # place_ps please specified due to the scene
            self.sm_Put_down.userdata.place_ps = PointStamped()
            self.sm_Put_down.userdata.place_ps.header.frame_id ='base_link'
            self.sm_Put_down.userdata.place_ps.point.x =0.8
            self.sm_Put_down.userdata.place_ps.point.y =0.0
            self.sm_Put_down.userdata.place_ps.point.z =0.6 
            self.sm_Put_down.userdata.objmode = 2
            # without moveit, if is just place it in a open space
            self.sm_Put_down.userdata.arm_mode_1 =2         
            StateMachine.add('PLACE',
                                ArmCmd(),
                                transitions ={'succeeded':'succeeded','aborted':'PLACE','error':'error'},
                                remapping ={'arm_ps':'place_ps','mode':'arm_mode_1'})
        

        
        '''
        上层为GPSR通用状态机
        '''
        #顶层状态机
        #HELP
        self.sm_HELP = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_HELP:
            self.sm_HELP.userdata.target = list()
            self.sm_HELP.userdata.action = list()
            self.sm_HELP.userdata.start_pos = gpsr_target['speaker']['pos']#xm开始的预定位置
            self.sm_HELP.userdata.sentences = 'which luggage you want' #for the Speak() state
            self.sm_HELP.userdata.start_follow = 'we can go to the car'
            
            StateMachine.add('START',
                            NavStack(),
                            transitions ={'succeeded':'SPEAK','aborted':'aborted','error':'error'},
                            remapping ={"pos_xm":'start_pos'}
                            )
            StateMachine.add('SPEAK',
                            Speak(),
                            transitions = {'succeeded':'GET_TARGET','aborted':'aborted','error':'error'},
                            remapping = {'sentences':'sentences'}
                            )

            StateMachine.add('GET_TARGET',
                            GetTask(),
                            transitions = {'succeeded':'PICK_UP','aborted':'GET_TARGET','error':'error'},
                            remapping = {'target':'target','action':'action','task_num':'task_num'}
                            )

            StateMachine.add('PICK_UP',
                                self.sm_Pick_up,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'FIND','aborted':'PICK_UP'})
            StateMachine.add('FIND',
                                self.sm_Find,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'START_FOLLOW','aborted':'FIND'})            
            
            StateMachine.add('START_FOLLOW',
                            Speak(),
                            transitions = {'succeeded':'FOLLOW','aborted':'aborted','error':'error'},
                            remapping = {'sentences':'start_follow'}
                            )
                            
            StateMachine.add('FOLLOW',
                                self.sm_Follow,
                                transitions={'succeeded':'RELEASE','aborted':'FOLLOW'})

            StateMachine.add('RELEASE',
                                Release(),
                                transitions={'succeeded':'BACK','aborted':'RELEASE'})
            
            StateMachine.add('BACK',
                            NavStack(),
                            transitions ={'succeeded':'SPEAK','aborted':'aborted','error':'error'},
                            remapping ={"pos_xm":'start_pos'}
                            )            
            
        intro_server = IntrospectionServer('sm_HELP',self.sm_HELP,'/SM_ROOT')
        intro_server.start()
        out = self.sm_HELP.execute()
        intro_server.stop()

    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

    #return True down
    def child_cb(self,outcome_map):
        if outcome_map['STOP'] == 'stop':
            rospy.logwarn('---------get the signal of stop,stop tracing---------')
            pip = get_pid("people_tracking")
            subprocess.call('kill'+str(pip[0]),shell=True)
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
            print outcome_map
            return False
        
    # use for concurrence
if __name__ == "__main__":
    try:
        CarryMyLuggage()
    except Exception,e:
        rospy.logerr(e)   
