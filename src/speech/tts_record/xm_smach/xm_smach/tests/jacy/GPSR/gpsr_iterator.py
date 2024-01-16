#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import StateMachine, Iterator
from smach_ros import IntrospectionServer
from xm_smach.common_lib import *
from xm_smach.gpsr_lib import * 
from xm_smach.help_me_carry_lib import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from xm_smach.pick_turn import PickTurn , IsTurn

#对于顶层状态机GPSR，
#(SPEAK-->LISTEN-->GET_TASK-->ACTION-->BACK)*3-->BYE
#(SPEAK-->LISTEN-->GET_TASK-->ACTION-->BACK)被分装成ITERATOR容器，通过参数服务器设置循环次数
#事实上，在single_mission里面，也包含了不止一个单一的更小的原子任务，这些任务通过GET_TASK获取并判断是否完成一个single_mission
#ACTION-->GET_TASK
#通过speak进行交互，然后listen接受一组命令，在这一组命令中，通过GET_TASK分解任务，分解给各种action执行
#一组命令结束后，回到讲话人附近，判断是否完成所有循环，如果完成，BYEBYE

#PLACE=GO+PICK_UP+GO+PICK_DOWN
#PICK=GO+PICK_UP
#GO:
#TALK
#FIND:前提是find的目标在附近
#FOLLOW：前提是follow的目标在前方



class Gpsr():
    def __init__(self):
        rospy.init_node('Gpsr_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to Gpsr!!!')
        self.smach_bool = False
        #参数服务器设置循环次数
        self.n_count = rospy.get_param("~n_count", 1)
        #self.n_count = rospy.get_param("~n_count", 3)
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
                                remapping = {'target':'target','current_task':'pos_xm','mode':'target_mode'}
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
        

        #顶层状态机
        self.sm_GPSR = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_GPSR:
            #任务循环体
            self.sm_mission_iterator = Iterator(outcomes = ['succeeded','aborted','error'],
                                          input_keys = [],
                                          it = lambda: range(0, self.n_count),
                                          output_keys = [],
                                          it_label = 'index',
                                          exhausted_outcome = 'succeeded')
            with self.sm_mission_iterator:
                
                self.sm_single_mission = StateMachine(outcomes=['succeeded','aborted','error','continue'])
                
                with self.sm_single_mission:
                    #为了配合循环容易，再单个任务里面，必须保证最终结果只能是continue
                    self.sm_single_mission.userdata.target = list()
                    self.sm_single_mission.userdata.action = list()
                    self.sm_single_mission.userdata.task_num = 0
                    self.sm_single_mission.userdata.current_task = -1
                    self.sm_single_mission.userdata.turn = 3              #the max_num of the mission
                    self.sm_single_mission.userdata.pos_xm_door = gpsr_target['speaker']['pos']
                    self.sm_single_mission.userdata.sentences = 'give me the mission please' #for the Speak() state
                    StateMachine.add('SPEAK_RESTART',
                                    Speak(),
                                    transitions = {'succeeded':'RECEIVE_TASKS','aborted':'RECEIVE_TASKS','error':'BACK_DOOR'},
                                    remapping = {'sentences':'sentences'}
                                    )

                    StateMachine.add('RECEIVE_TASKS',
                                    GetTask(),
                                    transitions = {'succeeded':'RECEIVE_TASKS','aborted':'RECEIVE_TASKS','error':'BACK_DOOR'},
                                    remapping = {'target':'target','action':'action','task_num':'task_num'}
                                    )
                    StateMachine.add('GET_NEXT_TASK',
                                        NextDo(),
                                        transitions = {'succeeded':'BACK_DOOR',
                                                        'aborted':'BACK_DOOR',
                                                        'error':'BACK_DOOR',
                                                        'go':'GO',
                                                        'find':'FIND',
                                                        'talk':'TALK',
                                                        'follow':'FOLLOW',
                                                        'pick_up':'PICK_UP',
                                                        'put_down':'PUT_DOWN'},
                                        remapping = {'action':'action',
                                                    'current_task':'current_task',
                                                    'task_num':'task_num'}
                                    )
                    StateMachine.add('BACK_DOOR',
                                        NavStack(),
                                        transitions = {'succeeded':'continue','aborted':'continue','error':'continue'},
                                        remapping = {'pos_xm':'pos_xm_door'}
                                    )
                    rospy.logwarn("Completed turn number: " + str(*))    
                    #add all the task smach
                    #-----------------#
                    StateMachine.add('TALK',
                                        self.sm_Talk,
                                        transitions={'succeeded':'GET_NEXT_TASK','aborted':'TALK'})
                    StateMachine.add('GO',
                                        self.sm_Nav,
                                        remapping ={'target':'target','current_task':'current_task'},
                                        transitions={'succeeded':'GET_NEXT_TASK','aborted':'GO'})
                    StateMachine.add('CLOSE',
                                        CloseKinect(),
                                        transitions={'succeeded':'GET_NEXT_TASK','aborted':'GET_NEXT_TASK'})
                    StateMachine.add('FIND',
                                        self.sm_Find,
                                        remapping ={'target':'target','current_task':'current_task'},
                                        transitions={'succeeded':'GET_NEXT_TASK','aborted':'FIND'})
                    StateMachine.add('FOLLOW',
                                        self.sm_Follow,
                                        transitions={'succeeded':'CLOSE','aborted':'FOLLOW'})
                    StateMachine.add('PICK_UP',
                                        self.sm_Pick_up,
                                        remapping ={'target':'target','current_task':'current_task'},
                                        transitions={'succeeded':'GET_NEXT_TASK','aborted':'PICK_UP'})
                    StateMachine.add('PUT_DOWN',
                                        self.sm_Put_down,
                                        transitions={'succeeded':'GET_NEXT_TASK','aborted':'PUT_DOWN'}) 
                

                #把sm_single_mission添加到循环体,现在sm_mission_iterator已装配完毕
                Iterator.set_contained_state('SINGLE_MISSION', self.sm_single_mission, loop_outcomes=['continue'])
            
            #它添加到顶层状态机
            StateMachine.add('MISSION_ITERATOR', self.sm_mission_iterator, {'succeeded':'GO_OUT', 'aborted':'GO_OUT'})

            #循环结束后退出竞技场
            StateMachine.add('GO_OUT',
                                NavStack(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={"pos_xm":'waypoint'}
                            )
        
        intro_server = IntrospectionServer('sm_gpsr',self.sm_GPSR,'/SM_ROOT')
        intro_server.start()
        out = self.sm_GPSR.execute()
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
        Gpsr()
    except Exception,e:
        rospy.logerr(e)   
