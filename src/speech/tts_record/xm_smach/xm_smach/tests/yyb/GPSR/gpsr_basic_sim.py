#!/usr/bin/env python
# encoding:utf8
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
#对于顶层状态机GPSR，
#SPEAK-->LISTEN-->GET_TASK-->ACTION-->BACK-->CHECK_TURN-->BYE
#ACTION-->GET_TASK
#CHECK_TURN-->LISTEN
#通过speak进行交互，然后listen接受一组命令，在这一组命令中，通过GET_TASK分解任务，分解给各种action执行
#一组命令结束后，回到讲话人附近，判断是否完成所有循环，如果完成，BYEBYE

#PLACE=GO+PICK_UP+GO+PICK_DOWN
#PICK=GO+PICK_UP
#GO:
#TALK
#FIND:前提是find的目标在附近
#FOLLOW：前提是follow的目标在前方

class Talk(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'])
        self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)

    def execute(self, userdata):
        try:
            getattr(userdata, 'sentences')
        except:
            rospy.logerr('No param specified ')
            return 'error'
        else：

            rospy.loginfo('.................Speak ^_^..........\n')
            rospy.logwarn(userdata.sentences)
            try:
                res = self.speak_client.call(command = 2)
                self.string_ = res.target[0]

            except:
                rospy.logerr('get wrong when transfrom the sentences') 
                return 'error'
            else:
                try:
                    # subprocess - Subprocesses with accessible I/O streams
                    # 创建一个子进程,父进程等待子进程完成,返回退出信息
                    # 让xm在子进程中说出userdata.sentences的语句，不影响父进程
                    self.speak_client.wait_for_service(timeout=10.0)
                    self.speak_client.call(self.string_)

                    rospy.sleep(2.0)

                    speech_bool = self.speak_client.call(self.string_)
                    if speech_bool.flag == 1:
                        subprocess.call(["play", "tts_sample.wav"])
                    elif speech_bool.flag == 0:
                        subprocess.call("espeak -vf5 -s 75 '%(a)s'" %
                                        {'a': str(self.string_)}, shell=True)
                    else:
                        rospy.logerr('the response error')
                        return 'error'
                except:
                    return 'aborted'
                else:
                    return 'succeeded'

class CheckIfTalk(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['talk_answer', 'ask'],
                       input_keys = ['current_target'])
    def execute(self, userdata):
        if userdata.current_target == "question" or userdata.current_target == "":
            return 'ask'
        else:
            return 'talk_answer'

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



class GetCallingPos(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted','error'],
                       output_keys = ['pos_xm'])
        self.tf_listener = tf.TransformListener()
        self.ps = Pose()
    def execute(self, userdata):
        try:
            subprocess.call("xterm -e python3 Vision/Function/Phone.py &",shell=True)
            rospy.sleep(2.0)
        except:
            rospy.logerr('No param specified')
            return 'error'
        try:
            filename = '/home/domistic/Vision/TXT/calling.txt'
            information = ""
            with open(filename) as f:
                information = f.readlines()
                
                information = information[0].rstrip('\n')
            pos_list = information.split(" ")

            pos = PointStamped()
            pos.point.x = float(pos_list[0])
            pos.point.y = float(pos_list[1])
            pos.point.z = float(pos_list[2])

            userdata.pos_xm = self.data_deal(pos,1.0)

            return 'succeeded'
        except:
            return 'error'
        return 'succeeded'

    def data_deal(self,pos_xm,distance):
        '''
        获得距离distance距离的位置，修改pos_xm和self.ps
        '''
        # the simple deal for data from the cv
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y
        angle = math.atan2(person_y, person_x)
        person_x = person_x - distance*math.cos(angle)
        person_y = person_y - distance*math.sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0
        new_header =Header()
        new_header.frame_id = 'kinect2_rgb_link'
        pos_xm.header = new_header
        # change 
        q_angle = quaternion_from_euler(0,0,angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header  =pos_xm.header
        qs.quaternion = self.q
      
   
        self.tf_listener.waitForTransform('map','kinect2_rgb_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    
        

        pos_xm =self.tf_listener.transformPoint('map',pos_xm)
        rospy.logwarn('tf point succeeded ')    

        #修改self.ps
        qs =self.tf_listener.transformQuaternion('map',qs)
        rospy.logwarn('tf quaternion succeeded ')    
        self.ps = Pose(pos_xm.point,qs.quaternion)
        self.ps.position.z = 0

        return self.ps


class Gpsr():
    def __init__(self):
        rospy.init_node('Gpsr_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to Gpsr!!!')
        self.smach_bool = False
        #add the function_smach here!
        #action状态机
        #---------------#
        #----GO-----#
        self.sm_Nav = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['target','current_task'])
        with self.sm_Nav:
            self.sm_Nav.userdata.pos_xm = Pose()#位于geometry_msgs.msg._Pose，返回xm位姿信息（位置和方向）
            self.sm_Nav.userdata.turn_pose = Pose()
            self.sm_Nav.userdata.target_mode = 1
            
            StateMachine.add('GETTARGET',
                                GetTarget(),#修改(获取)userdata.current_target
                                transitions = {'succeeded':'GO','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task','current_target':'pos_xm','mode':'target_mode'}#相当于将current_target赋值给Pos_xm
                            )
            StateMachine.add('GO',
                                NavStack(),#以pos_xm为目标移动
                                transitions = {'succeeded':'SPEAK','aborted':'GO','error':'error'},
                                remapping = {'pos_xm':'pos_xm'}
                            )
            self.sm_Nav.userdata.sentences = 'I have arrive here'
            StateMachine.add('SPEAK',
                                Speak(),#使用语音的服务，开一个子进程，播放音频
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


        self.sm_Follow = Concurrence(outcomes = ['succeeded','aborted'],
                                 default_outcome = 'aborted',
                                 outcome_map={'succeeded':{'STOP':'stop'},
                                              'aborted':{'FOLLOW':'aborted'},
                                              'aborted':{'FOLLOW':'preempted'}},
                                 child_termination_cb = self.child_cb)
        with self.sm_Follow:
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
                    Concurrence.add('NAV',NavStack0(),remapping={'pos_xm':'pos_xm'})#跟随人
                    Concurrence.add('WAIT',Wait_trace())#以一定频率发移动指令
                StateMachine.add('META_NAV',
                                    self.meta_nav,
                                    transitions={'get_pos':'FIND','time_over':'FIND','aborted':'FIND'})
            Concurrence.add('FOLLOW',self.meta_follow)
            Concurrence.add('STOP',CheckStop())
            Concurrence.add('RUNNODE',RunNode())#开启跟随人的图像节点
      
        #----TALK----#
        #请人布置任务
        self.sm_Talk = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys = ['target','current_task'])
        with self.sm_Talk:
            self.sm_Talk.userdata.people_condition = list()
            self.sm_Talk.userdata.sentences = ""
            self.sm_Talk.userdata.target_mode = 0
            self.sm_Talk.userdata.sentences_ask_me = "please ask me"
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
                                    transitions = {'succeeded':'TALK','error':'error'},
                                    remapping = {'sentences':'sentences_ask_me'})
            StateMachine.add('TALK',
                                    Talk(),
                                    transitions = {'succeeded':'succeeded','error':'TALK'})
            StateMachine.add('SPEAKANSWER',
                                    Speak(),#说出问题
                                    transitions = {'succeeded':'succeeded','error':'error'},
                                    remapping = {'sentences':'current_target'})
        
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

            self.sm_Calling = StateMachine(outcomes=['succeeded','aborted','error'])
            with self.sm_Calling:
                self.sm_Calling.userdata.sentences = 'I have found you are calling.'
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

            #顶层状态机装配
            StateMachine.add('PERSON_OR_POS',
                                PersonOrPosition(),
                                transitions ={'person':'PERSON','position':'POS','calling':'CALLING'},
                                remapping = {'target':'target','current_task':'current_task'})
            StateMachine.add('PERSON',
                                self.sm_Person,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})
            StateMachine.add('POS',
                                self.sm_Pos,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task'})
            StateMachine.add('CALLING',
                                self.sm_Calling,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})
        
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
            self.sm_Pick_up.userdata.distance = 0.7
            self.sm_Pick_up.userdata.table_depth = 0.04
            self.sm_Pick_up.userdata.traget_size = [0.04, 0.065, 0.105]
            self.sm_Pick_up.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
            self.sm_Pick_up.userdata.take_back_pose = [ [0,0.026,3.14,0,3.14,0] ]
            self.sm_Pick_up.userdata.mode_1 =1
            self.sm_Pick_up.userdata.object_pos = PointStamped()
            
            StateMachine.add('PICK_POS',
                                ArmTrajectory(),#轨迹分析
                                transitions={'succeeded': 'FIND_1', 'error': 'error'},
                                remapping={'arm_waypoints':'pick_pose'})                                   
            
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
            
            StateMachine.add('NAV_BACK', 
                                GoAhead(),
                                transitions={'succeeded': 'BACK_POS','error': 'error'},
                                remapping={'move_len':'back_distance'})
            
            #TODO arm back pose
            StateMachine.add('BACK_POS',
                                ArmTrajectory(),
                                transitions={'succeeded': 'succeeded', 'error': 'error'},
                                remapping={'arm_waypoints':'take_back_pose'}) 

        

        #-------Put_down---------#
        self.sm_Put_down = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.sm_Put_down:    
            self.sm_Put_down.userdata.commond =0
            StateMachine.add('PLACE',
                                GripperCommond(),#控制爪子（打开）
                                transitions ={'succeeded':'succeeded','error':'error'})
        

        #-------Enter_Room-------#
        self.sm_EnterRoom = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_EnterRoom:
            self.sm_EnterRoom.userdata.start_waypoint = gpsr_target['livingroom']['pos']
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
                                transitions={'succeeded':'succeeded','aborted':'NAV','error':'error'},
                                remapping = {'pos_xm':'start_waypoint'})
            


        #顶层状态机
        self.sm_GPSR = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_GPSR:
            self.sm_GPSR.userdata.target = list()
            self.sm_GPSR.userdata.action = list()
            self.sm_GPSR.userdata.task_num = 0
            self.sm_GPSR.userdata.current_task = -1
            self.sm_GPSR.userdata.current_turn = -1
            self.sm_GPSR.userdata.turn = 3              #the max_num of the mission
            self.sm_GPSR.userdata.pos_xm_door = gpsr_target['livingroom']['pos']
            self.sm_GPSR.userdata.waypoint = gpsr_target['exit_pos']['pos']
            self.sm_GPSR.userdata.sentences = 'give me the mission please' #for the Speak() state
            self.sm_GPSR.userdata.gripper_release = 0.0
            self.sm_GPSR.userdata.give_file = "give_me_the_mission.wav"
            self.sm_GPSR.userdata.smsentences_release = "i will release the gripper,please grap it"
            self.sm_GPSR.userdata.rec = 5.0
            StateMachine.add('ENTERROOM',
                                self.sm_EnterRoom,
                                transitions ={'succeeded':'SPEAK_RESTART','aborted':'SPEAK_RESTART','error':'error'})
            
            
            StateMachine.add('SPEAK_RESTART',
                            SpeakWAV(),
                            transitions = {'succeeded':'RECEIVE_TASKS','aborted':'aborted','error':'error'},
                            remapping = {'filename':'give_file'}
                            )

            StateMachine.add('RECEIVE_TASKS',
                            GetTask(),
                            transitions = {'succeeded':'GET_NEXT_TASK','aborted':'RECEIVE_TASKS','error':'error'},
                            remapping = {'target':'target','action':'action','task_num':'task_num'}
                            )
            
            StateMachine.add('GET_NEXT_TASK',
                                NextDo(),
                                transitions = {'succeeded':'BACK_DOOR',
                                                'aborted':'aborted',
                                                'error':'error',
                                                'go':'GO',
                                                'find':'FIND',
                                                'tell':'TALK',
                                                'follow':'FOLLOW',
                                                'get':'PICK_UP',
                                                'deliver':'PUT_DOWN'},
                                remapping = {'action':'action',
                                              'current_task':'current_task',
                                              'task_num':'task_num'}
                            )
            StateMachine.add('BACK_DOOR',
                                NavStack(),
                                transitions = {'succeeded':'CHECK_TURN','aborted':'BACK_DOOR','error':'error'},
                                remapping = {'pos_xm':'pos_xm_door'}
                            )

            StateMachine.add('CHECK_TURN',
                                CheckTurn(),
                                transitions = {'succeeded':'GO_OUT','continue':'SPEAK_RESTART','error':'error'}
                            )
            StateMachine.add('GO_OUT',
                                NavStack(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={"pos_xm":'waypoint'}
                            )
        
            #add all the task smach
            #-----------------#
            StateMachine.add('TALK',
                                self.sm_Talk,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'TALK'})
            StateMachine.add('GO',
                                self.sm_Nav,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'GO'})
            StateMachine.add('FIND',
                                self.sm_Find,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'FIND'})
            StateMachine.add('FOLLOW',
                                self.sm_Follow,
                                transitions={'succeeded':'CLOSE','aborted':'FOLLOW'})
            #跟随人结束后关闭摄像头
            StateMachine.add('CLOSE',
                                CloseKinect(),
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'GET_NEXT_TASK'})

            StateMachine.add('PICK_UP',
                                self.sm_Pick_up,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'PICK_UP'})
            StateMachine.add('PUT_DOWN',
                                self.sm_Put_down,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'PUT_DOWN'}) 
            
            # StateMachine.add('RELEASE_NAV',
            #                     NavStack(),
            #                     transitions={'succeeded':'SPEAK_RELEASE','aborted':'RELEASE','error':'RELEASE'},
            #                     remapping ={'pos_xm':'pos_xm_door'})
            # StateMachine.add('SPEAK_RELEASE',
            #                     Speak(),
            #                     transitions = {'succeeded':'WAIT_RELEASE','error':'error'},
            #                     remapping ={'sentences':'sentences_release'})
            # StateMachine.add('WAIT_RELEASE' , 
            #                     Wait(),
            #                     transitions={'succeeded':'RELEASE' , 'error':'RELEASE'})
            # StateMachine.add('RELEASE',
            #                     GripperCommond(),
            #                     transitions={'succeeded':'GET_NEXT_TASK','error':'RELEASE'},
            #                     remapping={'gripper_release'})
            
            
            
        
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