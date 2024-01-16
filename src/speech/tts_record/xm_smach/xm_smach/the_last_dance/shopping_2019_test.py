#!/usr/bin/env python
# encoding:utf-8
#请求跟随->跟随的同时语音识别->识别到后记录位置和物品->继续跟随->语音识别到结束后停止->语音识别物品名->
import rospy
from smach_ros import IntrospectionServer
from xm_smach.common_lib import *
from xm_smach.shopping_lib import * 
from xm_smach.target_gpsr import *
from smach import State, StateMachine, Concurrence
from math import pi
from geometry_msgs.msg import *
import subprocess
from new_lib.special import ShoppingGetTask,ShoppingNextTask

def get_pid(name):
    return map(int,subprocess.check_output(["pidof",name]).split())
class Shopping():
    def __init__(self):
        rospy.init_node('Shopping')
        self.smach_bool =False
        rospy.on_shutdown(self.shutdown)
        self.get_command = StateMachine(outcomes =['succeeded','aborted'])
        self.sm_EnterRoom = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_EnterRoom:
            # rospy.logerr('sm_EnterRoom add State')342
            # arm go to the nav_pose in the srdf
            # self.sm_EnterRoom.userdata.arm_mode =0
            # self.sm_EnterRoom.userdata.arm_ps_1 = PointStamped()
            # StateMachine.add('NAV_POSE',
            #                     ArmCmd(),
            #                     transitions={'succeeded':'DOOR_DETECT','aborted':'NAV_POSE','error':'error'},
            #                     remapping ={'mode':'arm_mode','arm_ps':'arm_ps_1'})
            # wait the door to open
            # StateMachine.add('DOOR_DETECT',
            #                     DoorDetect().door_detect_,
            #                     transitions={'invalid':'WAIT','valid':'DOOR_DETECT','preempted':'error'})
            # simple delay 5s
            self.sm_EnterRoom.userdata.rec = 5.0
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'SIMPLE_MOVE','error':'error'},
                                remapping ={'rec':'rec'})

            self.sm_EnterRoom.userdata.point = Point(1.5,0.0,0.0)
            StateMachine.add('SIMPLE_MOVE',
                                SimpleMove_move(),
                                transitions={'succeeded':'NAV_1','aborted':'NAV_1','error':'error'},
                                remapping={'point':'point'})
            
            # navstack to room of crowds
            # waypoints are the list of Pose fit the data type need
            self.sm_EnterRoom.userdata.start_waypoint  = gpsr_target['speaker']['pos']
            # self.sm_EnterRoom.userdata.nav_mode =0
            StateMachine.add('NAV_1',
                                NavStack(),
                                transitions={'succeeded':'succeeded','aborted':'NAV_1','error':'error'},
                                remapping = {'pos_xm':'start_waypoint'})
            
            # self-introduce
            # self.sm_EnterRoom.userdata.sentences_2 = 'I am robot xiaomeng, I come from the nwpu'
            # StateMachine.add('SELF_INTRO',
            #                     Speak(),
            #                     remapping ={'sentences':'sentences_2'},
            #                     transitions ={'succeeded':'succeeded','aborted':'SELF_INTRO','error':'error'})
        with self.get_command:
            StateMachine.add('GET_FOLLOW',
                                GetSignal(),
                                transitions={'succeeded':'succeeded','aborted':'aborted'})
        self.trace = Concurrence(outcomes = ['remeber','stop','aborted'],
                                 default_outcome = 'stop',
                                 outcome_map={'remeber':{'STOP':'remeber'},
                                              'stop':{'STOP':'stop'},
                                              'aborted':{'FOLLOW':'aborted'}},
                                 child_termination_cb = self.trace_child_cb,
                                 input_keys = ['PT_LIST','mission'],
                                 output_keys= ['PT_LIST','mission'])
        with self.trace:
            self.meta_follow = StateMachine(['succeeded','aborted','preempted'])
            with self.meta_follow:
                StateMachine.add('FIND',
                                    FindPeople().find_people_,
                                    transitions = {'invalid':'META_NAV','valid':'FIND','preempted':'preempted'},
                            
                                    remapping={'pos_xm':'pos_xm'})
                self.meta_nav = Concurrence(outcomes=['time_over','get_pos','aborted'],
                                                default_outcome  = 'aborted',
                                                outcome_map={'time_over':{'WAIT':'succeeded'},
                                                             'get_pos':{'NAV':'succeeded'},
                                                             'aborted':{'NAV':'aborted'}},
                                                child_termination_cb=self.nav_child_cb,
                                                input_keys=['pos_xm'])
                with self.meta_nav:
                    Concurrence.add('NAV',NavStack(),remapping={'pos_xm':'pos_xm'})
                    Concurrence.add('WAIT',Wait_trace())
                StateMachine.add('META_NAV',
                                    self.meta_nav,
                                    transitions={'get_pos':'FIND','time_over':'FIND','aborted':'FIND'})
            Concurrence.add('FOLLOW',self.meta_follow)
            Concurrence.add('STOP',CheckStop(),remapping = {'PT_LIST':'PT_LIST','mission':'mission'})

        # self.get_mission = StateMachine(outcomes=['succeeded','aborted'],
        #                                 output_keys=['things_list'])
        # with self.get_mission:
        #     self.get_mission.userdata.things_list = list()
        #     StateMachine.add('LISTEN',
        #                         GetMission(),
        #                         remapping = ['things_list'],
        #                         transitions={'succeeded':'succeeded','aborted':'aborted'})

        self.nav_mission = StateMachine(outcomes=['succeeded','aborted','error'],
                                        input_keys=['mission'])
        with self.nav_mission:
            self.nav_mission.userdata.pos_xm = Pose()
            StateMachine.add('GET_TARGET',
                                GetTarget(),
                                remapping = {'pos_xm':'pos_xm','mission':'mission','mission_name':'mission_name'},
                                transitions = {'succeeded':'NAV','aborted':'aborted','finish':'NAV_FINAL'})
            StateMachine.add('NAV',
                                NavStack(),
                                remapping = {'pos_xm':'pos_xm'},
                                transitions = {'succeeded':'GET_TARGET','aborted':'aborted','error':'error'}) 
            self.sm_Pick = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['name'])
        
            with self.sm_Pick:
                
                self.sm_Pick.userdata.name =''
                self.sm_Pick.userdata.target_mode =0
                self.sm_Pick.userdata.objmode = -1
                StateMachine.add('RUNNODE_IMG',
                                    RunNode_img(),
                                    transitions = {'succeeded':'GETNAME','aborted':'RUNNODE_IMG'})


                self.sm_Pick.userdata.object_pos = PointStamped()
                StateMachine.add('FIND_OBJECT',
                                    FindObject(),
                                    transitions ={'succeeded':'POS_JUSTFY','aborted':'FIND_OBJECT','error':'SPEAK'},
                                    remapping ={'name':'name','object_pos':'object_pos','objmode':'objmode'})
                
                #making the xm foreward the object may make the grasping task easier  
                self.sm_Pick.userdata.pose = Pose()
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
                                    transitions ={'succeeded':'PICK','aborted':'FIND_AGAIN','error':'SPEAK'},
                                    remapping ={'name':'name','object_pos':'object_pos','objmode':'objmode'})
                self.sm_Pick.userdata.arm_mode_1 =1
                StateMachine.add('PICK',
                                    ArmCmd(),
                                    transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                    remapping ={'arm_ps':'object_pos','mode':'arm_mode_1'})
                self.sm_Pick.userdata.sentences = 'xiao meng can not find things'
                StateMachine.add('SPEAK',
                                    Speak(),
                                    transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})
            StateMachine.add('PICK_THING',
                                self.sm_Pick,
                                transitions={'succeeded':'GET_TARGET','aborted':'aborted','error':'error'},
                                remapping={'name':'mission_name'})

            StateMachine.add('NAV_FINAL',
                                NavStack(),
                                remapping = {'pos_xm':'pos_xm'},
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'})           

        self.GetTask = StateMachine(outcomes = ['succeeded' , 'error' ,'aborted'] , 
                                    output_keys = ['task'])
        
        with self.GetTask:
            self.FindPeo = StateMachine(
                outcomes=['succeeded', 'aborted', 'error'])

            with self.FindPeo:
                self.FindPeo.userdata.rec = 2.0
                StateMachine.add('RUNNODE',
                                RunNode(),
                                transitions={'succeeded': 'WAIT', 'aborted': 'RUNNODE'})  # On simulation , We need to skip RunNode ;So aborted -> Wait Or aborted->RunNode
                StateMachine.add('WAIT',
                                Wait(),
                                transitions={
                                    'succeeded': 'GET_PEOPLE_POS', 'error': 'error'},
                                remapping={'rec': 'rec'})
                self.FindPeo.userdata.pos_xm = Pose()
                StateMachine.add('GET_PEOPLE_POS',
                                FindPeople().find_people_,
                                transitions={
                                    'invalid': 'NAV_PEOPLE', 'valid': 'GET_PEOPLE_POS', 'preempted': 'aborted'},
                                remapping={'pos_xm': 'pos_xm'}
                                )

                StateMachine.add('NAV_PEOPLE',
                                NavStack(),
                                transitions={
                                    'succeeded': 'SPEAK', 'aborted': 'NAV_PEOPLE', 'error': 'error', 'preempted': 'NAV_PEOPLE'},
                                remapping={'pos_xm': 'pos_xm'})
                self.FindPeo.userdata.sentences = 'I find you'
                StateMachine.add('SPEAK',
                                SpeakSentence(),
                                transitions={
                                    'succeeded': 'CLOSEKINECT', 'error': 'error'},
                                remapping={'sentences': 'sentences'})

                StateMachine.add('CLOSEKINECT',
                                CloseKinect(),
                                transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

            StateMachine.add('NAV_PEO' , self.FindPeo , transitions={'succeeded':'ASK_TASK' , 
                                                                     'aborted':'ASK_TASK',
                                                                     'error':'error'})

            self.GetTask.userdata.sentence1 = 'what do you want?'
            StateMachine.add('ASK_TASK' , Speak() , transitions={'succeeded':'ANS1',
                                                             'aborted':'aborted',
                                                             'error':'error'},
                                                    remapping={'sentence':'sentence1'})
            StateMachine.add('ANS1' , ShoppingGetTask() , transitions={'succeeded':'ASK_TASK2',
                                                                        'aborted':'ASK_TAKS2'},
                                                        remapping={'task':'task'})
            StateMachine.add('ASK_TASK2' , Speak() , transitions={'succeeded':'ANS2',
                                                             'aborted':'aborted',
                                                             'error':'error'},
                                                    remapping={'sentence':'sentence1'})
            StateMachine.add('ANS2' , ShoppingGetTask() , transitions={'succeeded':'succeeded',
                                                                        'aborted':'aborted'},
                                                        remapping={'task':'task'})
            


        self.DealTask = StateMachine(outcomes = ['succeeded' , 'error' , 'error'] , input_keys = ['task' , 'mission'])

        with self.DealTask:
            self.DealTask.userdata.nav_pos = Pose()
            self.DealTask.userdata.indice = 0
            self.DealTask.userdata.indice2 == 3
            self.DealTask.userdata.name = ''

            StateMachine.add('NXT_TASK' , ShoppingNextTask() , 
                                transitions={'go':'NAV' , 'back':'NAV_CASH' ,'finish':'succeeded','aborted':"aborted"},
                                remapping={'pose':'nav_pos',
                                            'name':'name',
                                            'indice':'indice'})

            StateMachine.add('NAV' , NavStack() , 
                                transitions={'succeeded':'PICK',
                                                'aborted':'NAV',
                                                'error':'error'},
                                remapping={'pos_xm' , 'nav_pos'})
            
            StateMachine.add('PICK' , self.sm_Pick , 
                                        transitions={'succeeded':'NXT_TASK',
                                                        'aborted':'NXT_TASK',
                                                        'error':'error'},
                                        remapping={'name':'name'})
            StateMachine.add('NAV_CASH' , NavStack() , 
                                transitions={'succeeded':'PLACE',
                                                'aborted':'NAV_CASH',
                                                'error':'error'},
                                remapping={'pos_xm' , 'nav_pos'})
            StateMachine.add('PLACE' , Place2() ,
                                    transitions={'succeeded':'succeeded',
                                                    'aborted':'aborted',
                                                    } )
            
                                          



        self.shopping = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.shopping:
            # 字典名字与位置对应
            self.shopping.userdata.PT_LIST = {}
            self.shopping.userdata.mission = {}
            self.shopping.userdata.task = list()
            self.shopping.userdata.rec = 5.0
            # StateMachine.add('ENTERROOM',
            #                     self.sm_EnterRoom,
            #                     transitions={'succeeded':'START','aborted':'aborted'})
            StateMachine.add('START',
                                self.get_command,
                                transitions={'succeeded':'RUNNODE','aborted':'aborted'})
            StateMachine.add('RUNNODE',
                                RunNode(),
                                transitions={'succeeded':'WAIT','aborted':'aborted'})
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'TRACE','error':'error'},
                                remapping={'rec':'rec'})
            StateMachine.add('TRACE',
                                self.trace,
                                transitions={'remeber':'TRACE','stop':'GET_TASK','aborted':'aborted'},
                                remapping={'PT_LIST':'PT_LIST','mission':'mission'})
            StateMachine.add('GET_TASK' , self.GetTask, 
                                transitions={'succeeded':'DEAL_TASK','aborted':'aborted', 'error':'error'},
                                remapping={'task':'task'})

            StateMachine.add('DEAL_TASK' , self.DealTask , transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                                            remapping={'task':'task'})
            # StateMachine.add('NAV_MISSION',
            #                     self.nav_mission,
            #                     transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
            #                     remapping={'mission':'mission'})
        intro_server = IntrospectionServer('shoping',self.shopping, 'SM_ROOT')
        intro_server.start()
        out = self.shopping.execute()
        intro_server.stop()
        if out == 'succeeded':
            self.smach_bool =True

    def shutdown(self):
        if self.smach_bool ==False:
            rospy.logwarn('smach execute failed')
        else:
            rospy.logwarn('smach execute successfully')

    def trace_child_cb(self,outcome_map):
        if outcome_map['STOP'] == 'stop':
            rospy.logwarn('get the stop signal, stop tracing ........')
            pid = get_pid("people_tracking")
            subprocess.call('kill '+str(pid[0]),shell=True)
            # subprocess.call('xterm -e touch /home/ye/Recognition/kinect2/close_image_test &', shell = True)
            return True
        elif outcome_map['STOP'] == 'remeber':
            rospy.logwarn('ready to remeber the position')
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

if __name__ =="__main__":
    try:
        Shopping()
    except Exception,e:
        rospy.logerr(e)

