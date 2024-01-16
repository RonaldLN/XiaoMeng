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
from xm_smach.common_lib import Place2
import math
import subprocess
from xm_smach.pick_turn import PickTurn, IsTurn
import time
from xm_smach.shopping_lib import FindPeople as PeopleTrack

def get_pid(name):
    return map(int,subprocess.check_output(["pidof",name]).split())

class GPSR():
    def __init__(self):
        

        rospy.init_node('GpsrSmach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('gogoogo')
        self.smach_bool = False

        self.showUp = StateMachine(outcomes=['succeeded', 'aborted', 'error'])

        with self.showUp:
            self.showUp.userdata.wait_len = 3.0
            self.showUp.userdata.point = Point()
            self.showUp.userdata.point.x = 1.0
            self.showUp.userdata.point.y = 0
            self.showUp.userdata.point.z = 0

            # wait
            StateMachine.add('DOOR_DETECT',
                                DoorDetect().door_detect_,
                                transitions={'invalid':'WAIT','valid':'DOOR_DETECT','preempted':'aborted'})

            # waits
            StateMachine.add('WAIT', Wait(), transitions={'succeeded': 'ENTER',
                                                          'error': 'error'},
                                            remapping={'rec': 'wait_len'})

            # StateMachine.add('ENTER', LinearDisplacement(), transitions={'succeeded': 'succeeded',
            #                                                              'preempted': 'ENTER',
            #                                                              'error': 'error'},
            #                  remapping={'displacementVec': 'point'})

            self.showUp.userdata.start_waypoint  = gpsr_target['speaker']['pos']
            StateMachine.add('ENTER',
                                NavStack(),
                                transitions={'succeeded':'succeeded','aborted':'ENTER','error':'error','preempted':"ENTER"},
                                remapping = {'pos_xm':'start_waypoint'})

        self.Nav = StateMachine(
            outcomes=['succeeded', 'error', 'aborted'], input_keys=['target'])

        with self.Nav:
            self.Nav.userdata.nav_pos = Pose()

            StateMachine.add('GET_ROOM_POS' ,GpsrGetRoomPos(), transitions={'succeeded':'MAP_NAV',
                                                            'aborted':'aborted',
                                                            'error':'error'},
                                                remapping={'nav_pos':'nav_pos',
                                                        'target':'target'})

            StateMachine.add('MAP_NAV',  NavStack(), transitions={'succeeded': 'succeeded',
                                                                  'aborted': 'MAP_NAV',
                                                                  'error': 'error',
                                                                  'preempted':'MAP_NAV'},
                             remapping={'pos_xm': 'nav_pos'})

            self.Nav.userdata.sentences = 'I arrive here'
            StateMachine.add('SPEAK',
                                SpeakSentence(),
                                transitions = {'succeeded':'succeeded','error':'error'},
                                remapping ={'sentences':'sentences'})

        self.FindObj = StateMachine(outcomes=['succeeded', 'aborted', 'error'], input_keys=['target', 'indice', 'targets','table_num'],
                                    output_keys=['target_pos','object_map_point'])
        with self.FindObj:
            #self.FindObj.userdata.table_num = 1
            self.FindObj.userdata.nav_pos = Pose()
            self.FindObj.userdata.object_map_point = PointStamped()
            self.FindObj.userdata.target_pos = Pose()
            StateMachine.add('GET_POS', GpsrGetTabPos(),
                             transitions={'succeeded': 'NAV', 'aborted': 'aborted', 'error': 'error'})

            StateMachine.add('NAV', NavStack(),
                             transitions={
                                 'succeeded': 'DETECT', 'aborted': 'NAV', 'error': 'error', 'preempted': 'NAV'},
                             remapping={'pos_xm': 'nav_pos'})

            StateMachine.add('DETECT', FindObject(),
                             transitions={'succeeded': 'succeeded',
                                          'aborted': 'GET_POS', 'error': 'error'},
                             remapping={'object_pos': 'target_pos',
                                        'name': 'target',
                                        'object_map_point':'object_map_point'})

            

        self.FindPeo = StateMachine(outcomes=['succeeded', 'aborted', 'error'])

        with self.FindPeo:
            self.FindPeo.userdata.rec = 2.0
            # StateMachine.add('RUNNODE',
            #                  RunNode(),
            #                  transitions={'succeeded': 'WAIT', 'aborted': 'RUNNODE'})  # On simulation , We need to skip RunNode ;So aborted -> Wait Or aborted->RunNode
            # StateMachine.add('WAIT',
            #                  Wait(),
            #                  transitions={
            #                      'succeeded': 'GET_PEOPLE_POS', 'error': 'error'},
            #                  remapping={'rec': 'rec'})
            self.FindPeo.userdata.pos_xm = Pose()
            StateMachine.add('GET_PEOPLE_POS',
                             FindPeople(),#.find_people_,
                             transitions={
                                 'invalid': 'NAV_PEOPLE', 'valid': 'TURN1', 'preempted': 'aborted'},
                             remapping={'pos_xm': 'pos_xm'}
                             )
            
            self.FindPeo.userdata.degree = math.pi/2
            self.FindPeo.userdata.degree2 = -math.pi
            StateMachine.add('TURN1' , TurnDegree() , 
                                transitions={'succeeded' :'GET_PEOPLE_POS2' ,
                                                'aborted':'GET_PEOPLE_POS2' ,
                                                'preempted':'aborted'},
                                remapping={'degree':'degree'})
            StateMachine.add('GET_PEOPLE_POS2',
                             FindPeople(),#.find_people_,
                             transitions={
                                 'invalid': 'NAV_PEOPLE', 'valid': 'TURN2', 'preempted': 'aborted'},
                             remapping={'pos_xm': 'pos_xm'}
                             )
            
            StateMachine.add('TURN2' , TurnDegree() , 
                                transitions={'succeeded' :'GET_PEOPLE_POS3' ,
                                                'aborted':'GET_PEOPLE_POS3' ,
                                                'preempted':'aborted'},
                                remapping={'degree':'degree2'})

            StateMachine.add('GET_PEOPLE_POS3',
                             FindPeople(),#.find_people_,
                             transitions={
                                 'invalid': 'NAV_PEOPLE', 'valid': 'CLOSEKINECT', 'preempted': 'aborted'},
                             remapping={'pos_xm': 'pos_xm'}
                             )

            StateMachine.add('NAV_PEOPLE',
                             NavStack(),
                             transitions={
                                 'succeeded': 'SPEAK', 'aborted': 'NAV_PEOPLE', 'error': 'error','preempted':'NAV_PEOPLE'},
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


        self.Find = StateMachine(outcomes = ['succeeded' , 'aborted','error'],
                                    input_keys=['target', 'indice', 'targets'])
        with self.Find:
            self.Find.userdata.sentence = 'I find it'
            self.Find.userdata.table_num = 1
            StateMachine.add('PERSON_OR_POS',
                                GpsrPersonOrPosition(),
                                transitions ={'person':'PERSON','position':'POS','error':'error'})
            
            StateMachine.add('PERSON' , self.FindPeo , 
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})

            StateMachine.add('POS' , self.FindObj , transitions={'succeeded':'SPEAK','aborted':'SPEAK','error':'error'},
                                                    remapping ={'table_num':'table_num'})

            StateMachine.add('SPEAK', SpeakSentence(),
                             transitions={'succeeded': 'succeeded',
                                          'error': 'error', 'aborted': 'aborted'},
                             remapping={'sentences': 'sentence'})

        self.Pick = StateMachine(outcomes=['succeeded', 'aborted', 'error'],
                                 input_keys=['target', 'indice', 'targets'])

        with self.Pick:
            self.Pick.userdata.target_pos = PointStamped()
            self.Pick.userdata.nav_pos = Pose()
            self.Pick.userdata.pick_pos = PointStamped()
            self.Pick.userdata.distance = 0.9

            self.Pick.userdata.distance2 = 0.9
            self.Pick.userdata.target_mode = 1
            self.Pick.userdata.objmode = 1
            self.Pick.userdata.object_state = 1
            self.Pick.userdata.object_map_point = PointStamped()
            self.Pick.userdata.table_num = 1


            StateMachine.add('FIND', self.FindObj,
                             transitions={'succeeded': 'DISTANCE',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping={'target': 'target',
                                        'indice': 'indice',
                                        'targets': 'targets',
                                        'object_map_point':'object_map_point',
                                        'target_pos': 'target_pos',
                                        'table_num':'table_num'})
            

            # StateMachine.add('FIND', self.FindObj,
            #                  transitions={'succeeded': 'DISTANCE',
            #                               'aborted': 'aborted', 'error': 'error'},
            #                  remapping = {'name':'target' ,
            #                                 'object_pos':'target_pos',
            #                                 'object_map_point':'object_map_point'})
            StateMachine.add('DISTANCE' , CBState(self.PickDistance,outcomes=['succeeded','error'],input_keys=['name'],output_keys=['distance']),
                                transitions={'succeeded':'POS_JUS',
                                                'error':'POS_JUS'},
                                remapping={'distance':'distance',
                                            'name':'target'})
            StateMachine.add('POS_JUS', PosJustfy(),
                             transitions={'succeeded': 'NAV',
                                          'aborted': 'aborted',
                                          'error': 'error'},
                             remapping={'pose': 'nav_pos',
                                        'distance': 'distance',
                                        'object_pos': 'target_pos'})

            StateMachine.add('NAV', NavStack(),
                             transitions={'succeeded': 'FIND_AGAIN',
                                          'aborted': 'NAV',
                                          'error': 'error',
                                          'preempted':'NAV'},
                             remapping={'pos_xm': 'nav_pos'})
            
            StateMachine.add('PICK2', ArmCmdForTf(),
                             transitions={'succeeded': 'succeeded',
                                          'error': 'error',
                                          'aborted': 'aborted'},
                             remapping={'arm_ps': 'object_map_point', 'mode': 'objmode'})

            StateMachine.add('FIND_AGAIN', FindObject(),
                             transitions={'succeeded': 'ARM_PS_JUS',
                                          'aborted': 'PICK2', 'error': 'error'},
                             remapping={'object_pos': 'pick_pos',
                                        'name': 'target',
                                        'object_state':'object_state'})
            
            StateMachine.add('ARM_PS_JUS' , CBState(self.Arm_psJus , outcomes=['succeeded','error'],input_keys=['name'],io_keys = ['pick_pos']),
                                transitions = {'succeeded':'PICK',
                                                'error':'PICK'},
                                remapping = {'pick_pos':'pick_pos',
                                            'name':'target'})

            StateMachine.add('PICK', ArmCmd(),
                             transitions={'succeeded': 'succeeded',
                                          'error': 'error',
                                          'aborted': 'aborted'},
                             remapping={'arm_ps': 'pick_pos', 'mode': 'objmode','object_state':'object_state'})
        

            # StateMachine.add('FIND', self.FindObj,
            #                  transitions={'succeeded': 'POS_JUS',
            #                               'aborted': 'aborted', 'error': 'error'},
            #                  remapping={'target': 'target',
            #                             'indice': 'indice',
            #                             'targets': 'targets',
            #                             'target_pos': 'target_pos'})
            # StateMachine.add('POS_JUS', PosJustfy(),
            #                  transitions={'succeeded': 'NAV',
            #                               'aborted': 'aborted',
            #                               'error': 'error'},
            #                  remapping={'pose': 'nav_pos',
            #                             'distance': 'distance',
            #                             'object_pos': 'target_pos'})

            # StateMachine.add('NAV', NavStack(),
            #                  transitions={'succeeded': 'FIND_AGAIN',
            #                               'aborted': 'NAV',
            #                               'error': 'error',
            #                               'preempted':'NAV'},
            #                  remapping={'pos_xm': 'nav_pos'})

            # StateMachine.add('FIND_AGAIN', FindObject(),
            #                  transitions={'succeeded': 'PICK',
            #                               'aborted': 'JUS_AGAIN', 'error': 'error'},
            #                  remapping={'object_pos': 'pick_pos',
            #                             'name': 'target'})
            # StateMachine.add('JUS_AGAIN', PosJustfy(),
            #                  transitions={'succeeded': 'NAV2',
            #                               'aborted': 'aborted',
            #                               'error': 'error'},
            #                  remapping={'pose': 'nav_pos',
            #                             'distance': 'distance2',
            #                             'object_pos': 'target_pos'})

            # StateMachine.add('NAV2', NavStack(),
            #                  transitions={'aborted': 'NAV2',
            #                               'succeeded': 'FIND_AGAIN2',
            #                               'error': 'error',
            #                               'preempted':'NAV'},
            #                  remapping={'pos_xm': 'nav_pos'})

            # StateMachine.add('FIND_AGAIN2', FindObject(),
            #                  transitions={'succeeded': 'PICK',
            #                               'aborted': 'JUS_AGAIN', 'error': 'error'},
            #                  remapping={'object_pos': 'pick_pos',
            #                             'name': 'target'})

            # StateMachine.add('PICK', ArmCmd(),
            #                  transitions={'succeeded': 'succeeded',
            #                               'error': 'error',
            #                               'aborted': 'aborted'},
            #                  remapping={'arm_ps': 'pick_pos', 'mode': 'objmode'})

        
        

        self.Talk = StateMachine(outcomes=['succeeded', 'aborted', 'error'])
        with self.Talk:
            self.Talk.userdata.people_condition = list()
            self.Talk.userdata.command = 1
            StateMachine.add('SPEAK',
                             GeneralAnswer(),
                             transitions={'succeeded': 'succeeded', 'aborted': 'aborted'})

        # self.Follow = Concurrence(outcomes=['succeeded','aborted'],
        #                                 default_outcome ='succeeded',
        #                                 outcome_map={'succeeded':{'STOP':'stop'},
        #                                              'aborted':{'FOLLOW':'aborted'}},
        #                                 child_termination_cb =self.child_cb)

        # with self.Follow:
        #     self.meta_follow = StateMachine(['succeeded','aborted','preempted'])
        #     with self.meta_follow:
        #         StateMachine.add('FIND',
        #                             FindPeople(),#.find_people_,
        #                             transitions = {'invalid':'META_NAV','valid':'FIND','preempted':'preempted'},
                            
        #                             remapping={'pos_xm':'pos_xm'})
        #         self.meta_nav = Concurrence(outcomes=['time_over','get_pos','aborted'],
        #                                         default_outcome  = 'aborted',
        #                                         outcome_map={'time_over':{'WAIT':'succeeded'},
        #                                                      'get_pos':{'NAV':'succeeded'},
        #                                                      'aborted':{'NAV':'aborted'}},
        #                                         child_termination_cb=self.nav_child_cb,
        #                                         input_keys=['pos_xm'])
        #         with self.meta_nav:
        #             Concurrence.add('NAV',NavStack(),remapping={'pos_xm':'pos_xm'})
        #             Concurrence.add('WAIT',Wait_trace())
        #         StateMachine.add('META_NAV',
        #                             self.meta_nav,
        #                             transitions={'get_pos':'FIND','time_over':'FIND','aborted':'FIND'})
        #     Concurrence.add('FOLLOW',self.meta_follow)
        #     Concurrence.add('STOP',CheckStop())

        # self.Follow = Concurrence(outcomes = ['succeeded','aborted'],
        #                          default_outcome = 'aborted',
        #                          outcome_map={'succeeded':{'STOP':'stop'},
        #                                       'aborted':{'FOLLOW':'aborted'},
        #                                       'aborted':{'FOLLOW':'preempted'}},
        #                          child_termination_cb = self.child_cb,
        #                          input_keys = ['people_id'])
        # with self.Follow:
        #     self.meta_follow = StateMachine(['succeeded','aborted','preempted'],input_keys = ['people_id'])
        #     with self.meta_follow:
        #         self.meta_follow.userdata.pos_xm = Pose()
        #         self.meta_follow.userdata.rec = 0.1
        #         StateMachine.add('FIND',
        #                             LegTracker().tracker,
        #                             transitions = {'invalid':'WAIT','valid':'FIND','preempted':'preempted'},
                            
        #                             remapping={'pos_xm':'pos_xm' , 'people_id':'people_id'})
        #         StateMachine.add('WAIT' , 
        #                             Wait(),
        #                             transitions={'succeeded':'META_NAV' , 'error':'META_NAV'},
        #                             remapping={'rec':'rec'})
        #         self.meta_nav = Concurrence(outcomes=['time_over','get_pos','aborted'],
        #                                         default_outcome  = 'aborted',
        #                                         outcome_map={'time_over':{'WAIT':'succeeded'},
        #                                                      'get_pos':{'NAV':'succeeded'},
        #                                                      'aborted':{'NAV':'aborted'}},
        #                                         child_termination_cb=self.nav_child_cb,
        #                                         input_keys=['pos_xm'])
        #         with self.meta_nav:
        #             Concurrence.add('NAV',NavStack(),remapping={'pos_xm':'pos_xm'})
        #             Concurrence.add('WAIT',Wait_trace())
        #         StateMachine.add('META_NAV',
        #                             self.meta_nav,
        #                             transitions={'get_pos':'FIND','time_over':'FIND','aborted':'FIND'})
        #     Concurrence.add('FOLLOW',self.meta_follow)
        #     Concurrence.add('STOP',CheckStop())

        self.trace = Concurrence(outcomes = ['succeeded','aborted'],
                                 default_outcome = 'aborted',
                                 outcome_map={'succeeded':{'STOP':'stop'},
                                              'aborted':{'FOLLOW':'aborted'},
                                              'aborted':{'FOLLOW':'preempted'}},
                                 child_termination_cb = self.child_cb)
        with self.trace:
            self.meta_follow = StateMachine(['succeeded','aborted','preempted'])
            with self.meta_follow:
                self.meta_follow.userdata.pos_xm = Pose()
                StateMachine.add('FIND',
                                    PeopleTrack().find_people_,
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
            Concurrence.add('STOP',CheckStop())


        self.Speak = StateMachine(outcomes = ['succeeded','error' ,'aborted'] , input_keys =['target'])

        with self.Speak:
            self.Speak.userdata.sentences = ''
            self.Speak.userdata.rec = 2.0
            StateMachine.add('GenerateSentence' , CBState(self.GenSen , outcomes = ['succeeded','aborted'],
                                                                    input_keys = ['target'] , 
                                                                    output_keys = ['sentences']),
                                                transitions = {'succeeded':'SAY',
                                                                'aborted':'aborted'})
            StateMachine.add('SAY' , SpeakSentence() , transitions = {'succeeded':'WAIT' , 'aborted':'aborted' , 'error':'error'},
                                                        remapping  = {'sentences':'sentences'})
            
            StateMachine.add('WAIT' ,Wait() , transitions = {'succeeded':'succeeded',
                                                                'error':'aborted'},
                                                remapping = {'rec':'rec'})
            

        self.handleTask = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'])

        with self.handleTask:
            self.handleTask.userdata.targets = []
            self.handleTask.userdata.actions = []
            self.handleTask.userdata.indice = -1
            self.handleTask.userdata.tasksNum = 3
            self.handleTask.userdata.target = ''
            self.handleTask.userdata.command = 2
            self.handleTask.userdata.consult_task = 'Please command me'
            self.handleTask.userdata.people_condition = list()
            self.handleTask.userdata.ins_counter = 0
            self.handleTask.userdata.speaker_pose = gpsr_target['speaker']['pos']
            self.handleTask.userdata.people_id = -1
            self.handleTask.userdata.rec = 5.0
            StateMachine.add('SHOW' , self.showUp , transitions={'succeeded':'GREETING' , 
                                                                    'aborted':'aborted' , 
                                                                    'error':'error'})

            StateMachine.add('GREETING', SpeakSentence(), transitions={'succeeded': 'GET_TASK',
                                                                       'aborted': 'aborted',
                                                                       'error': 'error'},
                             remapping={'sentences': 'consult_task'})

            StateMachine.add('GET_TASK', GeneralAnswer(), transitions={'succeeded': 'NXT_MOVEMENT',
                                                                       'aborted': 'aborted'},
                             remapping={'targets': 'targets',
                                        'actions': 'actions',
                                        'task_num':'tasksNum'})

            StateMachine.add('NXT_MOVEMENT', GpsrNextDo(), transitions={'done': 'INS_COUNTER',
                                                                        'nav': 'NAV',
                                                                        'pick': 'PICK',
                                                                        'put': 'PUT',
                                                                        'talk': 'TALK',
                                                                        'find': 'FIND',
                                                                        'follow': 'RUNISR2',
                                                                        'speak':'SPEAK',
                                                                        'aborted': 'INS_COUNTER'},
                             remapping={'tasksNum': 'tasksNum',
                                        'indice': 'indice',
                                        'current_target': 'target',
                                        'targets':'targets'})
            
            StateMachine.add('INS_COUNTER' , CBState(self.InstructionCounter , outcomes=['finish' ,'continue' , 'error'],
                                                        io_keys = ['ins_counter','indice']),
                                                transitions={'finish':'OUT',
                                                                'error':'OUT',
                                                                'continue':'BACK'},
                                                remapping = {'ins_counter':'ins_counter',
                                                                'indice':'indice'})
            StateMachine.add('BACK' , NavStack() , transitions={'succeeded':'GET_TASK',
                                                                    'aborted':'BACK',
                                                                    'error':'error',
                                                                    'preempted':'BACK'},
                                                    remapping = {'pos_xm':'speaker_pose'})

            StateMachine.add('NAV', self.Nav, transitions={'succeeded': 'NXT_MOVEMENT',
                                                           'aborted': 'NAV',
                                                           'error': 'error'},
                             remapping={'target': 'target'})

            # wait
            StateMachine.add('PICK', self.Pick, transitions={'succeeded': 'NXT_MOVEMENT',
                                                             'aborted': 'NXT_MOVEMENT',
                                                             'error': 'error'}, 
                                                remapping={'target': 'target'})

            StateMachine.add('PUT', Place2(), transitions={'succeeded': 'NXT_MOVEMENT',
                                                           'aborted': 'NXT_MOVEMENT',
                                                           }, 
                            )

            StateMachine.add('TALK', self.Talk, transitions={'succeeded': 'NXT_MOVEMENT',
                                                             'aborted': 'NXT_MOVEMENT',
                                                             'error': 'error'}, 
                             remapping={'target': 'target'})

            # StateMachine.add('FOLLOW', self.Follow, transitions={'succeeded': 'NXT_MOVEMENT',
            #                                                      'aborted': 'NXT_MOVEMENT',
            #                                                     #  'error': 'error'
            #                                                     }, 
            #                  remapping={'people_id': 'people_id'})

            StateMachine.add('RUNISR2' , CBState(self.RunIsr2 , outcomes = ['succeeded','error']) , 
                                        transitions = {'succeeded':'RUNNODE',
                                                        'error':'RUNNODE'})

            StateMachine.add('RUNNODE',
                                RunNode(),
                                transitions={'succeeded':'WAIT','aborted':'aborted'})
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'TRACE','error':'error'},
                                remapping={'rec':'rec'})
            StateMachine.add('TRACE',
                                self.trace,
                                transitions={'succeeded':'NXT_MOVEMENT','aborted':'NXT_MOVEMENT'},
                               )
            
            StateMachine.add('SPEAK' , self.Speak , transitions = {'succeeded':'NXT_MOVEMENT',
                                                                    'aborted':'NXT_MOVEMENT',
                                                                    'error':'error'},
                                                    remapping = {'target':'target'})

            self.handleTask.userdata.door = gpsr_target['out_door']['pos']
            StateMachine.add('OUT', NavStack(), transitions={'succeeded': 'succeeded',
                                                           'aborted': 'OUT',
                                                           'preempted': 'OUT',
                                                           'error':'error'}, 
                             remapping={'pos_xm': 'door'})
            StateMachine.add('FIND', self.Find, transitions={'succeeded': 'NXT_MOVEMENT',
                                                             'aborted': 'NXT_MOVEMENT',
                                                             'error': 'error'}, 
                             remapping={'target': 'target',
                                        })
    
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


    def child_cb(self,outcome_map):
        if outcome_map['STOP'] == 'stop':
            try:
                rospy.logwarn('get the stop signal, stop tracing ........')
                # subprocess.call('touch /home/ye/Recognition/kinect2/dummy_excution_final &', shell = True)
                pid = get_pid("people_tracking")
                subprocess.call('kill '+str(pid[0]),shell=True)

                rospy.sleep(1.0)

                pid = get_pid("isr_sample_shopping")
                subprocess.call('kill '+str(pid[0]),shell=True)
                rospy.sleep(1.0)

                subprocess.call('gnome-terminal -x bash -c "rosrun xm_speech isr_sample;exec bash"&' ,shell = True)
                rospy.sleep(1.0)
            except Exception,e:
                rospy.logerr(e)
                return True

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

    def PickDistance(self, userdata):
        try:
            if(userdata.name == 'ice_tea' or userdata.name == 'water' or userdata.name=='tea'):
                userdata.distance = 0.85
            elif userdata.name == 'milk' or userdata.name == 'redbull' or userdata.name == 'orange_juice' or userdata.name == 'grape_juice' or userdata.name =='yoghurt':
                userdata.distance =0.8
           
            else:
                userdata.distance = 0.85
            return 'succeeded'

        except Exception ,e:
            rospy.logerr(e)
            return 'error'

    
    def Arm_psJus(self , userdata):
        try:
            if userdata.name == 'ice_tea' :
                userdata.pick_pos.point.x +=-0.02
                userdata.pick_pos.point.y+= 0.02
                userdata.pick_pos.point.z+=0.04
            elif userdata.name == 'tea':
                userdata.pick_pos.point.x +=-0.05
                userdata.pick_pos.point.y+= -0.02
                userdata.pick_pos.point.z+=0.04
            elif userdata.name == 'cola' or userdata.name == 'herbal_tea' or userdata.name == 'fanta' or userdata.name=='porridge' or userdata.name == 'redbull':
                userdata.pick_pos.point.z -= 0.05
            elif userdata.name == 'grape_juice' or userdata.name == 'milk_tea' or userdata.name == 'water':
                userdata.pick_pos.point.z+=0.04
            rospy.logerr(userdata.pick_pos)
        
            return 'succeeded'

        except Exception,e:
            rospy.logerr(e)
            return 'error' 

    
    def GenSen(self , userdata):
        try:
            ct = userdata.target
            if  len(ct)>5:
                userdata.sentences = ct
            elif  ct == 'name':
                userdata.sentences = 'I am  Xiao Meng'
            elif ct == 'tomo':
                userdata.sentences = 'It\'s Saturday , August 31st , two thousand nineteen' 
            else:
                userdata.sentences = 'It\'s Friday , August 30th , two thousand nineteen' 
        
            return 'succeeded'
        
        except Exception,e:
            rospy.logerr(e)
            return 'aborted'
        

    def InstructionCounter(self , userdata):
        try:
            userdata.ins_counter+=1

            if(userdata.ins_counter>=3):
                return 'finish'
            
            else:
                userdata.indice = -1
                return 'continue'
        
        except:
            return 'error'
    
    def RunIsr2(self , userdata):
        try:
            pid = get_pid("isr_sample")
            subprocess.call('kill '+str(pid[0]),shell=True)
            rospy.sleep(1.0)
            subprocess.call('gnome-terminal -x bash -c "rosrun xm_speech isr_sample_shopping;exec bash"&' ,shell = True)
            rospy.sleep(1.0)

            return 'succeeded'

        except Exception,e:
            rospy.logerr(e)
            return 'error'



if __name__ == "__main__":
    try:
        GPSR()
    except Exception,e:
        rospy.logerr(e)   
