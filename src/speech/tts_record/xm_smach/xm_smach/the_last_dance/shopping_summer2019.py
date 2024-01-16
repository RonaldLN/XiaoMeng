#! usr/bin/env python

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
from new_lib.basic_pick import ArmCmdForTf
from new_lib.basic_pick import ArmCmd as new_ArmCmd
from new_lib.special import PosJustfy as new_PosJustfy

def get_pid(name):
    return map(int,subprocess.check_output(["pidof",name]).split())

class Shopping():
    def __init__(self):
        rospy.init_node('Shopping')
        self.smach_bool =False
        rospy.on_shutdown(self.shutdown)

        self.sm_EnterRoom = StateMachine(outcomes = ['succeeded','aborted','error'])
        
        with self.sm_EnterRoom:
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

        # how to get stop signal?
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

        self.Pick = StateMachine(outcomes = ['succeeded','aborted' , 'error'] , input_keys =['target'])
        with self.Pick:

            self.Pick.userdata.target_pos = PointStamped()
            self.Pick.userdata.nav_pos = Pose()
            self.Pick.userdata.pick_pos = PointStamped()
            self.Pick.userdata.distance = 1.0
            self.Pick.userdata.distance2 = 0.9
            self.Pick.userdata.target_mode = 1
            self.Pick.userdata.objmode = 1
            #self.Pick.userdata.target = 'ice_tea'

            StateMachine.add('FIND', FindObject(),
                             transitions={'succeeded': 'DISTANCE',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping = {'name':'target' ,
                                            'object_pos':'target_pos',
                                            'object_map_point':'object_map_point'})
        #    StateMachine.add('FIND', self.FindObj,
        #                      transitions={'succeeded': 'DISTANCE',
        #                                   'aborted': 'aborted', 'error': 'error'},
        #                      remapping={'target': 'target',
        #                                 'indice': 'indice',
        #                                 'targets': 'targets',
        #                                 'object_map_point':'object_map_point',
        #                                 'target_pos': 'target_pos'})
            

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
            StateMachine.add('POS_JUS', new_PosJustfy(),
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
                             transitions={'succeeded': 'PICK',
                                          'aborted': 'PICK2', 'error': 'error'},
                             remapping={'object_pos': 'pick_pos',
                                        'name': 'target',
                                        'object_state':'object_state'})

            StateMachine.add('PICK', new_ArmCmd(),
                             transitions={'succeeded': 'succeeded',
                                          'error': 'error',
                                          'aborted': 'aborted'},
                             remapping={'arm_ps': 'pick_pos', 'mode': 'objmode','object_state':'object_state'})
            
        self.GetTask = StateMachine(outcomes = ['succeeded' , 'aborted' , 'error'] , output_keys = ['task'])
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
            
            StateMachine.add('PICK' , self.Pick , 
                                        transitions={'succeeded':'NXT_TASK',
                                                        'aborted':'NXT_TASK',
                                                        'error':'error'},
                                        remapping={'target':'name'})
            StateMachine.add('NAV_CASH' , NavStack() , 
                                transitions={'succeeded':'PLACE',
                                                'aborted':'NAV_CASH',
                                                'error':'error'},
                                remapping={'pos_xm' , 'nav_pos'})
            
            #place need to be  upgraded
            StateMachine.add('PLACE' , Place2() ,
                                    transitions={'succeeded':'NXT_TASK',
                                                    'aborted':'NXT_TASK',
                                                    } )   


        self.shopping = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.shopping:
            
            self.shopping.userdata.PT_LIST = {}
            self.shopping.userdata.mission = {}
            self.shopping.userdata.task = list()
            self.shopping.userdata.rec = 5.0
            # StateMachine.add('ENTERROOM',
            #                     self.sm_EnterRoom,
            #                     transitions={'succeeded':'START','aborted':'aborted'})
            StateMachine.add('START',
                                GetSignal(),
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

        intro_server = IntrospectionServer('shopping',self.shopping, 'SM_ROOT')
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

    
    def PickDistance(self, userdata):
        try:
            if(userdata.name == 'ice_tea'):
                userdata.distance = 0.85
            elif userdata.name == 'milk':
                userdata.distance =0.8
            else:
                userdata.distance = 0.85
            
            return 'succeeded'

        except Exception ,e:
            rospy.logerr(e)
            return 'error' 

if __name__ == "__main__":
    try:
        Shopping()
    
    except Exception,e:
        rospy.logerr(e)