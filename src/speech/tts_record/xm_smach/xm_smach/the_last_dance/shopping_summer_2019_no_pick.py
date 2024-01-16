#! /usr/bin/env python

import rospy
from smach_ros import IntrospectionServer
from xm_smach.common_lib import *
from xm_smach.shopping_lib import * 
from xm_smach.target_gpsr import *
from xm_smach.whoiswho_lib import GetValue,FaceReco
from smach import State, StateMachine, Concurrence,CBState
from math import pi
from geometry_msgs.msg import *
import subprocess
from new_lib.special import ShoppingGetTask,ShoppingNextTask
from new_lib.basic_pick import ArmCmdForTf
from new_lib.basic_pick import ArmCmd as new_ArmCmd
from new_lib.special import PosJustfy as new_PosJustfy
from new_lib.basic_vision import FindTwoPeople
from new_lib.basic_vision import FindObject as FindObject2

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

            self.sm_EnterRoom.userdata.start_waypoint  = gpsr_target['speaker']['pos']
            StateMachine.add('ENTER',
                                NavStack(),
                                transitions={'succeeded':'succeeded','aborted':'ENTER','error':'error'},
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
                self.meta_follow.userdata.pos_xm = Pose()
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
            Concurrence.add('STOP',CheckStop2(),remapping = {'PT_LIST':'PT_LIST','mission':'mission'})

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
            
            #remove
            StateMachine.add('FIND', FindObject2(),
                             transitions={'succeeded': 'succeeded',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping = {'name':'target' ,
                                            'object_pos':'target_pos',
                                            'object_map_point':'object_map_point'})
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
                                          },
                             remapping={'pos_xm': 'nav_pos'})
            
            StateMachine.add('PICK2', ArmCmdForTf(),
                             transitions={'succeeded': 'succeeded',
                                          'error': 'error',
                                          'aborted': 'aborted'},
                             remapping={'arm_ps': 'object_map_point', 'mode': 'objmode'})

            StateMachine.add('FIND_AGAIN', FindObject2(),
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

            StateMachine.add('PICK', new_ArmCmd(),
                             transitions={'succeeded': 'succeeded',
                                          'error': 'error',
                                          'aborted': 'aborted'},
                             remapping={'arm_ps': 'pick_pos', 'mode': 'objmode','object_state':'object_state'})


        self.sm_FaceDetect = StateMachine(outcomes = ['succeeded','aborted','error'],
                                            output_keys = ['people_position','num_list'])
        with self.sm_FaceDetect:

            
            self.sm_FaceDetect.userdata.people_position =list()
            
            self.sm_FaceDetect.userdata.sentences = 'Hey man ,what\'s up'
            StateMachine.add('SPEAK',
                                Speak(),
                                remapping = {'sentences':"sentences"},
                                transitions = {'succeeded':'GET_POSITION','aborted':'aborted','error':'error'})
            # call face_reco service for get all the people position which is a list
            self.sm_FaceDetect.userdata.name_id = -1  #
            self.sm_FaceDetect.userdata.num_list = list()
            self.sm_FaceDetect.userdata.distance = 0.6
            StateMachine.add('GET_POSITION',
                                FindTwoPeople(),
                                remapping  ={'people_position':'people_position'},
                                transitions ={'succeeded':'succeeded',
                                              'aborted':'MOVEAHEAD',
                                              'error':'error'})
            # self.sm_FaceDetect.userdata.point_1 = Point(0.1,0.0,0.0) 
            StateMachine.add('MOVEAHEAD' , CBState(self.ahead_justice,outcomes=['succeeded','error']),
                                transitions={'succeeded':'GET_POSITION' ,
                                                    'error':'GET_POSITION',
                                                    } 
                                    
                                            )

            
        self.GetTask = StateMachine(outcomes = ['succeeded' , 'aborted' , 'error'] , output_keys = ['task'])
        with self.GetTask:
            self.GetTask.userdata.people_position = list()
            self.GetTask.userdata.num_list = list()
            self.GetTask.userdata.person_position = Pose()
            self.GetTask.userdata.task = list()
            
            
            StateMachine.add('GET_POSITION' , self.sm_FaceDetect , transitions={'succeeded':'GET_VALUE1',
                                                                                'aborted':'aborted',
                                                                                'error':'error'},
                                                                    remapping={'people_position':'people_position',
                                                                                'num_list':'num_list'})
            
            StateMachine.add('GET_VALUE1',
                                GetValue(),
                                remapping ={'element_list':'people_position','element':'person_position'},
                                transitions ={'succeeded':'NAV1','aborted':"succeeded",'error':'error'}
                                )

            StateMachine.add('NAV1' , 
                                NavStack() , 
                                transitions={'succeeded':'ASK_TASK1',
                                                'aborted':'NAV1',
                                                'error':'error'},
                                remapping={'pos_xm':'person_position'})

            # StateMachine.add('GET_VALUE1',
            #                     GetValue(),
            #                     remapping ={'element_list':'people_position','element':'person_position'},
            #                     transitions ={'succeeded':'ASK_TASK1','aborted':"succeeded",'error':'error'}
            #                     )
            
                                    

            self.GetTask.userdata.sentence1 = 'what do you want?'
            StateMachine.add('ASK_TASK1' , Speak() , transitions={'succeeded':'ANS1',
                                                             'aborted':'GET_VALUE1',
                                                             'error':'error'},
                                                    remapping={'sentences':'sentence1'})
            StateMachine.add('ANS1' , ShoppingGetTask() , transitions={'succeeded':'GET_VALUE1',
                                                                        'aborted':'GET_VALUE1'},
                                                        remapping={'task':'task'})
            # StateMachine.add('ASK_TASK2' , Speak() , transitions={'succeeded':'ANS2',
            #                                                  'aborted':'aborted',
            #                                                  'error':'error'},
            #                                         remapping={'sentence':'sentence1'})
            # StateMachine.add('ANS2' , ShoppingGetTask() , transitions={'succeeded':'succeeded',
            #                                                             'aborted':'aborted'},
            #                                             remapping={'task':'task'})

        self.DealTask = StateMachine(outcomes = ['succeeded' , 'aborted' , 'error'] , input_keys = ['task' , 'mission'])

        with self.DealTask:
            self.DealTask.userdata.nav_pos = Pose()
            self.DealTask.userdata.indice = 0
            self.DealTask.userdata.indice2 = 3
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
                                remapping={'pos_xm':'nav_pos'})
            
            StateMachine.add('PICK' , self.Pick , 
                                        transitions={'succeeded':'NXT_TASK',
                                                        'aborted':'NXT_TASK',
                                                        'error':'error'},
                                        remapping={'target':'name'})
            StateMachine.add('NAV_CASH' , NavStack() , 
                                transitions={'succeeded':'PLACE', 
                                                'aborted':'NAV_CASH',
                                                'error':'error'},
                                remapping={'pos_xm':'nav_pos'})
            
            #place need to be  upgraded #remove
            StateMachine.add('PLACE' , Place2() ,
                                    transitions={'succeeded':'NXT_TASK',
                                                    'aborted':'NXT_TASK',
                                                    } )   


        self.shopping = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.shopping:
            
            self.shopping.userdata.PT_LIST = {}
            self.shopping.userdata.mission1 = {}
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
                                remapping={'PT_LIST':'PT_LIST','mission':'mission1'})
            StateMachine.add('GET_TASK' , self.GetTask, 
                                transitions={'succeeded':'DEAL_TASK','aborted':'aborted', 'error':'error'},
                                remapping={'task':'task'})

            StateMachine.add('DEAL_TASK' , self.DealTask , transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                                            remapping={'task':'task',
                                                                        'mission':'PT_LIST'})

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
            if(userdata.name == 'ice_tea' or userdata.name == 'water' or userdata.name=='tea'):
                userdata.distance = 0.85
            elif userdata.name == 'milk' or userdata.name == 'redbull':
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
                userdata.pick_pos.point.x +=-0.05
                userdata.pick_pos.point.y+= 0
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


    def ahead_justice(self , userdata):
        try:
            self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel',Twist,queue_size=1)
            angular_speed = 0.1
            self.turn = Twist()
            self.turn.linear.x = 0.1
            self.turn.linear.y = 0.0
            self.turn.linear.z = 0.0
            self.turn.angular.x = 0.0
            self.turn.angular.y = 0.0
            self.turn.angular.z = 0.0

            goal_angle = 0.1
            angular_duration = goal_angle/angular_speed
            
            rate = 50
            r = rospy.Rate(rate)
            ticks = int(goal_angle*rate)+5
            for i in range(ticks):
                self.cmd_vel.publish(self.turn)
                r.sleep()   

            return 'succeeded'

        except Exception,e:
            rospy.logerr(e)
            return 'aborted'     

if __name__ == "__main__":
    try:
        Shopping()
    
    except Exception,e:
        rospy.logerr(e)