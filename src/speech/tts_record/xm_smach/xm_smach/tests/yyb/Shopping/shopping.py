#!/usr/bin/env python
# encoding:utf8
import rospy
from smach_ros import IntrospectionServer
from smach_special.gpsr import *
from smach_special.gpsr import *
from smach_compose.compose import *
from smach_common.common import *
from xm_smach.shopping_lib import *
from xm_smach.target_gpsr import *
from xm_smach.whoiswho_lib import GetValue,FaceReco
from smach import State, StateMachine, Concurrence,CBState
from math import pi
from geometry_msgs.msg import *
import subprocess
from new_lib.special import ShoppingGetTask,ShoppingNextTask,ShoppingNextTask_Test
from new_lib.basic_pick import ArmCmdForTf
from new_lib.basic_pick import ArmCmd as new_ArmCmd
from new_lib.special import PosJustfy as new_PosJustfy
from new_lib.basic_vision import FindObject as FindObject2

def get_pid(name):
    pid = ''
    with open("/home/xm/vision_pid/{}.txt".format(name)) as f:
        pid = f.read()
    return pid

class Shopping():
    def __init__(self):
        rospy.init_node('Shopping')
        self.smach_bool =False
        rospy.on_shutdown(self.shutdown)

        # self.sm_EnterRoom = StateMachine(outcomes = ['succeeded','aborted','error'])

        # with self.sm_EnterRoom:
        #     StateMachine.add('DOOR_DETECT',
        #                         DoorDetect().door_detect_,
        #                         transitions={'invalid':'WAIT','valid':'DOOR_DETECT','preempted':'aborted'})

        #     # waits
        #     StateMachine.add('WAIT', Wait(), transitions={'succeeded': 'ENTER',
        #                                                   'error': 'error'},
        #                                     remapping={'rec': 'wait_len'})

        #     # StateMachine.add('ENTER', LinearDisplacement(), transitions={'succeeded': 'succeeded',
        #     #                                                              'preempted': 'ENTER',
        #     #                                                              'error': 'error'},
        #     #                  remapping={'displacementVec': 'point'})

            # self.sm_EnterRoom.userdata.start_waypoint  = gpsr_target['speaker']['pos']
            # StateMachine.add('ENTER',
            #                     NavStack(),
            #                     transitions={'succeeded':'succeeded','aborted':'ENTER','error':'error'},
            #                     remapping = {'pos_xm':'start_waypoint'})

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
                    Concurrence.add('NAV',NavStack0(),remapping={'pos_xm':'pos_xm'})
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
                                            input_keys= ['people_position_1'],
                                            output_keys = ['people_position','num_list','people_position_1'])
        with self.sm_FaceDetect:


            self.sm_FaceDetect.userdata.people_position =list()

            self.sm_FaceDetect.userdata.sentences = 'Hey man ,please look at me'
            StateMachine.add('SPEAK',
                                Speak(),
                                remapping = {'sentences':"sentences"},
                                transitions = {'succeeded':'GET_POSITION','aborted':'aborted','error':'error'})
            # call face_reco service for get all the people position which is a list
            self.sm_FaceDetect.userdata.name_id = -1  #
            self.sm_FaceDetect.userdata.num_list = list()
            self.sm_FaceDetect.userdata.distance = 0.6
            StateMachine.add('GET_POSITION',
                                FindTwoPeople_1(),
                                remapping  ={'people_position':'people_position','people_position_1':'people_position_1'},
                                transitions ={'succeeded':'succeeded',
                                              'aborted':'aborted',
                                              'error':'error'})
            # # self.sm_FaceDetect.userdata.point_1 = Point(0.1,0.0,0.0)
            # StateMachine.add('MOVEAHEAD' , CBState(self.ahead_justice,outcomes=['succeeded','error']),
            #                     transitions={'succeeded':'GET_POSITION' ,
            #                                         'error':'GET_POSITION',
            #                                         }

            #                                 )


        self.GetTask = StateMachine(outcomes = ['succeeded' , 'aborted' , 'error'] ,
                                    output_keys = ['task','people_position_1'],
                                    input_keys = ['PT_LIST'])
        with self.GetTask:
            self.GetTask.userdata.people_position = list()
            self.GetTask.userdata.people_position_1 = list()
            self.GetTask.userdata.num_list = list()
            self.GetTask.userdata.person_position = Pose()
            self.GetTask.userdata.task = list()
            self.GetTask.userdata.pos_cashier = Pose()
            self.GetTask.userdata.pose = Pose()

            StateMachine.add('GET_POSITION' , self.sm_FaceDetect , transitions={'succeeded':'GET_VALUE1',
                                                                                'aborted':'aborted',
                                                                                'error':'error'},
                                                                    remapping={'people_position':'people_position',
                                                                                'people_position_1':'people_position_1',
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


            self.GetTask.userdata.sentence1 = 'what do you want?'
            StateMachine.add('ASK_TASK1' , Speak() , transitions={'succeeded':'ANS1',
                                                             'aborted':'GET_VALUE1',
                                                             'error':'error'},
                                                    remapping={'sentences':'sentence1'})
            StateMachine.add('ANS1' , ShoppingGetTask() , transitions={'succeeded':'GET_VALUE2',
                                                                        'aborted':'ASK_TASK1'},
                                                        remapping={'task':'task'})


            # StateMachine.add('GET_CASHIER_POS' , GetCashierPos() ,
            #                     transitions={'succeeded':'NAV_CASH' ,'aborted':"aborted"},
            #                     remapping={'pose':'pose',
            #                                 'PT_LIST':'PT_LIST'})

            # StateMachine.add('NAV_CASH' , NavStack() ,
            #                     transitions={'succeeded':'GET_VALUE2',
            #                                     'aborted':'NAV_CASH',
            #                                     'error':'error'},
            #                     remapping={'pos_xm':'pose'})

            StateMachine.add('GET_VALUE2',
                                GetValue(),
                                remapping ={'element_list':'people_position','element':'person_position'},
                                transitions ={'succeeded':'NAV2','aborted':"succeeded",'error':'error'}
                                )

            StateMachine.add('NAV2' ,
                                NavStack() ,
                                transitions={'succeeded':'ASK_TASK2',
                                                'aborted':'NAV2',
                                                'error':'error'},
                                remapping={'pos_xm':'person_position'})

            StateMachine.add('ASK_TASK2' , Speak() , transitions={'succeeded':'ANS2',
                                                             'aborted':'aborted',
                                                             'error':'error'},
                                                    remapping={'sentences':'sentence1'})
            StateMachine.add('ANS2' , ShoppingGetTask() , transitions={'succeeded':'succeeded',
                                                                        'aborted':'ASK_TASK2'},
                                                        remapping={'task':'task'})
            
            StateMachine.add('GET_CASHIER_POS' , GetCashierPos() ,
                                transitions={'succeeded':'NAV_CASH' ,'aborted':"aborted"},
                                remapping={'pose':'pose',
                                            'PT_LIST':'PT_LIST'})

            StateMachine.add('NAV_CASH' , NavStack() ,
                                transitions={'succeeded':'succeeded',
                                                'aborted':'aborted',
                                                'error':'error'},
                                remapping={'pos_xm':'pose'})

        self.DealTask = StateMachine(outcomes = ['succeeded' , 'aborted' , 'error'] , input_keys = ['task' , 'mission','people_position_1'])

        with self.DealTask:
            self.DealTask.userdata.nav_pos = Pose()
            self.DealTask.userdata.indice = 0
            self.DealTask.userdata.indice2 = 3
            self.DealTask.userdata.name = ''
            self.DealTask.userdata.people_position = list()
            self.DealTask.userdata.person_position = Pose()
            self.DealTask.userdata.num_list = list()

            # StateMachine.add('GET_POSITION',
            #                     FindTwoPeople_2(),
            #                     remapping  ={'people_position':'people_position','people_position_1':'people_position_1'},
            #                     transitions ={'succeeded':'NXT_TASK',
            #                                   'aborted':'aborted',
            #                                   'error':'error'})

            # StateMachine.add('NXT_TASK_TEST' , ShoppingNextTask_Test() ,
            #                     transitions={'go':'NXT_TASK_TEST' , 'back':'GET_POSITION_1' ,'finish':'succeeded','aborted':"aborted"},
            #                     remapping={'indice':'indice'})

            StateMachine.add('NXT_TASK' , ShoppingNextTask() ,
                                transitions={'go':'NAV' , 'back':'NAV_CASH' ,'finish':'succeeded','aborted':"aborted"},
                                remapping={'pose':'nav_pos',
                                            'name':'name',
                                            'indice':'indice'})

            StateMachine.add('NAV' , NavStack() ,
                                transitions={'succeeded':'GET',
                                                'aborted':'NAV',
                                                'error':'error'},
                                remapping={'pos_xm':'nav_pos'})
            self.DealTask.userdata.sentence1 = 'please help me get it.'
            StateMachine.add('GET' , Speak() , transitions={'succeeded':'NXT_TASK',
                                                             'aborted':'GET',
                                                             'error':'error'},
                                                    remapping={'sentences':'sentence1'})

            StateMachine.add('PICK' , self.Pick ,
                                        transitions={'succeeded':'NXT_TASK',
                                                        'aborted':'NXT_TASK',
                                                        'error':'error'},
                                        remapping={'target':'name'})

            StateMachine.add('NAV_CASH' , NavStack() ,
                                transitions={'succeeded':'GET_POSITION_1',
                                                'aborted':'NAV_CASH',
                                                'error':'error'},
                                remapping={'pos_xm':'nav_pos'})

            self.DealTask.userdata.sentence2 = 'please look at me.'
            StateMachine.add('ATTENTION' , Speak() , transitions={'succeeded':'GET_POSITION_1',
                                                             'aborted':'ATTENTION',
                                                             'error':'error'},
                                                    remapping={'sentences':'sentence2'})
            StateMachine.add('GET_POSITION_1',
                                FindTwoPeople_2(),
                                remapping  ={'people_position':'people_position','people_position_1':'people_position_1','indice':'indice'},
                                transitions ={'succeeded':'GET_VALUE',
                                              'aborted':'GET_POSITION_1',
                                              'error':'error'})

            StateMachine.add('GET_VALUE',
                                GetValue_new(),
                                remapping ={'element_list':'people_position','person_position':'person_position','indice':'indice'},
                                transitions ={'succeeded':'NAV_PEOPLE','aborted':"succeeded",'error':'error'}
                                )

            StateMachine.add('NAV_PEOPLE' ,
                                NavStack() ,
                                transitions={'succeeded':'ASK_TASK',
                                                'aborted':'NAV_PEOPLE',
                                                'error':'error'},
                                remapping={'pos_xm':'person_position'})


            self.DealTask.userdata.sentence2 = 'here you are'
            StateMachine.add('ASK_TASK' , Speak() , transitions={'succeeded':'NXT_TASK',
                                                             'aborted':'ASK_TASK',
                                                             'error':'error'},
                                                    remapping={'sentences':'sentence2'})
            # #place need to be  upgraded #remove
            # StateMachine.add('PLACE' , Place2() ,
            #                         transitions={'succeeded':'NXT_TASK',
            #                                         'aborted':'NXT_TASK',
            #                                         } )


        self.shopping = StateMachine(outcomes=['succeeded','aborted','error'])
        with self.shopping:

            self.shopping.userdata.PT_LIST = {}
            self.shopping.userdata.mission1 = {}
            self.shopping.userdata.task = list()
            self.shopping.userdata.rec = 5.0
            self.shopping.userdata.start_follow = 'Now I will follow you to next location.'

            # StateMachine.add('ENTERROOM',
            #                     self.sm_EnterRoom,
            #                     transitions={'succeeded':'START','aborted':'aborted'})
            StateMachine.add('START',
                                GetSignal(),
                                transitions={'succeeded':'RUNNODE','aborted':'START'})

            StateMachine.add('RUNNODE',
                                RunNode(),
                                #transitions={'succeeded':'SPEAK_BEGIN_FOLLOW','aborted':'RUNNODE'})
                                transitions={'succeeded':'TRACE','aborted':'RUNNODE'})
            StateMachine.add('SPEAK_BEGIN_FOLLOW',
                            Speak(),
                            transitions = {'succeeded':'TRACE','aborted':'aborted','error':'error'},
                            remapping = {'sentences':'start_follow'}
                            )
            StateMachine.add('TRACE',
                                self.trace,
                                transitions={'remeber':'TRACE','stop':'GET_TASK','aborted':'aborted'},
                                remapping={'PT_LIST':'PT_LIST','mission':'mission1'})
            StateMachine.add('GET_TASK' , self.GetTask,
                                transitions={'succeeded':'DEAL_TASK','aborted':'aborted', 'error':'error'},
                                remapping={'task':'task','PT_LIST':'PT_LIST'})

            StateMachine.add('DEAL_TASK' , self.DealTask , transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                                            remapping={'task':'task',
                                                                        'mission':'PT_LIST'
                                                                        })

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
            print('kill pid -> {}'.format(pid))
            print('kill pid -> {}'.format(pid))
            print('kill pid -> {}'.format(pid))
            print('kill pid -> {}'.format(pid))
            subprocess.Popen(['kill','-9',pid],shell=False)
            # with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
            #     f.write('')
            print("sleepiiiiiiiiing!!!!!!!!!!!")
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