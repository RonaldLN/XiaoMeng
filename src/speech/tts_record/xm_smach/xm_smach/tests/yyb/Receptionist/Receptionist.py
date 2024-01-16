#!/usr/bin/env python
# encoding:utf8
'''
@Author: yybS
@Date: 2020-03-31 09:16:51
LastEditTime: 2020-11-14 10:24:41
LastEditors: Please set LastEditors
@Description: 
@FilePath: /undefined/home/jacy/catkin_ws/src/xm_smach/tests/jacy/Receptionsit/Receptionist.py
'''

import rospy
from std_msgs.msg import String, Int32, Bool, Header
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_common.common import *
from tf.transformations import quaternion_matrix
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from copy import deepcopy

Index = 0

class GetNameAndDrink(State):
    '''
    获取名字和饮料并插入列表
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        io_keys =['name_list','drink_list'])
        #rospy.wait_for_service('speech_core')
        self.speech_client = rospy.ServiceProxy('speech_core', speech_to_smach)
        # self.client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        # self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
    def execute(self,userdata):
        try:
            getattr(userdata, 'name_list')
            getattr(userdata, 'drink_list')
        except:
            rospy.logerr('no param')
            return 'aborted'
        # try:
        #     self.client.wait_for_service(timeout=10)
        # except:
        #     rospy.logerr('xm_speech_meaning service is error')
        #     return 'aborted'
        else :
            #example :  I am Tom and I want ice tea.
            res = self.speech_client.call(command=4)
            rospy.logwarn(res)

            name = res.name
            drink = res.drink
            if drink == '' or name == '':
                return 'aborted'
            rospy.logwarn(name)
            rospy.logwarn(drink)
            userdata.name_list.append(name)
            userdata.drink_list.append(drink)

            return 'succeeded'

class Point_Guest(State):
    '''
    指向的机械臂姿态
    '''
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['angle'])
        self.arm_client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    def execute(self,userdata):
        
        trajectory = JointTrajectory()
        joint_names = ["arm_joint_0", "arm_joint_1", "arm_joint_2","arm_joint_3", "arm_joint_4", "arm_joint_5"]	
        trajectory.joint_names = joint_names

        pos_demo = JointTrajectoryPoint()
        pos_demo.positions = [0.0 for i in joint_names]
        pos_demo.velocities = [0.0 for i in joint_names]
        pos_demo.accelerations = [0.0 for i in joint_names]
        pos_demo.time_from_start = rospy.Duration(5.0)

        arm_waypoints = list()
        joints = [userdata.angle,-0.8067,3.14,0,2.35,0]

        pos = deepcopy(pos_demo)
        pos.positions = joints
        trajectory.points.append(pos)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result(rospy.Duration(60.0))

        return 'succeeded'

class Point(State):
    '''
    指向的机械臂姿态
    '''
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['angle'])
        self.arm_client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    def execute(self,userdata):
        
        trajectory = JointTrajectory()
        joint_names = ["arm_joint_0", "arm_joint_1", "arm_joint_2","arm_joint_3", "arm_joint_4", "arm_joint_5"]	
        trajectory.joint_names = joint_names

        pos_demo = JointTrajectoryPoint()
        pos_demo.positions = [0.0 for i in joint_names]
        pos_demo.velocities = [0.0 for i in joint_names]
        pos_demo.accelerations = [0.0 for i in joint_names]
        pos_demo.time_from_start = rospy.Duration(5.0)

        arm_waypoints = list()
        joints = [userdata.angle,0,1.57,0,1.57,0]

        pos = deepcopy(pos_demo)
        pos.positions = joints
        trajectory.points.append(pos)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result(rospy.Duration(60.0))

        return 'succeeded'


class GetReceptionWorld(State):
    '''
    生成介绍的语句
    '''
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['name_list','drink_list'],
                        output_keys = ['receptionist_sentence'])
    def execute(self, userdata):
        try:
            receptionist_sentene = ""
            
            name_list = deepcopy(userdata.name_list)
            drink_list = deepcopy(userdata.drink_list)
            
            rospy.logwarn(name_list)
            rospy.logwarn(drink_list)

            host_name = name_list.pop(0)
            host_drink = drink_list.pop(0)
            
            recepition_people_name = name_list.pop()
            recepition_people_drink = drink_list.pop()

            if name_list:
                receptionist_sentene = " this is "+recepition_people_name+" and he likes " + recepition_people_drink+"."
                receptionist_sentene += " "+name_list.pop() + " is another guest and he likes " + drink_list.pop()+"."
                receptionist_sentene += " "+host_name + " is the host and he likes " + host_drink+"."
            else:
                receptionist_sentene = " this is "+recepition_people_name+" and he likes " + recepition_people_drink+"."
                receptionist_sentene +=" "+ host_name + " is the host and he likes " + host_drink+"."

            userdata.receptionist_sentence = receptionist_sentene
            return 'succeeded'
        except Exception as  e:
            rospy.logerr(e)
            return 'aborted'

class GetSeatAngle(State):
    
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['seat_angle_list','index'],
                        output_keys = ['index','seat_angle'])
        pi = 3.1415926535
        #rospy.wait_for_service('receptionist')
        self.xm_getAngle = rospy.ServiceProxy('receptionist', xm_getAngle)
        #self.index = 0
        #self.xm_getAngle = rospy.ServiceProxy('receptionist', Int32)
        #rospy.Subscriber('follow', xm_FollowPerson, self.move_base_follow_cb)
        #rospy.init_node('receptionist_subscriber', anonymous=True)

    def execute(self,userdata):
        try:
            a = subprocess.Popen(['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/Receptionist/mrsupw_receptionist_local.py','-d','true'] ,shell =False)
            with open("/home/xm/vision_pid/receptionist.txt",'w+') as f:
                print('-------------------')
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('-------------------')
                f.write(str(a.pid))
            rospy.sleep(7.0)
            # rospy.sleep(10000.0)
            print("!!!")
            #rospy.spin()
            print("@@@@@@@@@@@@")
            # a.wait()
            # if a.poll() != 0:
            #     rospy.logerr('error')
        except Exception as e:
            print(e)
            rospy.logerr('Subscriber process error')
            return 'error'
        
        try:
            self.xm_getAngle.wait_for_service(timeout=30.0)
            #req = xm_getAngleRequest()
            res = self.xm_getAngle.call(0)
            userdata.index = res.index
            self.seat_angle_list = userdata.seat_angle_list
            
            rospy.logwarn(userdata.index)
            userdata.seat_angle = self.seat_angle_list[userdata.index]
            pid = ''
            with open('/home/xm/vision_pid/receptionist.txt') as f:
                pid = f.read()
            print('killing!',pid)
            print('killing!',pid)
            print('killing!',pid)
            subprocess.Popen(['kill -9 {}'.format(pid)],shell=True)
            rospy.sleep(1.0)
            return 'succeeded'
        except Exception as e:
            rospy.logerr(e)
            return 'aborted'

class GetPointSentence(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['index','sentence_list'],
                        output_keys = ['point_sentence'])
    def execute(self,userdata):
        try:

            index = userdata.index
            sentence_list = userdata.sentence_list

            userdata.point_sentence = sentence_list[index]

            return 'succeeded'

        except Exception as e:
            rospy.logerr(e)
            userdata.point_sentence = "please seat in the sofa"
            return 'error'


class Receptionist():
    def __init__(self):
        rospy.init_node('Receptionist_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logwarn('Welcome to Receptionist!!!')
        self.smach_bool = False
        
        self.xm_Arm_OpenDoor = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.xm_Arm_OpenDoor:
            self.xm_Arm_OpenDoor.userdata.sentence_open_door = 'i will open the door ,please be attention to the door.'
            self.xm_Arm_OpenDoor.userdata.move_len_go = 0.7
            self.xm_Arm_OpenDoor.userdata.move_len_back = -1.0
            self.xm_Arm_OpenDoor.userdata.rec = 3
            self.xm_Arm_OpenDoor.userdata.arm_waypoints = [[0,-1.4,3.14,0,0,0]]
            #self.xm_Arm_OpenDoor.userdata.arm_open_door_waypoints = [[-0.2,-0.7,0.5,0,0,1.4]]
            self.xm_Arm_OpenDoor.userdata.arm_open_door_waypoints = [[0,-1.4,3.14,0,1.71,-1.47]]
            StateMachine.add('SPEAK_OPEN_DOOR1',
                                Speak(),
                                transitions={'succeeded':'ARMOPENDOORPOSE','aborted':'ARMOPENDOORPOSE','error':'error'},
                                remapping ={'sentences':'sentence_open_door'}
                                )
            StateMachine.add('ARMOPENDOORPOSE',
                                ArmTrajectory(),
                                transitions={'succeeded':'GOAHEAD','error':'error'},
                                remapping ={'arm_waypoints':'arm_open_door_waypoints'}
                                ) 
            StateMachine.add('GOAHEAD',
                                GoAhead(),
                                transitions={'succeeded':'WAIT','aborted':'WAIT','error':'error'},
                                remapping ={'move_len':'move_len_go'}
                                ) 
            StateMachine.add('WAIT',
                                Wait(),
                                transitions={'succeeded':'GOBACK','error':'error'},
                                remapping ={'rec':'rec'}
                                ) 
            StateMachine.add('GOBACK',
                                GoAhead(),
                                transitions={'succeeded':'TAKEBACKARM','aborted':'TAKEBACKARM','error':'error'},
                                remapping ={'move_len':'move_len_back'}
                                ) 
            StateMachine.add('TAKEBACKARM',
                                ArmTrajectory(),
                                transitions={'succeeded':'succeeded','error':'error'},
                                remapping ={'arm_waypoints':'arm_waypoints'}
                                ) 
        self.xm_PointSeat = StateMachine(outcomes = ['succeeded','aborted','error'],
                                        input_keys=['point_pos'])
        with self.xm_PointSeat:
            self.xm_PointSeat.userdata.arm_waypoints = [[0,-1.3,1.57,0,-1.3,0]]
            self.xm_PointSeat.userdata.seat_angle_list = [-40*3.14/180,-15*3.14/180,15*3.14/180,40*3.14/180]
            self.xm_PointSeat.userdata.sentence_list = ['please seat in the left chair.',
                                                        'please seat in the center sofa.',
                                                        'please seat in the center sofa.',
                                                        'please seat in the rigt sofa.']
            self.xm_PointSeat.userdata.index = 0
            StateMachine.add('GETSEATANGLE',
                                GetSeatAngle(),
                                transitions={'succeeded':'GETPERSONPOS1','aborted':'GETPERSONPOS1','error':'error'},
                                remapping ={'seat_angle':'seat_angle','index':'index','seat_angle_list':'seat_angle_list'}
                                )  
            # StateMachine.add('NAVPOINTPOSE',
            #                     NavStack(),
            #                     transitions={'succeeded':'GETPERSONPOS1','aborted':'GETPERSONPOS1','error':'error'},
            #                     remapping ={'pos_xm':'point_pos'}
            #                     )            
            StateMachine.add('GETPERSONPOS1',
                                Point(),
                                transitions={'succeeded':'GETPOINTSENTENCE','aborted':'aborted','error':'error'},
                                remapping= {'angle':'seat_angle'}
                                )
            StateMachine.add('GETPOINTSENTENCE',
                                GetPointSentence(),
                                transitions={'succeeded':'SPEAK1','aborted':'aborted','error':'error'},
                                remapping ={'index':'index','sentence_list':'sentence_list','point_sentence':'point_sentence'}
                                )                                 
            StateMachine.add('SPEAK1',
                                Speak(),
                                transitions={'succeeded':'TAKEBACKARM','aborted':'TAKEBACKARM','error':'error'},
                                remapping ={'sentences':'point_sentence'}
                                ) 
            StateMachine.add('TAKEBACKARM',
                                ArmTrajectory(),
                                transitions={'succeeded':'succeeded','error':'error'},
                                remapping ={'arm_waypoints':'arm_waypoints'}
                                ) 

        #顶层状态机
        self.RECEPTIONIST = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.RECEPTIONIST:
            self.RECEPTIONIST.userdata.name_list = ['john']
            self.RECEPTIONIST.userdata.drink_list = ['cola']
            self.RECEPTIONIST.userdata.pos_door = gpsr_target['open_door_pose']['pos']#开门的位置
            self.RECEPTIONIST.userdata.pos_scan = gpsr_target['receptionist_pose']['pos']#观察的位置
            self.RECEPTIONIST.userdata.point_pos = gpsr_target['point_pose']['pos']#指座位的位置
            self.RECEPTIONIST.userdata.sentences_ask = 'tell me your name and your favourite drink.'
            self.RECEPTIONIST.userdata.sentences_point = 'please have a seat.'
            self.RECEPTIONIST.userdata.sentences_open_door = 'please help me open the door.'
            self.RECEPTIONIST.userdata.sentences_stand_right = 'please stand in my left front.'
            self.RECEPTIONIST.userdata.straight_angle = 0
            self.RECEPTIONIST.userdata.rec_wait = 3
            self.RECEPTIONIST.userdata.sentences_follow = 'please follow me.'
            self.RECEPTIONIST.userdata.arm_back_waypoints = [[0,-1.3,1.57,0,-1.3,0]]
            # StateMachine.add('NAVDOOR1',
            #                     NavStack(),
            #                     transitions={'succeeded':'ARM_OPEN_DOOR1','aborted':'NAVDOOR1','error':'error'},
            #                     remapping ={'pos_xm':'pos_door'}
            #                     )
            StateMachine.add('ARM_OPEN_DOOR1',
                                self.xm_Arm_OpenDoor,
                                transitions={'succeeded':'ASK1','aborted':'ASK1','error':'error'}
                                )
            StateMachine.add('ASK1',
                                Speak(),
                                transitions={'succeeded':'GETINFORMATION1','aborted':'GETINFORMATION1','error':'error'},
                                remapping ={'sentences':'sentences_ask'}
                                ) 
            StateMachine.add('GETINFORMATION1',
                                GetNameAndDrink(),
                                transitions={'succeeded':'POINT_GUEST1','aborted':'GETINFORMATION1','error':'error'},
                                remapping ={'name_list':'name_list','drink_list':'drink_list'}
                                )
            StateMachine.add('POINT_GUEST1',
                                Point_Guest(),
                                transitions={'succeeded':'GET_RECEPTION_WORLD1','aborted':'aborted','error':'error'},
                                remapping={'angle':'straight_angle'}#attention the angle
                                )
            StateMachine.add('GET_RECEPTION_WORLD1',
                                GetReceptionWorld(),
                                transitions={'succeeded':'RECEPTIONIST1','aborted':'aborted','error':'error'},
                                remapping ={'name_list':'name_list','drink_list':'drink_list'}
                                )
            StateMachine.add('RECEPTIONIST1',
                                Speak(),
                                transitions={'succeeded':'TAKEBACKARM_AFTER_POINT1','aborted':'aborted','error':'error'},
                                remapping ={'sentences':'receptionist_sentence'}
                                )
            StateMachine.add('TAKEBACKARM_AFTER_POINT1',
                                ArmTrajectory(),
                                transitions={'succeeded':'SPEAK_FOLLOW1','error':'error'},
                                remapping ={'arm_waypoints':'arm_back_waypoints'}
                                )   
            StateMachine.add('SPEAK_FOLLOW1',
                                Speak(),
                                transitions={'succeeded':'NAVLIVINGROOM1','aborted':'NAVLIVINGROOM1','error':'error'},
                                remapping ={'sentences':'sentences_follow'}
                                ) 
            StateMachine.add('NAVLIVINGROOM1',
                                NavStack(),
                                transitions={'succeeded':'POINTSEAT1','aborted':'NAVLIVINGROOM1','error':'error'},
                                remapping ={'pos_xm':'pos_scan'}
                                )


            # StateMachine.add('SPEAK_RIGHT_POSE1',
            #                     Speak(),
            #                     transitions={'succeeded':'WAIT_STAND_RIGHT1','aborted':'WAIT_STAND_RIGHT1','error':'error'},
            #                     remapping ={'sentences':'sentences_stand_right'}
            #                     ) 
            # StateMachine.add('WAIT_STAND_RIGHT1',
            #                     Wait(),
            #                     transitions={'succeeded':'POINT_GUEST1','error':'error'},
            #                     remapping ={'rec':'rec_wait'}
            #                     ) 
            # StateMachine.add('POINT_GUEST1',
            #                     Point(),
            #                     transitions={'succeeded':'GET_RECEPTION_WORLD1','aborted':'aborted','error':'error'},
            #                     remapping={'angle':'straight_angle'}
            #                     )
            # StateMachine.add('GET_RECEPTION_WORLD1',
            #                     GetReceptionWorld(),
            #                     transitions={'succeeded':'RECEPTIONIST1','aborted':'aborted','error':'error'},
            #                     remapping ={'name_list':'name_list','drink_list':'drink_list'}
            #                     )
            # StateMachine.add('RECEPTIONIST1',
            #                     Speak(),
            #                     transitions={'succeeded':'TAKEBACKARM_AFTER_POINT1','aborted':'aborted','error':'error'},
            #                     remapping ={'sentences':'receptionist_sentence'}
            #                     ) 
            # StateMachine.add('TAKEBACKARM_AFTER_POINT1',
            #                     ArmTrajectory(),
            #                     transitions={'succeeded':'POINTSEAT1','error':'error'},
            #                     remapping ={'arm_waypoints':'arm_back_waypoints'}
            #                     )             
            StateMachine.add('POINTSEAT1',
                                self.xm_PointSeat,
                                transitions={'succeeded':'NAVDOOR2','aborted':'aborted','error':'error'} ,
                                remapping={'point_pos':'point_pos'})
            
            StateMachine.add('NAVDOOR2',
                                NavStack(),
                                transitions={'succeeded':'ASK2','aborted':'ASK2','error':'error'},
                                remapping ={'pos_xm':'pos_door'}
                                )
            # StateMachine.add('SPEAK_OPEN_DOOR2',
            #                     Speak(),
            #                     transitions={'succeeded':'GETINFORMATION2','aborted':'GETINFORMATION2','error':'error'},
            #                     remapping ={'sentences':'sentences_open_door'}
            #                     )
            StateMachine.add('ASK2',
                                Speak(),
                                transitions={'succeeded':'GETINFORMATION2','aborted':'GETINFORMATION2','error':'error'},
                                remapping ={'sentences':'sentences_ask'}
                                ) 
            StateMachine.add('GETINFORMATION2',
                                GetNameAndDrink(),
                                transitions={'succeeded':'POINT_GUEST2','aborted':'GETINFORMATION2','error':'error'},
                                remapping ={'name_list':'name_list','drink_list':'drink_list'}
                                )
            StateMachine.add('POINT_GUEST2',
                                Point_Guest(),
                                transitions={'succeeded':'GET_RECEPTION_WORLD2','aborted':'aborted','error':'error'},
                                remapping={'angle':'straight_angle'}
                                )
            StateMachine.add('GET_RECEPTION_WORLD2',
                                GetReceptionWorld(),
                                transitions={'succeeded':'RECEPTIONIST2','aborted':'aborted','error':'error'},
                                remapping ={'name_list':'name_list','drink_list':'drink_list'}
                                )
            StateMachine.add('RECEPTIONIST2',
                                Speak(),
                                transitions={'succeeded':'TAKEBACKARM_AFTER_POINT2','aborted':'aborted','error':'error'},
                                remapping ={'sentences':'receptionist_sentence'}
                                ) 
            StateMachine.add('TAKEBACKARM_AFTER_POINT2',
                                ArmTrajectory(),
                                transitions={'succeeded':'SPEAK_FOLLOW2','error':'error'},
                                remapping ={'arm_waypoints':'arm_back_waypoints'}
                                ) 
            StateMachine.add('SPEAK_FOLLOW2',
                                Speak(),
                                transitions={'succeeded':'NAVLIVINGROOM2','aborted':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_follow'}
                                ) 
            StateMachine.add('NAVLIVINGROOM2',
                                NavStack(),
                                transitions={'succeeded':'POINTSEAT2','aborted':'POINTSEAT2','error':'error'},
                                remapping ={'pos_xm':'pos_scan'}
                                )
            # StateMachine.add('SPEAK_RIGHT_POSE2',
            #                     Speak(),
            #                     transitions={'succeeded':'WAIT_STAND_RIGHT2','aborted':'aborted','error':'error'},
            #                     remapping ={'sentences':'sentences_stand_right'}
            #                     )  
            # StateMachine.add('WAIT_STAND_RIGHT2',
            #                     Wait(),
            #                     transitions={'succeeded':'POINT_GUEST2','error':'error'},
            #                     remapping ={'rec':'rec_wait'}
            #                     ) 
            # StateMachine.add('POINT_GUEST2',
            #                     Point(),
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
            #                     transitions={'succeeded':'TAKEBACKARM_AFTER_POINT2','aborted':'aborted','error':'error'},
            #                     remapping ={'sentences':'receptionist_sentence'}
            #                     ) 
            # StateMachine.add('TAKEBACKARM_AFTER_POINT2',
            #                     ArmTrajectory(),
            #                     transitions={'succeeded':'POINTSEAT2','error':'error'},
            #                     remapping ={'arm_waypoints':'arm_back_waypoints'}
            #                     ) 
            
            StateMachine.add('POINTSEAT2',
                                self.xm_PointSeat,
                                transitions={'succeeded':'NAVDOOR3','aborted':'POINTSEAT2','error':'error'},
                                remapping={'point_pos':'point_pos'})  
            StateMachine.add('NAVDOOR3',
                                NavStack(),
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={'pos_xm':'pos_door'}
                                )                                              
        intro_server = IntrospectionServer('RECEPTIONIST',self.RECEPTIONIST,'/XM_ROOT')
        intro_server.start()
        out = self.RECEPTIONIST.execute()
        intro_server.stop()

    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')


if __name__ == "__main__":
    try:
        Receptionist()
    except Exception,e:
        rospy.logerr(e)   
