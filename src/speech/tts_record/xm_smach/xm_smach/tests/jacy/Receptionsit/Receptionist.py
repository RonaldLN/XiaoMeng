#!/usr/bin/env python
# encoding:utf8
'''
@Author: 愤怒的卤蛋
@Date: 2020-03-31 09:16:51
LastEditTime: 2020-11-14 10:24:41
LastEditors: Please set LastEditors
@Description: 
@FilePath: /undefined/home/jacy/catkin_ws/src/xm_smach/tests/jacy/Receptionsit/Receptionist.py
'''

import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_common.common import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from copy import deepcopy
class GetNameAndDrink(State):
    '''
    获取名字和饮料并插入列表
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        io_keys =['name_list','drink_list'])

        self.client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
    def execute(self,userdata):
        try:
            getattr(userdata, 'name_list')
            getattr(userdata, 'drink_list')
        except:
            rospy.logerr('no param')
            return 'aborted'
        try:
            self.client.wait_for_service(timeout=10)
        except:
            rospy.logerr('xm_speech_meaning service is error')
            return 'aborted'
        else :
            #example :  I am Tom and I want ice tea.
            res = self.client.call(command=3)
            rospy.logwarn(res)

            name = res.name.pop()
            drink = res.object.pop()
            rospy.logwarn(name)
            rospy.logwarn(drink)
            userdata.name_list.append(name)
            userdata.drink_list.append(drink)

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
        joints = [userdata.angle,-1.1,0.9,0,-0.3,0]

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
        except Exception, e:
            rospy.logerr(e)
            return 'aborted'

class GetSeatAngle(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['seat_angle_list'],
                        output_keys = ['index','seat_angle'])
        pi = 3.1415926535
        
    def execute(self,userdata):
        try:
            a = subprocess.Popen('xterm -e python3 Vision/Function/receptionist_new.py receptionist',shell =True)
            rospy.sleep(4.0)
            a.wait()
            if a.poll() != 0:
                rospy.logerr('error')
        except:
            rospy.logerr('Vision/Function/receptionist.py process error')
            #return 'error'
        else:
            try:
                self.seat_angle_list = userdata.seat_angle_list
                person_pos_list = list()
                seat_num = len(self.seat_angle_list)
                
                filename = '/home/domistic/Vision/TXT/receptionist.txt'
                index = 0
                single = 0
                information = list()
                seat_index_list = list()
                with open(filename) as f:
                    information = f.readlines()
                    information = list(information[0].replace('\n','').split(' ')[0:len(information[0].replace('\n','').split(' '))])
                
                for i in range(len(information)-1):
                    seat_index_str = information[i]
                    seat_index = int(seat_index_str)
                    seat_index_list.append(seat_index)
                #3 4 选3
                for i in range(len(seat_index_list)):
                    if i < len(seat_index_list)-1 and seat_index_list[i] + 1 == seat_index_list[i+1]:
                        signle = 1
                        index = seat_index_list[i] + 1
                        break
                
                if single == 0 and len(seat_index_list)>=1:
                    index = seat_index_list[0]
                rospy.logwarn(index)
                userdata.seat_angle = self.seat_angle_list[index]
                userdata.index = index
                return 'succeeded'
            except Exception ,e:
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

        except Exception ,e:
            rospy.logerr(e)
            userdata.point_sentence = "please seat in the sofa"
            return 'error'


class Receptionist():
    def __init__(self):
        rospy.init_node('Receptionist_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logwarn('Welcome to Receptionist!!!')
        self.smach_bool = False
        
        self.sm_Arm_OpenDoor = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.sm_Arm_OpenDoor:
            self.sm_Arm_OpenDoor.userdata.sentence_open_door = 'i will open the door ,please be attention to the door.'
            self.sm_Arm_OpenDoor.userdata.move_len_go = 1
            self.sm_Arm_OpenDoor.userdata.move_len_back = -1
            self.sm_Arm_OpenDoor.userdata.rec = 3
            self.sm_Arm_OpenDoor.userdata.arm_waypoints = [[0,-1.3,1.57,0,-1.3,0]]
            self.sm_Arm_OpenDoor.userdata.arm_open_door_waypoints = [[-0.2,-0.7,0.5,0,0,1.4]]
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
        self.sm_PointSeat = StateMachine(outcomes = ['succeeded','aborted','error'],
                                        input_keys=['point_pos'])
        with self.sm_PointSeat:
            self.sm_PointSeat.userdata.arm_waypoints = [[0,-1.3,1.57,0,-1.3,0]]
            self.sm_PointSeat.userdata.seat_angle_list = [-0.6,-0.2,0.1,0.3,0.5,0.9]
            self.sm_PointSeat.userdata.sentence_list = ['please seat in the right chair.',
                                                        'please seat in the right sofa.',
                                                        'please seat in the right sofa.',
                                                        'please seat in the left sofa.',
                                                        'please seat in the left sofa.',
                                                        'please seat in the left chair.']

            StateMachine.add('GETSEATANGLE',
                                GetSeatAngle(),
                                transitions={'succeeded':'NAVPOINTPOSE','aborted':'NAVPOINTPOSE','error':'error'},
                                remapping ={'seat_angle':'seat_angle','index':'index','seat_angle_list':'seat_angle_list'}
                                )  
            StateMachine.add('NAVPOINTPOSE',
                                NavStack(),
                                transitions={'succeeded':'GETPERSONPOS1','aborted':'GETPERSONPOS1','error':'error'},
                                remapping ={'pos_xm':'point_pos'}
                                )            
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
            self.RECEPTIONIST.userdata.right_angle = -1
            self.RECEPTIONIST.userdata.rec_wait = 3
            self.RECEPTIONIST.userdata.sentences_follow = 'please follow me.'
            self.RECEPTIONIST.userdata.arm_back_waypoints = [[0,-1.3,1.57,0,-1.3,0]]
            StateMachine.add('NAVDOOR1',
                                NavStack(),
                                transitions={'succeeded':'ARM_OPEN_DOOR1','aborted':'NAVDOOR1','error':'error'},
                                remapping ={'pos_xm':'pos_door'}
                                )
            StateMachine.add('ARM_OPEN_DOOR1',
                                self.sm_Arm_OpenDoor,
                                transitions={'succeeded':'ASK1','aborted':'ASK1','error':'error'}
                                )
            StateMachine.add('ASK1',
                                Speak(),
                                transitions={'succeeded':'GETINFORMATION1','aborted':'GETINFORMATION1','error':'error'},
                                remapping ={'sentences':'sentences_ask'}
                                ) 
            StateMachine.add('GETINFORMATION1',
                                GetNameAndDrink(),
                                transitions={'succeeded':'SPEAK_FOLLOW1','aborted':'GETINFORMATION1','error':'error'},
                                remapping ={'name_list':'name_list','drink_list':'drink_list'}
                                )
            StateMachine.add('SPEAK_FOLLOW1',
                                Speak(),
                                transitions={'succeeded':'NAVLIVINGROOM1','aborted':'NAVLIVINGROOM1','error':'error'},
                                remapping ={'sentences':'sentences_follow'}
                                ) 
            StateMachine.add('NAVLIVINGROOM1',
                                NavStack(),
                                transitions={'succeeded':'SPEAK_RIGHT_POSE1','aborted':'NAVLIVINGROOM1','error':'error'},
                                remapping ={'pos_xm':'pos_scan'}
                                )
            StateMachine.add('SPEAK_RIGHT_POSE1',
                                Speak(),
                                transitions={'succeeded':'WAIT_STAND_RIGHT1','aborted':'WAIT_STAND_RIGHT1','error':'error'},
                                remapping ={'sentences':'sentences_stand_right'}
                                ) 
            StateMachine.add('WAIT_STAND_RIGHT1',
                                Wait(),
                                transitions={'succeeded':'POINT_GUEST1','error':'error'},
                                remapping ={'rec':'rec_wait'}
                                ) 
            StateMachine.add('POINT_GUEST1',
                                Point(),
                                transitions={'succeeded':'GET_RECEPTION_WORLD1','aborted':'aborted','error':'error'},
                                remapping={'angle':'right_angle'}
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
                                transitions={'succeeded':'POINTSEAT1','error':'error'},
                                remapping ={'arm_waypoints':'arm_back_waypoints'}
                                )             
            StateMachine.add('POINTSEAT1',
                                self.sm_PointSeat,
                                transitions={'succeeded':'NAVDOOR2','aborted':'aborted','error':'error'} ,
                                remapping={'point_pos':'point_pos'})
            
            StateMachine.add('NAVDOOR2',
                                NavStack(),
                                transitions={'succeeded':'SPEAK_OPEN_DOOR2','aborted':'SPEAK_OPEN_DOOR2','error':'error'},
                                remapping ={'pos_xm':'pos_door'}
                                )
            StateMachine.add('SPEAK_OPEN_DOOR2',
                                Speak(),
                                transitions={'succeeded':'ASK2','aborted':'ASK2','error':'error'},
                                remapping ={'sentences':'sentences_open_door'}
                                )
            StateMachine.add('ASK2',
                                Speak(),
                                transitions={'succeeded':'GETINFORMATION2','aborted':'GETINFORMATION2','error':'error'},
                                remapping ={'sentences':'sentences_ask'}
                                ) 
            StateMachine.add('GETINFORMATION2',
                                GetNameAndDrink(),
                                transitions={'succeeded':'SPEAK_FOLLOW2','aborted':'GETINFORMATION2','error':'error'},
                                remapping ={'name_list':'name_list','drink_list':'drink_list'}
                                )
            StateMachine.add('SPEAK_FOLLOW2',
                                Speak(),
                                transitions={'succeeded':'NAVLIVINGROOM2','aborted':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_follow'}
                                ) 
            StateMachine.add('NAVLIVINGROOM2',
                                NavStack(),
                                transitions={'succeeded':'SPEAK_RIGHT_POSE2','aborted':'SPEAK_RIGHT_POSE2','error':'error'},
                                remapping ={'pos_xm':'pos_scan'}
                                )
            StateMachine.add('SPEAK_RIGHT_POSE2',
                                Speak(),
                                transitions={'succeeded':'WAIT_STAND_RIGHT2','aborted':'aborted','error':'error'},
                                remapping ={'sentences':'sentences_stand_right'}
                                )  
            StateMachine.add('WAIT_STAND_RIGHT2',
                                Wait(),
                                transitions={'succeeded':'POINT_GUEST2','error':'error'},
                                remapping ={'rec':'rec_wait'}
                                ) 
            StateMachine.add('POINT_GUEST2',
                                Point(),
                                transitions={'succeeded':'GET_RECEPTION_WORLD2','aborted':'aborted','error':'error'},
                                remapping={'angle':'right_angle'}
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
                                transitions={'succeeded':'POINTSEAT2','error':'error'},
                                remapping ={'arm_waypoints':'arm_back_waypoints'}
                                ) 
            
            StateMachine.add('POINTSEAT2',
                                self.sm_PointSeat,
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping={'point_pos':'point_pos'})                                                
        intro_server = IntrospectionServer('RECEPTIONIST',self.RECEPTIONIST,'/SM_ROOT')
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
