#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_special.TidyUp import * 
from smach_compose.compose import * 
from smach_common.common import * 
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

'''
Author: yyb
Date: 2021-01-21 
LastEditTime: 2021-01-21 19:17:27
LastEditors: yyb
Description: Codes for GPSR
FilePath: /xm_smach/tests/yyb/GPSR
'''

class Gpsr():
    def __init__(self):
        rospy.init_node("Gpsr_Smach")
        rospy.on_shutdown(self.shutdown)
        rospy.logerr("Welcome to GPSR!")
        self.smach_bool = False

        #-------PICK_UP-------#
        #RUNNODE_IMG-->GETNAME-->GET_POS-->NAV-->FIND-->POS_JUS-->NAV-->RUNNODE_IMG-->FIND-->POS_JUS-->PICK-->SPEAK
        #运行图像节点，获取物体名字和大物体位置后，nav到大物体位置，然后用图像找物体，然后用pos_jus得出适宜抓取的坐标，nav后继续循环一次
        #在此之后，进行抓取
        #如果上面的图像没有找到物体，则通过speak进行错误反馈
        self.xm_Pick_up = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['target','current_task'])    
        with self.xm_Pick_up: 
            self.xm_Pick_up.userdata.name =''
            self.xm_Pick_up.userdata.target_mode =0
            self.xm_Pick_up.userdata.objmode = -1
            self.xm_Pick_up.userdata.go_counter = 2
            self.xm_Pick_up.userdata.distance = 0.5 #change this element from 0.7 to 0.2
            self.xm_Pick_up.userdata.table_depth = 0.04
            self.xm_Pick_up.userdata.traget_size = [0.04, 0.065, 0.105]
            self.xm_Pick_up.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
            self.xm_Pick_up.userdata.take_back_pose = [ [0,0.026,3.14,0,3.14,0] ]
            self.xm_Pick_up.userdata.mode_1 =1
            self.xm_Pick_up.userdata.object_pos = PointStamped()
            
            StateMachine.add('PICK_POS',
                                 ArmTrajectory(),#轨迹分析
                                 transitions={'succeeded': 'FIND_1', 'error': 'error'},
                                 remapping={'arm_waypoints':'pick_pose'})                                   
            
            StateMachine.add('FIND_1',
                                GetTargetPosition(),#区分物体，识别物体
                                transitions={'succeeded': 'GETPICKPOSE', 'error': 'error'})

            StateMachine.add('GETPICKPOSE',
                                GetPickPos(),
                                transitions ={'succeeded':'NAVTOPICKPOS','error':'error'})       
            
            StateMachine.add('NAVTOPICKPOS',
                                NavStack(),
                                transitions ={'succeeded':'FIND_2','aborted':'NAVTOPICKPOS','error':'error'},
                                remapping ={"pos_xm":'pick_pos'})

            StateMachine.add('FIND_2',
                                GetTargetPosition(),
                                transitions={'succeeded': 'TARGET_POINT_JUSFY', 'error': 'error'})   
                                                               
            StateMachine.add('TARGET_POINT_JUSFY',
                                TargerPosReload(),
                                transitions={'succeeded': 'PICK', 'error': 'error'})

            StateMachine.add('PICK', 
                                ArmStack(),
                                transitions={'succeeded': 'NAV_BACK','error': 'error'})
            
            self.xm_Pick_up.userdata.distance = -0.2

            StateMachine.add('NAV_BACK', 
                                GoAhead(),
                                transitions={'succeeded': 'BACK_POS','error': 'error'},
                                remapping={'move_len':'distance'})
            
            #TODO arm back pose
            StateMachine.add('BACK_POS',
                                ArmTrajectory(),
                                transitions={'succeeded': 'succeeded', 'error': 'error'},
                                remapping={'arm_waypoints':'take_back_pose'})

        #--------GO--------#
        self.xm_Nav = StateMachine(outcomes = ['succeeded', 'aborted', 'error'],
                                input_keys = ['target', 'current_task'])
        with self.xm_Nav:
            self.xm_Nav.userdata.pose_xm = Pose()
            self.xm_Nav.userdata.turn_pose = Pose()
            self.xm_Nav.userdata.target_mode = 1

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

            self.xm_Nav.userdata.sentences = 'I have arrived here'

            StateMachine.add('SPEAK',
                                Speak(),#使用语音的服务，开一个子进程，播放音频
                                transitions = {'succeeded':'succeeded','error':'error'},
                                remapping = {'sentences':'sentences'}
                            )    

        #-------Put_down---------#
        self.xm_Put_down = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys = ['target'])
        with self.xm_Put_down:    
            self.xm_Put_down.userdata.commond =0
            self.xm_Put_down.userdata.put_down_pos = gpsr_target['put_down']['pos']
            self.xm_Put_down.userdata.put_pose = [ [0,-1.5,3.14,0,0,0] ]
            self.xm_Put_down.userdata.name =''
            self.xm_Put_down.userdata.target_mode =0
            self.xm_Put_down.userdata.objmode = -1
            self.xm_Put_down.userdata.go_counter = 2
            self.xm_Put_down.userdata.distance = 0.5 #change this element from 0.7 to 0.2
            self.xm_Put_down.userdata.table_depth = 0.04
            self.xm_Put_down.userdata.traget_size = [0.04, 0.065, 0.105]
            self.xm_Put_down.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
            self.xm_Put_down.userdata.take_back_pose = [ [0,0.026,3.14,0,3.14,0] ]
            self.xm_Put_down.userdata.mode_1 =1
            self.xm_Put_down.userdata.object_pos = PointStamped()
            StateMachine.add('NAV',
                                NavStack(),#移动到livingroom
                                transitions={'succeeded':'FIND_PUT_PLACE_1','aborted':'NAV','error':'error'},
                                remapping = {'pos_xm':'put_down_pos'})
            
            StateMachine.add('FIND_PUT_PLACE_1',
                                GetPutPosition(),
                                transitions={'succeeded':'TARGET_POINT_JUSFY','aborted':'FIND_PUT_PLACE_1','error':'error'},)
            
            # StateMachine.add('GETPUTPOSE',
            #                     GetPutPos(),
            #                     transitions ={'succeeded':'NAVTOPICKPOS','error':'error'}) 
            
            # StateMachine.add('NAVTOPICKPOS',
            #                     NavStack(),
            #                     transitions ={'succeeded':'FIND_2','aborted':'NAVTOPICKPOS','error':'error'},
            #                     remapping ={"pos_xm":'put_pos'})
            # StateMachine.add('FIND_PUT_PLACE_2',
            #                     GetTargetPosition(),
            #                     transitions={'succeeded': 'TARGET_POINT_JUSFY', 'error': 'error'})   
                                                               
            StateMachine.add('TARGET_POINT_JUSFY',
                                TargerPosReload(),
                                transitions={'succeeded': 'PUT_DOWN', 'error': 'error'})

            StateMachine.add('PUT_DOWN',
                                PutDown(),#控制爪子（打开）
                                transitions ={'succeeded':'succeeded','error':'error'})
         #顶层状态机
        self.xm_GPSR = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.xm_GPSR:

            self.xm_GPSR.userdata.target = list()
            self.xm_GPSR.userdata.action = list()
            self.xm_GPSR.userdata.task_num = 0
            self.xm_GPSR.userdata.answer = "this is the answer"
            self.xm_GPSR.userdata.current_task = -1
            self.xm_GPSR.userdata.current_turn = -1
            self.xm_GPSR.userdata.turn = 5              #the max_num of the mission
            self.xm_GPSR.userdata.pos_xm_door = gpsr_target['livingroom']['pos']
            self.xm_GPSR.userdata.waypoint = gpsr_target['exit_pos']['pos']
            self.xm_GPSR.userdata.sentences = 'give me the mission please' #for the Speak() state
            self.xm_GPSR.userdata.gripper_release = 0.0
            self.xm_GPSR.userdata.give_file = "give_me_the_mission.wav"
            self.xm_GPSR.userdata.smsentences_release = "i will release the gripper,please grap it"
            self.xm_GPSR.userdata.rec = 5.0

            StateMachine.add('RECEIVE_TASKS',#get the room to clean
                            GetTask(),
                            transitions = {'succeeded':'GO','aborted':'RECEIVE_TASKS','error':'error'},
                            remapping = {'target':'target','action':'action','task_num':'task_num','answer':'answer'}
                            )

            # StateMachine.add('CHECK_TURN',
            #                     CheckTurn(),
            #                     transitions = {'succeeded':'GO_OUT','continue':'SPEAK_RESTART','error':'error'}
            #                 )
            StateMachine.add('CHECK_TURN',
                CheckTurn(),
                transitions = {'succeeded':'GO_OUT','continue':'GO','error':'error'}
            )
            StateMachine.add('GO_OUT',
                                NavStack(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={"pos_xm":'waypoint'}
                            )
        
            StateMachine.add('GO',
                                self.xm_Nav,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'PICK_UP_FOOD','aborted':'GO'})
            # StateMachine.add('CLOSE',
            #                     CloseKinect(),
            #                     transitions={'succeeded':'GET_NEXT_TASK','aborted':'GET_NEXT_TASK'})
            StateMachine.add('PICK_UP_FOOD',
                                self.xm_Pick_up,
                                remapping ={'target':'target_food_juice','current_task':'current_task'},
                                transitions={'succeeded':'PUT_DOWN_FOOD','aborted':'PICK_UP_FRUITS','error':'PICK_UP_FRUITS'})
            
            StateMachine.add('PUT_DOWN_FOOD',
                                self.xm_Put_down,
                                transitions={'succeeded':'CHECK_TURN','aborted':'PUT_DOWN_FOOD'}) 

            StateMachine.add('PICK_UP_FRUITS',
                                self.xm_Pick_up,
                                remapping ={'target':'target_fruits','current_task':'current_task'},
                                transitions={'succeeded':'PUT_DOWN_FRUITS','aborted':'PICK_UP_OTHERS','error':'PICK_UP_OTHERS'})

            StateMachine.add('PUT_DOWN_FRUITS',
                                self.xm_Put_down,
                                transitions={'succeeded':'CHECK_TURN','aborted':'PUT_DOWN_FRUITS'}) 

            StateMachine.add('PICK_UP_OTHERS',
                                self.xm_Pick_up,
                                remapping ={'target':'target_others','current_task':'current_task'},
                                transitions={'succeeded':'PUT_DOWN_OTHERS','aborted':'aborted','error':'error'})

            StateMachine.add('PUT_DOWN_OTHERS',
                                self.xm_Put_down,
                                transitions={'succeeded':'CHECK_TURN','aborted':'PUT_DOWN_OTHERS'}) 
        
        intro_server = IntrospectionServer('xm_gpsr',self.xm_GPSR,'/XM_ROOT')
        intro_server.start()
        out = self.xm_GPSR.execute()
        intro_server.stop()


    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')
        
    # use for concurrence
if __name__ == "__main__":
    try:
        Gpsr()
    except:
        rospy.logerr(e)
