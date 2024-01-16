#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_special.TidyUp_lib import *
from smach_special.gpsr import Wait_trace, RunNode
from smach_compose.compose import *
from smach_common.common import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

'''
Author: yishui
Date: 2022-08-29
LastEditTime: 2022-11-23 3:17:38
LastEditors: yishui
Description: Codes for TidyUp
FilePath: ~/catkin_ws/src/xm_smach/xm_smach/tests/yishui
'''

class Tidy():
    def __init__(self):
        rospy.init_node("TidyUp_Smach")
        rospy.on_shutdown(self.shutdown)
        rospy.logerr("Welcome to TidyUp!")
        self.smach_bool = False

        self.xm_Follow = Concurrence(outcomes = ['succeeded','aborted'],
                                 default_outcome = 'aborted',
                                 outcome_map={'succeeded':{'STOP':'stop'},
                                              'aborted':{'FOLLOW':'aborted'},
                                              'aborted':{'FOLLOW':'preempted'}},
                                 child_termination_cb = self.child_cb)   
        with self.xm_Follow:

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
                    print("8888888888888888888")
                    Concurrence.add('NAV',NavStack0(),remapping={'pos_xm':'pos_xm'})#跟随人
                    Concurrence.add('WAIT',Wait_trace())#以一定频率发移动指令
                StateMachine.add('META_NAV',
                                    self.meta_nav,
                                    transitions={'get_pos':'FIND','time_over':'FIND','aborted':'FIND'})
            Concurrence.add('FOLLOW',self.meta_follow)
            Concurrence.add('STOP',CheckStop())
            #Concurrence.add('RUNNODE',RunNode())#开启跟随人的图像节点    


        #-------PICK_UP-------#
        #RUNNODE_IMG-->GETNAME-->GET_POS-->NAV-->FIND-->POS_JUS-->NAV-->RUNNODE_IMG-->FIND-->POS_JUS-->PICK-->SPEAK
        #运行图像节点，获取物体名字和大物体位置后，nav到大物体位置，然后用图像找物体，然后用pos_jus得出适宜抓取的坐标，nav后继续循环一次
        #在此之后，进行抓取
        #如果上面的图像没有找到物体，则通过speak进行错误反馈
        self.xm_Pick_up = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['target'])    
        with self.xm_Pick_up: 
            # self.xm_Pick_up.userdata.name ='orange water'
            self.xm_Pick_up.userdata.distance = 0.9 #change this element from 0.7 to 0.2
            # 桌子高度？
            self.xm_Pick_up.userdata.table_depth = 0.04
            # self.xm_Pick_up.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
            self.xm_Pick_up.userdata.target=['orange water']
            self.xm_Pick_up.userdata.pick_pos=Pose()
            self.xm_Pick_up.userdata.up_down = 1
            target_camera_pose = PoseStamped()
            target_camera_pose.header.frame_id = "camera_link_up"
            target_camera_pose.pose.position.x = 0
            target_camera_pose.pose.position.y = 0
            target_camera_pose.pose.position.z = 0
            self.xm_Pick_up.userdata.target_camera_point=target_camera_pose
            self.xm_Pick_up.userdata.target_size=[0,0,0]
            self.xm_Pick_up.userdatatable_depth=0

            # self.xm_Pick_up.userdata.target_camera_point = Pose()
            #StateMachine.add('PICK_POS',
                                # ArmTrajectory(),#轨迹分析
            #                     transitions={'succeeded': 'FIND_1', 'error': 'error'},
            #                     remapping={'arm_waypoints':'pick_pose'})                                   
            
            # StateMachine.add('FIND_1',
            #                     GetObjectPosition(),#区分物体，识别物体
            #                     transitions={'succeeded': 'GETPICKPOSE', 'error': 'error'})

            # StateMachine.add('GETPICKPOSE',
            #                     GetPickPosBack(),  
            #                     transitions ={'succeeded':'NAVTOPICKPOS','error':'error'})       
            
            # StateMachine.add('NAVTOPICKPOS',
            #                     NavStack(),
            #                     transitions ={'succeeded':'TARGET_POINT_JUSFY','aborted':'NAVTOPICKPOS','error':'error'},
            #                     remapping ={"pos_xm":'pick_pos'})

            # StateMachine.add('FIND_2',
            #                     GetObjectPosition(),
            #                     transitions={'succeeded': 'TARGET_POINT_JUSFY', 'error': 'error'},
            #                     remapping={'target':'name'})   
                                                               
            # StateMachine.add('TARGET_POINT_JUSFY',
            #                     TargerPosReload(),
            #                     transitions={'succeeded': 'PICK', 'error': 'error'})

            StateMachine.add('PICK', 
                                ArmStack(),3
                                transitions={'succeeded': 'NAV_BACK','error': 'error'})
            self.xm_Pick_up.userdata.distance = -0.2
            StateMachine.add('NAV_BACK', 
                                GoAhead(),
                                transitions={'succeeded': 'succeeded','error': 'error'},
                                remapping={'move_len':'distance'})
            

        #--------GO--------#
        # self.xm_Nav = StateMachine(outcomes=['succeeded', 'aborted', 'error'],
        #                            input_keys=['target', 'current_task'])
        # with self.xm_Nav:
        #     self.xm_Nav.userdata.pose_xm = gpsr_target['livingroom']['pos']
        #     self.xm_Nav.userdata.turn_pose = Pose()
        #     self.xm_Nav.userdata.target_mode = 1

        #     # StateMachine.add('GETTARGET',
        #     #                  GetTarget(),  # 修改(获取)userdata.current_target
        #     #                  transitions={'succeeded': 'GO',
        #     #                               'aborted': 'aborted', 'error': 'error'},
        #     #                  remapping={'target': 'target', 'current_task': 'current_task',
        #     #                             'current_target': 'pos_xm', 'mode': 'target_mode'}  # 相当于将current_target赋值给Pos_xm
        #     #                  )
        #     StateMachine.add('GO',
        #                      NavStack(),  # 以pos_xm为目标移动
        #                      transitions={'succeeded': 'SPEAK',
        #                                   'aborted': 'GO', 'error': 'error'},
        #                      remapping={'pos_xm': 'pose_xm'}
        #                      )

        #     self.xm_Nav.userdata.sentences = 'I have arrived here'

        #     StateMachine.add('SPEAK',
        #                      Speak(),  # 使用语音的服务，开一个子进程，播放音频
        #                      transitions={
        #                          'succeeded': 'succeeded', 'error': 'error'},
        #                      remapping={'sentences': 'sentences'}
        #                      )

        #-------Put_down---------#
        self.xm_Put_down = StateMachine(outcomes=['succeeded', 'aborted', 'error'],
                                        input_keys=['target'])
        with self.xm_Put_down:
            self.xm_Put_down.userdata.commond = 0
            self.xm_Put_down.userdata.put_down_pos = gpsr_target['put_down']['pos']
            self.xm_Put_down.userdata.distance = 0.5  # change this element from 0.7 to 0.2
            self.xm_Put_down.userdata.table_depth = 0.04
            self.xm_Put_down.userdata.traget_size = [0.04, 0.065, 0.105]
            self.xm_Put_down.userdata.up_down = 1
            
            StateMachine.add('NAV',
                             NavStack(),  # 移动到livingroom
                             transitions={'succeeded': 'FIND_PUT_PLACE_1',
                                          'aborted': 'NAV', 'error': 'error'},
                             remapping={'pos_xm': 'put_down_pos'})

            StateMachine.add('FIND_PUT_PLACE_1',
                             GetPutPosition(),
                             transitions={'succeeded': 'GETPUTPOSE', 'aborted': 'FIND_PUT_PLACE_1', 'error': 'error'},)

            StateMachine.add('GETPUTPOSE',
                                GetPickPosBack_tidyup(),  
                                transitions ={'succeeded':'NAVTOPICKPOS','error':'error'})   

            StateMachine.add('NAVTOPICKPOS',
                                NavStack(),
                                transitions ={'succeeded':'PUT_DOWN','aborted':'NAVTOPICKPOS','error':'error'},
                                remapping ={"pos_xm":'pick_pos'})
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

            # StateMachine.add('TARGET_POINT_JUSFY',
            #                  TargerPosReload(),
            #                  transitions={'succeeded': 'PUT_DOWN', 'error': 'error'})

            StateMachine.add('PUT_DOWN',
                             GripperCommond(),  # 控制爪子（打开）
                             transitions={'succeeded': 'succeeded', 'error': 'error'})
         # 顶层状态机
        self.xm_TidyUp = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'])
        with self.xm_TidyUp:

            self.xm_TidyUp.userdata.target = list()
            self.xm_TidyUp.userdata.action = list()
            self.xm_TidyUp.userdata.task_num = 0
            self.xm_TidyUp.userdata.answer = "this is the answer"
            self.xm_TidyUp.userdata.current_task = -1
            self.xm_TidyUp.userdata.current_turn = 0
             # the max_num of the mission，因为任务绝对做不完，所以设一个比较大的值
            self.xm_TidyUp.userdata.turn = 100 
            self.xm_TidyUp.userdata.pos_xm_door = gpsr_target['livingroom']['pos']
            self.xm_TidyUp.userdata.waypoint = gpsr_target['exit_pos']['pos']
            self.xm_TidyUp.userdata.sentences = 'give me the mission please'  # for the Speak() state
            self.xm_TidyUp.userdata.gripper_release = 0.0
            self.xm_TidyUp.userdata.give_file = "give_me_the_mission.wav"
            self.xm_TidyUp.userdata.smsentences_release = "i will release the gripper,please grap it"
            self.xm_TidyUp.userdata.rec = 5.0

            self.xm_TidyUp.userdata.gesture=''
            StateMachine.add('RECEIVE_TASKS',
                            GetTask(),
                            transitions = {'succeeded':'GET_NEXT_TASK','aborted':'RECEIVE_TASKS','error':'error'},
                            remapping = {'target':'target','action':'action','task_num':'task_num','answer':'answer','gesture':'gesture'}
                            )
            StateMachine.add('CHECK_TURN',
                             CheckTurn(),
                             transitions={'succeeded': 'GO_OUT',
                                          'continue': 'RECEIVE_TASKS', 'error': 'error'}
                             )
            StateMachine.add('GO_OUT',
                             NavStack(),
                             transitions={'succeeded': 'succeeded',
                                          'aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'waypoint'}
                             )

            StateMachine.add('GET_NEXT_TASK',
                    NextDo(),
                    transitions = {'succeeded':'succeeded',
                                    'aborted':'GET_NEXT_TASK',
                                    'error':'error',
                                    'follow':'RUNNODE',#open the camera first
                                    'get':'PICK_UP',},
                    remapping = {'action':'action',
                                    'current_task':'current_task',
                                    'task_num':'task_num'}
                )
            StateMachine.add('RUNNODE',
                                    RunNode(),
                                    transitions={'succeeded':'FOLLOW','aborted':'RUNNODE'})
            StateMachine.add('FOLLOW',
                                self.xm_Follow,
                                transitions={'succeeded':'PICK_UP','aborted':'FOLLOW'})
            # StateMachine.add('CLOSE',
            #                     CloseKinect(),
            #                     transitions={'succeeded':'GET_NEXT_TASK','aborted':'GET_NEXT_TASK'})
            StateMachine.add('PICK_UP',
                             self.xm_Pick_up,
                             transitions={'succeeded': 'PUT_DOWN', 'aborted': 'PICK_UP', 'error': 'PICK_UP'})

            StateMachine.add('PUT_DOWN',
                             self.xm_Put_down,
                             transitions={'succeeded': 'CHECK_TURN', 'aborted': 'PUT_DOWN'})

        intro_server = IntrospectionServer(
            'xm_tidyup', self.xm_TidyUp, '/XM_ROOT')
        intro_server.start()
        out = self.xm_TidyUp.execute()
        intro_server.stop()

    def shutdown(self):
        if self.smach_bool == True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')


        #return True down
    def child_cb(self,outcome_map):
        if outcome_map['STOP'] == 'stop':
            rospy.logwarn('---------get the signal of stop,stop tracing---------')
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill','-9',pid],shell=True)
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
            print(outcome_map)
            return False




    # use for concurrence
if __name__ == "__main__":
    try:
        Tidy()
    except:
        rospy.logerr(e)
