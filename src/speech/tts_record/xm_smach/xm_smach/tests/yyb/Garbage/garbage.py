#!/usr/bin/env python
# encoding:utf8
from os import WIFSTOPPED
import os
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_special.gpsr import * 
from smach_special.garbage import * 
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

class Gpsr():
    def __init__(self):
        rospy.init_node("Gpsr_Smach")
        rospy.on_shutdown(self.shutdown)
        rospy.logerr("Welcome to GPSR!")
        # empty pid file
        dir_path = '/home/xm/vision_pid/'
        for file in os.listdir(dir_path):
            with open(os.path.join(dir_path,file),'w') as f:
                f.write('')
        print('Empty PID Files Success!')
        self.smach_bool = False
        
        #-------Enter_Room-------#
        #任务开始，首先要识别是否门已经打开，如果门已经打开，自动进入房间
        #首先进入的房间一定是Livingroom
        self.xm_EnterRoom = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.xm_EnterRoom:
            self.xm_EnterRoom.userdata.start_waypoint = gpsr_target['instruction_pos']['pos']
            self.xm_EnterRoom.userdata.rec = 3.0
            

            StateMachine.add('DOORDETECT',
                                DoorDetect(),
                                transitions={'succeeded':'WAIT','aborted':'DOORDETECT','error':'error'})
            
            StateMachine.add('WAIT',
                                Wait(),#等待三秒
                                remapping ={'rec':'rec'},
                                transitions ={'succeeded':'NAV','error':'error'})              
            
            StateMachine.add('NAV',
                                NavStack(),#移动到livingroom
                                transitions={'succeeded':'succeeded','aborted':'NAV','error':'error'},
                                remapping = {'pos_xm':'start_waypoint'})
        
        #--------GO--------#
        self.xm_Nav = StateMachine(outcomes = ['succeeded', 'aborted', 'error'],
                                input_keys = ['target', 'current_task'])
        with self.xm_Nav:
            self.xm_Nav.userdata.pose_xm = Pose()
            self.xm_Nav.userdata.turn_pose = Pose()
            self.xm_Nav.userdata.target_mode = 1

            StateMachine.add('GETTARGET',
                                GetTargetG(),#修改(获取)userdata.current_target
                                transitions = {'succeeded':'GO','error':'error'},
                                remapping = {'target':'target','current_task':'current_task','current_target':'pos_xm','mode':'target_mode'}#相当于将current_target赋值给Pos_xm
                            )

            StateMachine.add('GO',
                                NavStack(),#以pos_xm为目标移动
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'pos_xm':'pos_xm'}
                            )

            self.xm_Nav.userdata.sentences = 'I have arrived here'

            StateMachine.add('SPEAK',
                                Speak(),#使用语音的服务，开一个子进程，播放音频
                                transitions = {'succeeded':'succeeded','error':'error'},
                                remapping = {'sentences':'sentences'}
                            )


        self.xm_Nav_1 = StateMachine(outcomes = ['succeeded', 'aborted', 'error'],
                                input_keys = ['target', 'current_task'])
        with self.xm_Nav_1:
            self.xm_Nav_1.userdata.pose_xm = Pose()
            self.xm_Nav_1.userdata.turn_pose = Pose()
            self.xm_Nav_1.userdata.target_mode = 1

            StateMachine.add('GETTARGET_1',
                                GetTargetG_1(),#修改(获取)userdata.current_target
                                transitions = {'succeeded':'GO_1','error':'error'},
                                remapping = {'target':'target','current_task':'current_task','current_target':'pos_xm','mode':'target_mode'}#相当于将current_target赋值给Pos_xm
                            )
            StateMachine.add('GO_1',
                                NavStack(),#以pos_xm为目标移动
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'pos_xm':'pos_xm'}
                            )

        self.xm_Nav_2 = StateMachine(outcomes = ['succeeded', 'aborted', 'error'],
                                input_keys = ['target', 'current_task'])
        with self.xm_Nav_2:
            self.xm_Nav_2.userdata.pose_xm = Pose()
            self.xm_Nav_2.userdata.turn_pose = Pose()
            self.xm_Nav_2.userdata.target_mode = 1

            StateMachine.add('GETTARGET_2',
                                GetTargetG_2(),#修改(获取)userdata.current_target
                                transitions = {'succeeded':'GO_2','error':'error'},
                                remapping = {'target':'target','current_task':'current_task','current_target':'pos_xm','mode':'target_mode'}#相当于将current_target赋值给Pos_xm
                            )
            StateMachine.add('GO_2',
                                NavStack(),#以pos_xm为目标移动
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'pos_xm':'pos_xm'}
                            )

        self.xm_Nav_Object = StateMachine(outcomes = ['succeeded', 'aborted', 'error'],
                                input_keys = ['targetName','targetType'])
        with self.xm_Nav_Object:
            self.xm_Nav_Object.userdata.pose_xm = Pose()
            self.xm_Nav_Object.userdata.turn_pose = Pose()
            self.xm_Nav_Object.userdata.target_mode = 1

            StateMachine.add('GET_OBJECT',
                                GetObjectName(),#修改(获取)userdata.current_target
                                transitions = {'succeeded':'GO_OBJECT','error':'error'},
                                remapping = {'target':'targetType','current_target':'pos_xm'}#相当于将current_target赋值给Pos_xm
                            )

            StateMachine.add('GO_OBJECT',
                                NavStack(),#以pos_xm为目标移动
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'pos_xm':'pos_xm'}
                            )       



        #----FIND----#
        self.xm_Find_1 = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['target','current_task','character','gesture','objectName','objectType','check_first'],
                                output_keys=['character','objectName','objectType','check_first'])
        
        with self.xm_Find_1:
            #xm_Person--RUNNODE-->WAIT-->GET_PEOPLE_POS-->NAV-->SPEAK-->CLOSE_KINECT
            
            self.xm_Pos_1 = StateMachine(outcomes=['succeeded','aborted','error'],
                                    input_keys = ['check_first','objectName','objectType'],
                                    output_keys=['objectName','objectType','check_first'])
            with self.xm_Pos_1:
                self.xm_Pos_1.userdata.pose = Pose()
                self.xm_Pos_1.userdata.mode_1 =1
                self.xm_Pos_1.userdata.nav_pos = gpsr_target['speaker']['pos']     
                #self.xm_Pos_1.userdata.objectName = ''
                self.xm_Pos_1.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
                self.xm_Pos_1.userdata.take_back_pose = [ [0,-1.4,3.14,0,0,0] ]
                self.xm_Pos_1.userdata.table_depth = 0.04
                self.xm_Pos_1.userdata.traget_size = [0.04, 0.065, 0.105]
                self.xm_Pos_1.userdata.distance = 0.3

                StateMachine.add('FIND_1',
                                GetObjectPositionG_new_1(),#区分物体，识别物体
                                transitions={'succeeded': 'GETPICKPOSE', 'error': 'error'})

                StateMachine.add('GETPICKPOSE',
                                GetPickPos(),
                                transitions ={'succeeded':'NAVTOPICKPOS','error':'error'})       
            
                StateMachine.add('NAVTOPICKPOS',
                                NavStack(),
                                transitions ={'succeeded':'SPEAK','aborted':'NAVTOPICKPOS','error':'error'},
                                remapping ={"pos_xm":'pick_pos'})
             
                self.xm_Pos_1.userdata.sentences = 'Please give me '
                # self.xm_Pos_1.userdata.back_target = 'speaker'
                StateMachine.add('SPEAK',
                                    SpeakObjectName(),
                                    transitions = {'succeeded':'CLOSE','error':'error'},
                                    remapping ={'sentences':'sentences','objectName':'objectName'})
                
                self.xm_Pos_1.userdata.commond =1
                rospy.sleep(5.0)
                StateMachine.add('CLOSE',
                                GripperCommond(),#控制爪子（bihe）
                                transitions ={'succeeded':'NAV_BACK','error':'error'})
                self.xm_Pos_1.userdata.distance = -0.2
               
                StateMachine.add('NAV_BACK', 
                                GoAhead(),
                                transitions={'succeeded': 'succeeded','error': 'error'},
                                remapping={'move_len':'distance'})
                                        
            StateMachine.add('POS_1',
                                self.xm_Pos_1,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task'})

        #----FIND----#
        self.xm_Find_2 = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['target','current_task','character','gesture','objectName','objectType','check_second'],
                                output_keys=['character','objectName','objectType','check_second'])
        
        with self.xm_Find_2:
            #xm_Person--RUNNODE-->WAIT-->GET_PEOPLE_POS-->NAV-->SPEAK-->CLOSE_KINECT
            
            self.xm_Pos_2 = StateMachine(outcomes=['succeeded','aborted','error'],
                                    input_keys = ['check_second','objectName','objectType'],
                                    output_keys=['objectName','objectType','check_second'])
            with self.xm_Pos_2:
                self.xm_Pos_2.userdata.pose = Pose()
                self.xm_Pos_2.userdata.mode_1 =1
                self.xm_Pos_2.userdata.nav_pos = gpsr_target['speaker']['pos']     
                #self.xm_Pos_1.userdata.objectName = ''
                self.xm_Pos_2.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
                self.xm_Pos_2.userdata.take_back_pose = [ [0,-1.4,3.14,0,0,0] ]
                self.xm_Pos_2.userdata.table_depth = 0.04
                self.xm_Pos_2.userdata.traget_size = [0.04, 0.065, 0.105]
                self.xm_Pos_2.userdata.distance = 0.3

                StateMachine.add('FIND_1',
                                GetObjectPositionG_new_2(),#区分物体，识别物体
                                transitions={'succeeded': 'GETPICKPOSE', 'error': 'error'})

                StateMachine.add('GETPICKPOSE',
                                GetPickPos(),
                                transitions ={'succeeded':'NAVTOPICKPOS','error':'error'})       
            
                StateMachine.add('NAVTOPICKPOS',
                                NavStack(),
                                transitions ={'succeeded':'SPEAK','aborted':'NAVTOPICKPOS','error':'error'},
                                remapping ={"pos_xm":'pick_pos'})
             
                self.xm_Pos_2.userdata.sentences = 'Please give me '
                # self.xm_Pos_1.userdata.back_target = 'speaker'
                StateMachine.add('SPEAK',
                                    SpeakObjectName(),
                                    transitions = {'succeeded':'CLOSE','error':'error'},
                                    remapping ={'sentences':'sentences','objectName':'objectName'})
                
                self.xm_Pos_2.userdata.commond =1
                StateMachine.add('CLOSE',
                                GripperCommond(),#控制爪子（bihe）
                                transitions ={'succeeded':'NAV_BACK','error':'error'})
                self.xm_Pos_2.userdata.distance = -0.2
               
                StateMachine.add('NAV_BACK', 
                                GoAhead(),
                                transitions={'succeeded': 'succeeded','error': 'error'},
                                remapping={'move_len':'distance'})
                                        
            StateMachine.add('POS_2',
                                self.xm_Pos_2,
                                transitions = {'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping = {'target':'target','current_task':'current_task'})


        #-------Put_down---------#
        self.xm_Put_down = StateMachine(outcomes =['succeeded','aborted','error'])
        with self.xm_Put_down:    
            self.xm_Put_down.userdata.commond =0
            
            self.xm_Put_down.userdata.take_back_pose_before = [ [0,-1.47,1.67,0,0.24,0] ]
            self.xm_Put_down.userdata.take_back_pose_after = [ [0,-1.47,3.14,0,0,0] ]
            
            StateMachine.add('PUT_DOWN', 
                                ArmTrajectory_Before(),
                                transitions={'succeeded': 'OPEN','error': 'error'},
                                remapping={'arm_waypoints':'take_back_pose_before'})
            rospy.sleep(10)
            StateMachine.add('OPEN',
                                GripperCommond(),#控制爪子（打开）
                                transitions ={'succeeded':'BACK_POS','error':'error'})
            #rospy.sleep(2)
            StateMachine.add('BACK_POS',
                               ArmTrajectory_After(),
                               transitions={'succeeded': 'succeeded', 'error': 'error'},
                               remapping={'arm_waypoints':'take_back_pose_after'})

         #顶层状态机
        self.xm_GPSR = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.xm_GPSR:

            self.xm_GPSR.userdata.target = list()
            self.xm_GPSR.userdata.action = list()
            self.xm_GPSR.userdata.task_num = 0
            self.xm_GPSR.userdata.answer = "this is the answer"
            self.xm_GPSR.userdata.current_task = -1
            self.xm_GPSR.userdata.current_turn = 1
            self.xm_GPSR.userdata.turn = 5              #the max_num of the mission
            self.xm_GPSR.userdata.pos_xm_door = gpsr_target['livingroom']['pos']
            self.xm_GPSR.userdata.waypoint = gpsr_target['dining_room_exit']['pos']
            self.xm_GPSR.userdata.sentences = 'give me the mission please' #for the Speak() state
            self.xm_GPSR.userdata.gripper_release = 0.0
            self.xm_GPSR.userdata.give_file = "give_me_the_mission.wav"
            self.xm_GPSR.userdata.sentences_release = "i will release the gripper,please grap it"
            self.xm_GPSR.userdata.rec = 5.0
            self.xm_GPSR.userdata.character = ''
            self.xm_GPSR.userdata.gesture = ''
            self.xm_GPSR.userdata.count_num = 0
            self.xm_GPSR.userdata.objectName = ''
            self.xm_GPSR.userdata.objectType = ''
            self.xm_GPSR.userdata.targetName = ''
            self.xm_GPSR.userdata.targetType = ''
            self.xm_GPSR.userdata.check_first = 0
            self.xm_GPSR.userdata.check_second = 0
            # StateMachine.add('ENTERROOM',
            #                      self.xm_EnterRoom,
            #                      transitions ={'succeeded':'RECEIVE_TASKS','aborted':'RECEIVE_TASKS','error':'error'})
            
            
            # StateMachine.add('SPEAK_RESTART',
            #                 SpeakWAV(),
            #                 transitions = {'succeeded':'RECEIVE_TASKS','aborted':'aborted','error':'error'},
            #                 remapping = {'filename':'give_file'}
            #                 )

            StateMachine.add('RECEIVE_TASKS',
                            GetTaskG(),
                            transitions = {'succeeded':'GET_NEXT_TASK','aborted':'RECEIVE_TASKS','error':'error'},
                            remapping = {'target':'target','action':'action','task_num':'task_num','answer':'answer','gesture':'gesture'}
                            )
            
            StateMachine.add('GET_NEXT_TASK',
                                NextDoG(),
                                transitions = {'succeeded':'BACK_DOOR',
                                                'aborted':'RECEIVE_TASKS',
                                                'error':'error',
                                                'go':'GO_ROOM',
                                                },
                                remapping = {'action':'action',
                                              'current_task':'current_task',
                                              'task_num':'task_num'}
                            )
            StateMachine.add('BACK_DOOR',
                                NavStack(),
                                transitions = {'succeeded':'CHECK_TURN','aborted':'BACK_DOOR','error':'error'},
                                remapping = {'pos_xm':'pos_xm_door'}
                            )

            # StateMachine.add('CHECK_TURN',
            #                     CheckTurn(),
            #                     transitions = {'succeeded':'GO_OUT','continue':'SPEAK_RESTART','error':'error'}
            #                 )
            StateMachine.add('CHECK_TURN',
                CheckTurnG(),
                transitions = {'succeeded':'GO_OUT','move1':'GO_ROOM','move2':'DETECT_1','move3':'DETECT_2','error':'error'}
            )
            StateMachine.add('GO_OUT',
                                NavStack(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={"pos_xm":'waypoint'}
                            )
        
            StateMachine.add('GO_ROOM',
                                self.xm_Nav,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'FIND_1','aborted':'GO_ROOM'})
            # StateMachine.add('CLOSE',
            #                     CloseKinect(),
            #                     transitions={'succeeded':'GET_NEXT_TASK','aborted':'GET_NEXT_TASK'})
            
            StateMachine.add('FIND_1',
                                self.xm_Find_1,
                                transitions={'succeeded':'GO_PUT_DOWN','aborted':'DETECT_1'})
            
            StateMachine.add('DETECT_1',
                                self.xm_Nav_1,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'FIND_2','aborted':'DETECT_1'})

            StateMachine.add('FIND_2',
                                self.xm_Find_2,
                                transitions={'succeeded':'GO_PUT_DOWN','aborted':'DETECT_2'})

            StateMachine.add('DETECT_2',
                                self.xm_Nav_2,
                                remapping ={'target':'target','current_task':'current_task'},
                                transitions={'succeeded':'FIND_3','aborted':'DETECT_2'})
            StateMachine.add('FIND_3',
                                self.xm_Find_2,
                                transitions={'succeeded':'GO_PUT_DOWN','aborted':'GO_ROOM'})

            StateMachine.add('GO_PUT_DOWN',
                                self.xm_Nav_Object,
                                remapping ={'targetName':'objectName','targetType':'objectType'},
                                transitions={'succeeded':'PUT_DOWN','aborted':'GO_PUT_DOWN'})

            StateMachine.add('PUT_DOWN',
                                self.xm_Put_down,
                                transitions={'succeeded':'CHECK_TURN','aborted':'PUT_DOWN'}) 
            
        
        intro_server = IntrospectionServer('xm_gpsr',self.xm_GPSR,'/XM_ROOT')
        intro_server.start()
        out = self.xm_GPSR.execute()
        intro_server.stop()


    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

    def child_cb_guide(self,outcome_map):
        if outcome_map['NAV_GUIDE'] == 'succeeded':
            rospy.logwarn('---------I have arrive there---------')
            
            return True
        elif outcome_map['NAV_GUIDE'] == 'aborted':
            rospy.logerr('the stop state meet error!')
            return True
            
        if outcome_map['SPEAK_FOLLOW'] == 'aborted':
            rospy.logerr('the speak_follow state meet error!')
            return True

        return False

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
        Gpsr()
    except:
        rospy.logerr(e)
        #print("7777777")
