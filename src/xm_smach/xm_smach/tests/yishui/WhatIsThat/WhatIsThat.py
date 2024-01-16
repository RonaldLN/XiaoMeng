#!/usr/bin/env python
# encoding:utf8
from os import WIFSTOPPED
import os
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_special.gpsr import * 
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
Description: Codes for whatsthat
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

class GetObjectName(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target','objectName'],
                       output_keys=['target_camera_point' ,'table_depth'])
        self.xm_findobject = rospy.ServiceProxy('findObjectName', xm_find_object)


    def execute(self, userdata):
        try:
        #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        #     rospy.sleep(2.0)
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_find_object.py','-d','true'] ,shell =False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w+') as f:
                if f.read() == '':
                    print('-------------------')
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('-------------------')
                    f.write(str(a.pid))
            rospy.sleep(2.0)
            print a.poll()
            if a.returncode != None:
                a.wait()
                return 'aborted'
        except:
             rospy.logerr('No param specified')
             return 'error'
        
        goal = Point()
        

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)

            req = xm_find_objectRequest()

            obj=str(userdata.target)
            L=obj.split("_")
            if "three_" in obj:
                req.name = "_".join(L[2:])
                req.adj = "three_"+L[1]
            elif L[0] in ["biggest","largest","thinnest","smallest","heaviest","lightest"]:
                req.name = "_".join(L[1:])
                req.adj = L[0]
            
            rospy.logwarn(req)

            
            res = self.xm_findobject.call(req)
            userdata.objectName = res.name   

            if res.position.point.x != 0 or res.position.point.y != 0 or res.position.point.z != 0:
                rospy.loginfo("find object!")
        
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        if res.position.point.x == -10.000 or res.position.point.x == 10.000 or res.position.point.x == 5:
            rospy.logerr('find nothing')
            self.killPro()
            return 'aborted'


        rospy.logwarn(res.position)

        userdata.target_camera_point = res.position
        self.killPro()
        return 'succeeded'

    def killPro(self):
        try:
            # pid_str = subprocess.check_output('ps -aux | grep mrsupw_detect_object.py' , shell= True)
            # pid_str1 = pid_str.splitlines()[0].split()[1]
            # rospy.logwarn(pid_str1)
            # subprocess.call('kill -9 '+pid_str1 , shell = True)

            pid = get_pid("people_tracking")
            subprocess.Popen(['kill','-9',pid],shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                f.write('')
                
        except Exception,e:
            rospy.logerr('No such process ')

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
            self.xm_EnterRoom.userdata.start_waypoint = gpsr_target['livingroom']['pos']
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
       
        #----FOLLOW-----#
        #xm_Follow包括两个并发状态，FOLLOW和STOP
        #STOP用于语音获取stop信号
        #FOLLOW包装了FindPeople().find_people_监视器和meta_nav
        #当找到人需要移动时，状态转移到meta_nav
        #meta_nav后,再继续进入监视状态
        #meta_nav是一个并发状态机，包装了NAV和WAIT
        #WAIT用于记录时间引发time_over
        #nav成功或中止后，与time_over一起进入继续找人状态
        
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
                    Concurrence.add('NAV',NavStack0(),remapping={'pos_xm':'pos_xm'})#跟随人
                    Concurrence.add('WAIT',Wait_trace())#以一定频率发移动指令
                StateMachine.add('META_NAV',
                                    self.meta_nav,
                                    transitions={'get_pos':'FIND','time_over':'FIND','aborted':'FIND'})
            Concurrence.add('FOLLOW',self.meta_follow)
            Concurrence.add('STOP',CheckStop())
            #Concurrence.add('RUNNODE',RunNode())#开启跟随人的图像节点            

        #----FIND----#        
        self.xm_Find = StateMachine(outcomes = ['succeeded','aborted','error'],
                                input_keys = ['target','current_task','character','gesture'],
                                output_keys=['character'])
        
        with self.xm_Find:
            
                self.xm_Find.userdata.pose = Pose()
                self.xm_Find.userdata.mode_1 =1
                self.xm_Find.userdata.nav_pos = gpsr_target['speaker']['pos']     
                self.xm_Find.userdata.objectName = ''
                self.xm_Find.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
                self.xm_Find.userdata.take_back_pose = [ [0,-1.4,3.14,0,0,0] ]
                self.xm_Find.userdata.table_depth = 0.04
                self.xm_Find.userdata.traget_size = [0.04, 0.065, 0.105]
                self.xm_Find.userdata.distance = 0.5

                StateMachine.add('FIND_1',
                                GetObjectName(),#区分物体，识别物体
                                transitions={'succeeded': 'GETPICKPOSE', 'error': 'error'})

                # self.xm_Pos.userdata.back_target = 'speaker'
                StateMachine.add('SPEAK',
                                    SpeakObjectName(),
                                    transitions = {'succeeded':'BACK_POS','error':'error'},
                                    remapping ={'sentences':'sentences','objectName':'objectName'})
                
                                                        
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
            self.xm_GPSR.userdata.sentences_follow = "Now I will follow you, please say stop when you want me to recognize the object"
            self.xm_GPSR.userdata.sentences_find = "Now I will recognize the object"
            self.xm_GPSR.userdata.rec = 5.0
            self.xm_GPSR.userdata.character = ''
            self.xm_GPSR.userdata.gesture = ''
            self.xm_GPSR.userdata.count_num = 0

            # StateMachine.add('ENTERROOM',
            #                      self.xm_EnterRoom,
            #                      transitions ={'succeeded':'RECEIVE_TASKS','aborted':'RECEIVE_TASKS','error':'error'})
            
            
            # StateMachine.add('SPEAK_RESTART',
            #                 SpeakWAV(),
            #                 transitions = {'succeeded':'RECEIVE_TASKS','aborted':'aborted','error':'error'},
            #                 remapping = {'filename':'give_file'}
            #                 )
            StateMachine.add('SPEAK_FOLLOW',
                                Speak(),#使用语音的服务，开一个子进程，播放音频
                                transitions = {'succeeded':'RunNode','error':'error'},
                                remapping = {'sentences':'sentences_follow'}
                            )
            StateMachine.add('RUNNODE',
                                    RunNode(),
                                    transitions={'succeeded':'FOLLOW','aborted':'RUNNODE'})
            StateMachine.add('FOLLOW',
                                self.xm_Follow,
                                transitions={'succeeded':'GET_NEXT_TASK','aborted':'FOLLOW'})
            StateMachine.add('SPEAK_FIND',
                                Speak(),#使用语音的服务，开一个子进程，播放音频
                                transitions = {'succeeded':'RunNode','error':'error'},
                                remapping = {'sentences':'sentences_find'}
                            )
            StateMachine.add('FIND',
                                self.xm_Find,
                                remapping ={'target':'target','current_task':'current_task','gesture':'gesture','character':'character'},
                                transitions={'succeeded':'CHECK_TURN','aborted':'FIND'})

            StateMachine.add('BACK_DOOR',
                                NavStack(),
                                transitions = {'succeeded':'CHECK_TURN','aborted':'BACK_DOOR','error':'error'},
                                remapping = {'pos_xm':'pos_xm_door'}
                            )

            StateMachine.add('CHECK_TURN',
                                CheckTurn(),
                                transitions = {'succeeded':'GO_OUT','continue':'SPEAK_RESTART','error':'error'}
                            )
            
            StateMachine.add('GO_OUT',
                                NavStack(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={"pos_xm":'waypoint'}
                            )
                
        intro_server = IntrospectionServer('xm_gpsr',self.xm_GPSR,'/XM_ROOT')
        intro_server.start()
        out = self.xm_GPSR.execute()
        intro_server.stop()


    def shutdown(self):
        if self.smach_bool ==True:
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
        Gpsr()
    except:
        rospy.logerr(e)
        #print("7777777")
