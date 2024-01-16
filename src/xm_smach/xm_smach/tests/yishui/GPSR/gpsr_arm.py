#!/usr/bin/env python
# encoding:utf8
from os import WIFSTOPPED
import os

from pytz import utc
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_special.gpsr import * 
from smach_compose.compose import * 
# from smach_common.common import ArmStack
# from smach_common.common import NavStack
# from smach_common.common import GetPickPosBack
# from smach_common.common import TargerPosReload
# from smach_common.common import GoAhead
# from smach_special.shopping_lib import ObjectDetect as GetObjectPosition
from smach_common.common import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

class GetObjectPosition1(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target'],
                       output_keys=['target_camera_point'])
        self.xm_findobject = rospy.ServiceProxy('get_position', xm_ObjectDetect)


    def execute(self, userdata):
        try:
        #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        #     rospy.sleep(2.0)
        # 
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/Realsense.py','-d','true'] ,shell =False)
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
            # name =str(userdata.target)
            name = userdata.target
            print ('name',name)
            # name.replace(' ' , '' , name.count(' '))    
        except Exception ,e:
            rospy.logerr(e)
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)

            req = xm_ObjectDetectRequest()
            req.object_name = name
            req.people_id = 0
            rospy.logwarn(name)
            rospy.logwarn(req)
            
            res = self.xm_findobject.call(req)
               
            if res.object.pos.point.x != 0 or res.object.pos.point.y != 0 or res.object.pos.point.z != 0:
                rospy.loginfo("find object!")
            else:
                rospy.loginfo("can't find object!")
                self.killPro()
                return 'aborted'
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        if res.object.pos.point.x == -10.000 or res.object.pos.point.x == 10.000 or res.object.pos.point.x == 5:
            rospy.logerr('find nothing')
            self.killPro()
            return 'aborted'


        rospy.logwarn(res.object)
        userdata.target_camera_point = res.object.pos
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
        rospy.logerr("Welcome to GPSR!")
        # empty pid file
        dir_path = r'/home/xm/vision_pid/'
        for file in os.listdir(dir_path):
            with open(os.path.join(dir_path,file),'w') as f:
                f.write('')
        print('Empty PID Files Success!')
        
        #-------PICK_UP-------#
        #RUNNODE_IMG-->GETNAME-->GET_POS-->NAV-->FIND-->POS_JUS-->NAV-->RUNNODE_IMG-->FIND-->POS_JUS-->PICK-->SPEAK
        #运行图像节点，获取物体名字和大物体位置后，nav到大物体位置，然后用图像找物体，然后用pos_jus得出适宜抓取的坐标，nav后继续循环一次
        #在此之后，进行抓取
        #如果上面的图像没有找到物体，则通过speak进行错误反馈
        # 传入一个target物品字符串，
        self.xm_Pick_up = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['target'])    
        with self.xm_Pick_up: 
            # self.xm_Pick_up.userdata.name ='orange water'
            self.xm_Pick_up.userdata.distance = 0.9 #change this element from 0.7 to 0.2
            # 抓到物品后倒退的距离
            self.xm_Pick_up.userdata.back_distance = -0.2
            # 桌子高度？
            self.xm_Pick_up.userdata.table_depth = 0.04
            # self.xm_Pick_up.userdata.pick_pose = [ [0,-1.5,3.14,0,0,0] ]
            self.xm_Pick_up.userdata.pick_pos=Pose()
            self.xm_Pick_up.userdata.up_down=1
            # 1是关闭，0是打开
            self.xm_Pick_up.userdata.commond=1
            # self.xm_Pick_up.userdata.target_camera_point = Pose()
            #StateMachine.add('PICK_POS',
            #                     ArmTrajectory(),#轨迹分析
            #                     transitions={'succeeded': 'FIND_1', 'error': 'error'},
            #                     remapping={'arm_waypoints':'pick_pose'})                                   
            
            # StateMachine.add('FIND_1',
            #                     GetObjectPosition1(),#区分物体，识别物体
            #                     transitions={'succeeded': 'PUT_DOWN', 'error': 'error'})

            # StateMachine.add('GETPICKPOSE',
            #                     GetPickPosBack(),  
            #                     transitions ={'succeeded':'NAVTOPICKPOS','error':'error'})       
            
            # StateMachine.add('NAVTOPICKPOS',
            #                     NavStack(),
            #                     transitions ={'succeeded':'TARGET_POINT_JUSFY','aborted':'NAVTOPICKPOS','error':'error'},
            #                     remapping ={"pos_xm":'pick_pos'})

            # # StateMachine.add('FIND_2',
            # #                     GetObjectPosition(),
            # #                     transitions={'succeeded': 'TARGET_POINT_JUSFY', 'error': 'error'},
            # #                     remapping={'target':'name'})   
                                                               
            # StateMachine.add('TARGET_POINT_JUSFY',
            #                     TargerPosReload(),
            #                     transitions={'succeeded': 'PICK', 'error': 'error'})

            # StateMachine.add('PICK', 
            #                     ArmStack(),
            #                     transitions={'succeeded': 'NAV_BACK','error': 'error'})

            # StateMachine.add('NAV_BACK', 
            #                     GoAhead(),
            #                     transitions={'succeeded': 'succeeded','error': 'error'},
            #                     remapping={'move_len':'back_distance'})
            StateMachine.add('PUT_DOWN',
                             GripperCommond(),  # 控制爪子（打开）
                             transitions={'succeeded': 'succeeded', 'error': 'error'})

         #顶层状态机
        self.xm_GPSR = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.xm_GPSR:

            self.xm_GPSR.userdata.target = "cookies"
            self.xm_GPSR.userdata.action = list()
            self.xm_GPSR.userdata.task_num = 0
            self.xm_GPSR.userdata.answer = "this is the answer"
            # current_task目前运行到的任务序号，从-1开始每次判断下一步做什么时，将其值+1
            # 任务实质上是从0开始，0为任务一，2为任务三，3为结束
            self.xm_GPSR.userdata.current_task = -1 
            self.xm_GPSR.userdata.current_turn = -1 
            self.xm_GPSR.userdata.turn = 1              #the max_num of the mission
            # self.xm_GPSR.userdata.pos_xm_door = gpsr_target['livingroom']['pos'] # 门的位置
            # self.xm_GPSR.userdata.waypoint = gpsr_target['dining_room_exit']['pos']
            self.xm_GPSR.userdata.sentences = 'give me the mission please' #for the Speak() state
            self.xm_GPSR.userdata.gripper_release = 0.0
            self.xm_GPSR.userdata.give_file = "give_me_the_mission.wav"
            self.xm_GPSR.userdata.sentences_release = "i will release the gripper,please grap it"
            self.xm_GPSR.userdata.rec = 5.0
            self.xm_GPSR.userdata.character = ''
            self.xm_GPSR.userdata.gesture = ''
            self.xm_GPSR.userdata.count_num = 0
            self.xm_GPSR.userdata.objectName = ''
            # StateMachine.add('ENTERROOM',
            #                      self.xm_EnterRoom,
            #                      transitions ={'succeeded':'RECEIVE_TASKS','aborted':'RECEIVE_TASKS','error':'error'})
            
            
            # StateMachine.add('SPEAK_RESTART',
            #                 SpeakWAV(),
            #                 transitions = {'succeeded':'RECEIVE_TASKS','aborted':'aborted','error':'error'},
            #                 remapping = {'filename':'give_file'}
            #                 )
            StateMachine.add('PICK_UP',
                                self.xm_Pick_up,
                                remapping ={'target':'target'},
                                transitions={'succeeded':'succeeded','aborted':'PICK_UP'})


        intro_server = IntrospectionServer('xm_gpsr',self.xm_GPSR,'/XM_ROOT')
        intro_server.start()
        out = self.xm_GPSR.execute()
        intro_server.stop()



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

