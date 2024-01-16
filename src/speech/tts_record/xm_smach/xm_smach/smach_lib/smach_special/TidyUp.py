#!/usr/bin/env python
# encoding:utf8
from xm_smach.target_gpsr import gpsr_target
import rospy
import tf
import actionlib
from math import *
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatus 
from time import sleep
from std_msgs.msg import String,Int32,Bool,Header
from std_srvs.srv import *
from xm_msgs.srv import *
from xm_msgs.msg import *
import subprocess
import math  
from speech.srv import *

class CheckTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','continue','error'],
                        io_keys=['current_turn','task_num','current_task'],
                        input_keys=['turn'])
    def execute(self,userdata):
        try:
            self.current_turn = userdata.current_turn
            self.turn = userdata.turn
            rospy.logwarn(self.current_turn)
            rospy.logwarn(self.turn)
        except Exception,e:
            rospy.logerr(e)
            return 'error'
        if self.current_turn >= self.turn:
            rospy.logwarn('xm finish the turn')
            return 'succeeded'
        else:
            rospy.logwarn('finish one turn')
            userdata.current_turn += 1
            userdata.task_num = 0
            userdata.current_task = -1
            return 'continue'
            
class GetTask(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=['succeeded','aborted','error'],
                        io_keys=['target','action','task_num','answer'])
                        
        # self.speech_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        rospy.wait_for_service('speech_core')
        self.speech_client = rospy.ServiceProxy('speech_core', speech_to_smach)
                
    def execute(self,userdata):
        try:
            getattr(userdata, 'target')
            getattr(userdata, 'action')            
            getattr(userdata, 'task_num')
            getattr(userdata, 'answer')
        except:
            rospy.logerr('No param specified')
            return 'error'
        try:
            self.speech_client.wait_for_service(timeout=10)
        #command = 1用于gpsr
            response =self.speech_client.call(command = 1,text = 'www')
            print(response.num)
            print(response.action)
            print(response.object)
        except:
            rospy.logerr('wrong in call the service')
            return 'error'
        # if response.action[0] == 'stop':
        #     rospy.logerr('response wrong!')
        #     return 'aborted'
        if response.num > 0:
            userdata.task_num = response.num
            userdata.action = response.action
            userdata.target = response.object
            userdata.answer = response.answer
            print(userdata.task_num)
            print(userdata.action)
            print(userdata.target)
            print(userdata.answer)
            # if userdata.action[1] == 'grasp':
                # something interest here
                # userdata.action[0] = 'nav_grasp'
                # userdata.action.append('place')
                # userdata.task_num += 1
            return 'succeeded'
        else:
            return 'aborted'

class GetTarget(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                            input_keys =['target','current_task','mode'],
                            output_keys =['current_target'])
    def execute(self,userdata):
        try:#判断这些属性是否存在
            getattr(userdata,'target')
            getattr(userdata,'current_task')
            getattr(userdata,'mode')
        except:
            rospy.logerr('No params specified ')
            return 'error'
        if userdata.mode ==0:
            # due to the cv may need a different name-style from the speech_node
            # this should be modify
            # 由于cv可能需要一个来自speech_node的不同的name-style
            # 这个需要被修改
            userdata.current_target = userdata.target[userdata.current_task]#存目标名字
        elif userdata.mode ==1:
            rospy.logwarn(userdata.target)

            #if userdata.current_task == 'speaker':
            #     userdata.current_target = gpsr_target[userdata.target['speaker2']]['pos']
            # else:
            target0 =userdata.target[userdata.current_task]
            if target0 =='show' or target0=='shop':
                target0 = 'shelf'
            if target0 == 'living_room':
                target0 = 'livingroom'
            if target0 == 'dining_room':
                target0 = 'diningroom'
            print(target0)
            userdata.current_target = gpsr_target[target0]['pos']#存目标位置
            rospy.logwarn(userdata.current_task)
        else:
            return 'aborted'
        return 'succeeded'

class GetTargetPosition(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target'],
                       output_keys=['target_camera_point' ,'object_state','table_depth'])
        self.xm_findobject = rospy.ServiceProxy('get_position', xm_ObjectDetect)


    def execute(self, userdata):
        try:
        #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        #     rospy.sleep(2.0)
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_vison_server.py','-d','true'] ,shell =False)
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
            name = userdata.target   
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
        userdata.target = name
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

class GetPutPosition(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target'],
                       output_keys=['target_camera_point' ,'object_state','table_depth'])
        self.xm_findobject = rospy.ServiceProxy('get_position', xm_ObjectDetect)


    def execute(self, userdata):
        try:
        #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        #     rospy.sleep(2.0)
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_vison_server.py','-d','true'] ,shell =False)
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
            name = userdata.target   
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
        userdata.target = name
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

class GetPutPos(State):
    '''
    根据图像传过来的数据，确定抓取时xm的位置
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','error'],
                        input_keys= ['distance','target_camera_point','put_pos'],
                        output_keys =['put_pos'])
        self.tf_listener = tf.TransformListener()
    def execute(self,userdata):
        try:
            getattr(userdata, 'distance')
            getattr(userdata, 'target_camera_point')
        except:
            rospy.logerr('no param')
            return 'error'
        else :
            target_camera_point = userdata.target_camera_point
            
            self.tf_listener.waitForTransform('base_link','kinect2_rgb_link',rospy.Time(),rospy.Duration(10.0))
            target_base_point = self.tf_listener.transformPoint('base_link',target_camera_point)

            print('target_base_point is ',target_base_point)
            #                           y
            #               ————————————————————————————
            #               |                       |
            #               |                   |
            #               |               |
            #          x    |  alpha    |
            #               | ~~~   |
            #               |   |
            #               |
            x = target_base_point.point.x - 0.85
            #x = target_base_point.point.x
            y = 0
            alpha = 0

            pick_base_point = PointStamped()
            pick_base_point.header.frame_id = 'base_link'
            pick_base_point.point.x = x - userdata.distance
            #pick_base_point.point.x = x
            pick_base_point.point.y = -y
            
            qs = QuaternionStamped()
            qs.header.frame_id = 'base_link'
            qs.quaternion = Quaternion(*quaternion_from_euler(0,0,-alpha))
            
            self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
            pick_base_point =self.tf_listener.transformPoint('map',pick_base_point)
            
            qs =self.tf_listener.transformQuaternion('map',qs)
            userdata.pick_pos = Pose(pick_base_point.point,qs.quaternion)
            print("userdata.pick_pos :",userdata.pick_pos)

            return 'succeeded'

class PutDown(State):#需要把抓取的动作改成放置！即爪子闭合改成爪子张开
    '''
    ArmStack
    '''
    def __init__(self):
        State.__init__(self,
                        outcomes=['succeeded','aborted' ,'error'],
                       input_keys=['target_camera_point','target_size','table_depth'])
        self.arm_stack_client = actionlib.SimpleActionClient("/xm_arm/arm_stack", xm_ArmStackAction)
    def execute(self,userdata):
        try:
            
            goal = xm_ArmStackGoal()
            goal.target_camera_point  = userdata.target_camera_point
            goal.target_size_l = userdata.target_size[0]
            goal.target_size_w = userdata.target_size[1]
            goal.target_size_h = userdata.target_size[2]
            goal.table_depth = userdata.table_depth
    
            
            self.arm_stack_client.wait_for_server(rospy.Duration(60.0))
            
            self.arm_stack_client.cancel_all_goals()
            rospy.logwarn("send the goal")
            self.arm_stack_client.send_goal(goal)

            # self.arm_stack_client.wait_for_result(rospy.Duration.from_sec(60.0))
            self.arm_stack_client.wait_for_result(rospy.Duration.from_sec(120.0))

            if self.arm_stack_client.get_state() == True:
            #if self.arm_stack_client.result_bool == True:
                return 'succeeded'
            else:
                rospy.logerr("arm stack ik failed!")
                return 'aborted'
        except Exception ,e:
            rospy.logerr('Arm Stack have not work!')
            rospy.logerr(e)
            return 'error'
        else:
            return 'succeeded' 
