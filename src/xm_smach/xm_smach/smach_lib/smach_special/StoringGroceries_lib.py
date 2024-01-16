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
from std_msgs.msg import String, Int32, Bool, Header
from std_srvs.srv import *
from xm_msgs.srv import *
from xm_msgs.msg import *
import subprocess
import math
from speech.srv import *

'''
Author: yishui
Date: 2022-08-25
LastEditTime: 2022-08-25 15:17:44
LastEditors: yishui
Description: Codes for StoringGroceries_lib
FilePath: ~/catkin_ws/src/xm_smach/xm_smach/smach_lib/smach_special
'''

def get_pid(name):
    pid = ''
    with open("/home/xm/vision_pid/{}.txt".format(name)) as f:
        pid = f.read()
    return pid


class CheckTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'food', 'drinks', 'others', 'error'],
                       io_keys=['current_turn'],
                       input_keys=['turn', 'sort'])

    def execute(self, userdata):
        try:
            self.current_turn = userdata.current_turn
            self.turn = userdata.turn
            self.sort = userdata.sort
            rospy.logwarn(self.current_turn)
            rospy.logwarn(self.turn)
            rospy.logwarn(self.sort)
        except Exception, e:
            rospy.logerr(e)
            return 'error'
        userdata.current_turn += 1
        if self.current_turn >= self.turn:
            rospy.logwarn('xm finish the turn')
            return 'succeeded'
        else:
            rospy.logwarn('finish one turn')
            
            if self.sort == 1:
                return 'food'
            if self.sort == 2:
                return 'drinks'
            if self.sort == 3:
                return 'others'


class CheckSort(State):
    def __init__(self):
        State.__init__(self, outcomes=['drinks', 'others', 'error'],
                       io_keys=['sort'])

    def execute(self, userdata):
        userdata.sort += 1
        self.sort = userdata.sort

        if self.sort == 2:
            return 'drinks'
        if self.sort == 3:
            return 'others'
        else:
            rospy.logwarn('error')
            return 'error'


class GetTargetPosition(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target'],
                       output_keys=['target_camera_point', 'object_state', 'table_depth'])
        self.xm_findobject = rospy.ServiceProxy(
            'get_position', xm_ObjectDetect)

    def execute(self, userdata):
        try:
            #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
            #     rospy.sleep(2.0)
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/Storing/mrsupw_vison_server.py', '-d', 'true'], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w+') as f:
                if f.read() == '':
                    print('-------------------')
                    print('pid', a.pid)
                    print('pid', a.pid)
                    print('pid', a.pid)
                    print('pid', a.pid)
                    print('pid', a.pid)
                    print('pid', a.pid)
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
        except Exception, e:
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
                rospy.logwarn("\nfind object!")
            if res.object.pos.point.x == 0 or res.object.pos.point.y == 0 or res.object.pos.point.z == 0:
                rospy.logwarn("\ncan't find object!\n")
                self.killPro()
                return "error"

        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
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
            subprocess.Popen(['kill', '-9', pid], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w') as f:
                f.write('')

        except Exception, e:
            rospy.logerr('No such process ')


class GetPutPosition(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target'],
                       output_keys=['target_camera_point', 'object_state', 'table_depth'])
        self.xm_findobject = rospy.ServiceProxy(
            'get_position', xm_ObjectDetect)

    def execute(self, userdata):
        try:
            #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/detect_object.py",shell=True)
            #     rospy.sleep(2.0)
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/Storing/mrsupw_vison_server.py', '-d', 'true'], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w+') as f:
                if f.read() == '':
                    print('-------------------')
                    print('pid', a.pid)
                    print('pid', a.pid)
                    print('pid', a.pid)
                    print('pid', a.pid)
                    print('pid', a.pid)
                    print('pid', a.pid)
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
        except Exception, e:
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
            subprocess.Popen(['kill', '-9', pid], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w') as f:
                f.write('')

        except Exception, e:
            rospy.logerr('No such process ')


class GetPutPos(State):
    '''
    根据图像传过来的数据，确定抓取时xm的位置
    '''

    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'error'],
                       input_keys=['distance',
                                   'target_camera_point', 'put_pos'],
                       output_keys=['put_pos'])
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        try:
            getattr(userdata, 'distance')
            getattr(userdata, 'target_camera_point')
        except:
            rospy.logerr('no param')
            return 'error'
        else:
            target_camera_point = userdata.target_camera_point

            self.tf_listener.waitForTransform(
                'base_link', 'kinect2_rgb_link', rospy.Time(), rospy.Duration(10.0))
            target_base_point = self.tf_listener.transformPoint(
                'base_link', target_camera_point)

            print('target_base_point is ', target_base_point)
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
            qs.quaternion = Quaternion(*quaternion_from_euler(0, 0, -alpha))

            self.tf_listener.waitForTransform(
                'map', 'base_link', rospy.Time(), rospy.Duration(60.0))
            pick_base_point = self.tf_listener.transformPoint(
                'map', pick_base_point)

            qs = self.tf_listener.transformQuaternion('map', qs)
            userdata.pick_pos = Pose(pick_base_point.point, qs.quaternion)
            print("userdata.pick_pos :", userdata.pick_pos)

            return 'succeeded'


class PutDown(State):  # 需要把抓取的动作改成放置！即爪子闭合改成爪子张开
    '''
    ArmStack
    '''

    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['target_camera_point', 'target_size', 'table_depth'])
        self.arm_stack_client = actionlib.SimpleActionClient(
            "/xm_arm/arm_storing", xm_ArmStackAction)

    def execute(self, userdata):
        try:

            goal = xm_ArmStackGoal()
            goal.target_camera_point = userdata.target_camera_point
            goal.target_size_l = userdata.target_size[0]
            goal.target_size_w = userdata.target_size[1]
            goal.target_size_h = userdata.target_size[2]
            goal.table_depth = userdata.table_depth

            self.arm_stack_client.wait_for_server(rospy.Duration(60.0))

            self.arm_stack_client.cancel_all_goals()
            rospy.logwarn("send the goal")
            self.arm_stack_client.send_goal(goal)

            # self.arm_stack_client.wait_for_result(rospy.Duration.from_sec(60.0))
            self.arm_stack_client.wait_for_result(
                rospy.Duration.from_sec(120.0))
            return 'succeeded'
            # if self.arm_stack_client.get_state() == True:
            #     # if self.arm_stack_client.result_bool == True:
            #     return 'succeeded'
            # else:
            #     rospy.logerr("arm stack ik failed!")
            #     return 'aborted'
        except Exception:
            rospy.logerr('Arm Stack have not work!')
            # rospy.logerr(e)
            return 'error'
        else:
            return 'succeeded'
