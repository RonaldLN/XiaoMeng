#!/usr/bin/env python
# encoding:utf8
import re
from turtle import pos, position
from xm_smach.target_gpsr import gpsr_target
import rospy
import time
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import *
import tf
from control_msgs.msg import *
import subprocess
import math
from speech.srv import *
from copy import deepcopy
import rospy


def get_pid(name):
    pid = ''
    with open('/home/xm/vision_pid/{}.txt'.format(name)) as f:
        pid = f.read()
        print("pid:         ", pid)
    return pid


class GetPersonPosition(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['people_pos'])
        self.xm_findperson = rospy.ServiceProxy(
            'get_position', xm_detectPerson)

    def execute(self, userdata):
        try:
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/Serveguest/detect_person.py', '-d', 'true'], shell=False)
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
            rospy.sleep(1.0)
            print a.poll()
            if a.returncode != None:
                a.wait()
                self.killPro()
                return 'aborted'
        except:
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        goal = Point()
        try:
            self.xm_findperson.wait_for_service(timeout=30.0)

            req = xm_detectPersonRequest()
            req.num = 0
            rospy.logwarn(req)

            res = self.xm_findperson.call(req)
            if res.coordinate.point.x == 0 or res.coordinate.point.y == 0 or res.coordinate.point.z == 0:
                rospy.loginfo("can't find people!")
                self.killPro()
                return 'aborted'
            if res.coordinate.point.x != 0 or res.coordinate.point.y != 0 or res.coordinate.point.z != 0:
                rospy.loginfo("find people!")
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        # if res.object.pos.point.x == -10.000 or res.object.pos.point.x == 10.000 or res.object.pos.point.x == 5:
        #     rospy.logerr('find nothing')
        #     self.killPro()
        #     return 'aborted'

        rospy.logwarn(res)
        # 传给底盘的人坐标直接使用摄像头的坐标
        # 这里应该有问题，应该有tf变换，但不会使用，测试有问题的话，留给叶总
        pos = Pose()
        pos.position.x = res.coordinate.point.x
        pos.position.y = res.coordinate.point.y
        pos.position.z = res.coordinate.point.z
        pos.orientation.w = 1

        userdata.people_pos.append(pos)
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


class SpeakAnswer1(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['sentences', 'object'])

        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            self.string = str(userdata.sentences)+str(userdata.object[-1])
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, self.string)
                rospy.sleep(1.0)
                return 'succeeded'

            except:
                return 'aborted'

# 用于人脸识别给分支
class SpeakAnswer2(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded1','succeeded2','succeeded3','succeeded4', 'aborted', 'error'],
                       io_keys=['present_guest_turn','sentences', 'object'])

        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            self.string = str(userdata.sentences)+str(userdata.object[-1])
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, self.string)
                rospy.sleep(1.0)
                rospy.logwarn(userdata.present_guest_turn)
                if userdata.present_guest_turn == 0:
                    return 'succeeded1'
                elif userdata.present_guest_turn == 1:
                    return 'succeeded2'
                elif userdata.present_guest_turn == 2:
                    return 'succeeded3'
                elif userdata.present_guest_turn == 3:
                    return 'succeeded4'
            except:
                return 'aborted'


class SortRoomPos(State):  # 跳转action
    def __init__(self):
        State.__init__(self,
                       # nav_grasp is something interesting
                       outcomes=['succeeded', 'aborted', 'error'])

    def execute(self, userdata):
        try:
            userdata.room_pos = userdata.search_pos1+userdata.search_pos2 + \
                userdata.search_pos3+userdata.search_pos4
        except:
            rospy.logerr('No param specified')
            return 'error'

        return 'succeeded'


class GetTargetPosition(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target'],
                       input_keys=['current_turn_num'],
                       output_keys=['target_camera_point', 'object_state', 'table_depth'])
        self.xm_findobject = rospy.ServiceProxy(
            'get_position_object', xm_ObjectDetect)

    def execute(self, userdata):
        try:
            #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
            #     rospy.sleep(2.0)
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_vison_server1.py', '-d', 'true'], shell=False)
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
                self.killPro()
                return 'aborted'
        except:
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        goal = Point()
        try:
            name = str(userdata.target[userdata.current_turn_num])
            name.replace(' ', '', name.count(' '))
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
        # userdata.target = name
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


class GetTargetPosition1(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'finished', 'error'],
                       io_keys=['target', 'current_turn_num'],
                       output_keys=['target_camera_point', 'object_state', 'table_depth'])
        self.xm_findobject = rospy.ServiceProxy(
            'get_position_object', xm_ObjectDetect)

    def execute(self, userdata):
        try:
            #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
            #     rospy.sleep(2.0)
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_vison_server1.py', '-d', 'true'], shell=False)
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
                self.killPro()
                return 'aborted'
        except:
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        goal = Point()
        try:
            name = str(userdata.target[userdata.current_turn_num])
            name.replace(' ', '', name.count(' '))
        except Exception, e:
            rospy.logerr(e)
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        try:
            rospy.logwarn(name)
            self.xm_findobject.wait_for_service(timeout=30.0)
            print(3696933)
            req = xm_ObjectDetectRequest()
            print(3696933999999999999)
            req.object_name = name
            req.people_id = 0
            rospy.logwarn(name)
            rospy.logwarn(req)

            res = self.xm_findobject.call(req)

            if res.object.pos.point.x != 0 or res.object.pos.point.y != 0 or res.object.pos.point.z != 0:
                rospy.logwarn("\nfind object!")
            if res.object.pos.point.x == 0 or res.object.pos.point.y == 0 or res.object.pos.point.z == 0:
                rospy.logwarn("\ncan't find object!\n")
                userdata.current_room_num += 1
                self.killPro()
                return "finished"

        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        rospy.logwarn(res.object)
        # userdata.target = name
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


class TransformCameraPoint(State):  # 根据摄像头传回的坐标计算人的位置
    def __init__(self):
        self.tracker = MonitorState('follow',
                                    xm_FollowPerson,
                                    self.people_cb,
                                    max_checks=5,
                                    output_keys=['pos_xm'])
        self.tf_listener = tf.TransformListener()
    # 如果相机找到人,这个状态将会返回False
    # 相反,如果在五个循环里相机都没找到人,将会返回True
    # msg传入主题的数据

    def people_cb(self, userdata, msg):
        if msg is not None:
            try:
                self.tmp_pos = msg.position
                rospy.logwarn(self.tmp_pos)
                rospy.logwarn('finding people........')
                if self.tmp_pos.point.x == -9.0 and self.tmp_pos.point.y == -9.0 and self.tmp_pos.point.z == -9.0:
                    rospy.logwarn('lost people and I will turn left!')
                    # 这里猜测如果人消失在机器人的左边，会传回一个值均为-9的Point数据
                    # data_deal_lose_left_sign,如果找不到左边的目标,创建一个Pose类型数据给x赋值为-9并返回
                    ps = self.data_deal_lose_left_sign(self.tmp_pos)
                    userdata.pos_xm = ps
                    # pass
                    return False
                if self.tmp_pos.point.x == -12.0 and self.tmp_pos.point.y == -12.0 and self.tmp_pos.point.z == -12.0:
                    rospy.logwarn('lost people and I will turn right!')
                    # 意思同上
                    ps = self.data_deal_lose_right_sign(self.tmp_pos)
                    userdata.pos_xm = ps
                    # pass
                    return False
                # 如果得到人的坐标信息返回移动的位置
                # elif self.get_distance(self.tmp_pos)>1 and self.get_distance(self.tmp_pos)<=4.0 :
                elif self.get_distance(self.tmp_pos) <= 4.0:
                    rospy.loginfo('i will move')
                    ps = self.data_deal_new(self.tmp_pos)
                    userdata.pos_xm = ps
                    return False
                    '''
                elif self.get_distance(self.tmp_pos) < 1:
                    rospy.loginfo('i will not move')
                    ps = self.data_deal_turn(self.tmp_pos)
                    userdata.pos_xm = ps
                    return False
                    '''

                else:
                    rospy.logerr('the person is out of the range')
                    return True
            except:
                rospy.logerr(e)
                return True
        else:
            raise Exception('MsgNotFind')

    def get_distance(self, pos_xm):
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y

        return hypot(person_x, person_y)

    def transToBase(self, point_xm):
        new_header = Header()
        new_header.frame_id = 'map'
        xm_point_stamped = PointStamped()
        xm_point_stamped.point = point_xm
        xm_point_stamped.header = new_header
        rospy.logwarn(xm_point_stamped)
        rospy.logwarn("!!!!")
        self.tf_listener.waitForTransform(
            'base_link', 'map', rospy.Time(), rospy.Duration(1))
        base_point = self.tf_listener.transformPoint(
            'base_link', xm_point_stamped)

        return base_point

    def data_deal_turn(self, pos_xm):
        # 图像到导航的坐标转换
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y
        # 计算人和xm连线与视线正前方夹角
        angle = atan2(person_y, person_x)
        # 初始化xm现在的位置用于之后得到base_link在全局坐标系中的位置
        person_x = person_x - 1.0*cos(angle)
        person_y = person_y - 1.0*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y = person_y
        pos_xm.point.z = 0
        new_header = Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        # 从角度得到四元数
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        rospy.logwarn(qs)
        # 等待tf的信息
        self.tf_listener.waitForTransform(
            'map', 'base_link', rospy.Time(), rospy.Duration(1))
        rospy.logwarn('get the tf message')
        # 利用tf信息转化坐标
        pos_xm = self.tf_listener.transformPoint('map', pos_xm)

        rospy.logwarn('tf point succeeded ')
        # qs是一个四元数
        qs = self.tf_listener.transformQuaternion('map', qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point, qs.quaternion)
        return ps
    # 返回xm经过处理后的Pose()

    def data_deal(self, pos_xm0):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
        person_x = pos_xm0.point.x
        person_y = pos_xm0.point.y
        angle = atan2(person_y, person_x)
        # 3.28 test
        # person_x = person_x - 0.7*cos(angle)
        # person_y = person_y -0.7*sin(angle)
        pos_xm0.point.x = person_x - 1.0
        pos_xm0.point.y = person_y
        pos_xm0.point.z = 0
        new_header = Header()
        new_header.frame_id = 'base_link'
        pos_xm0.header = new_header
        # change
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header = pos_xm0.header
        qs.quaternion = self.q

        # self.tf_listener.waitForTransform('base_footprint','camera_link',rospy.Time(),rospy.Duration(60.0))
        # self.tf_listener.waitForTransform('odom','base_footprint',rospy.Time(),rospy.Duration(60.0))
        self.tf_listener.waitForTransform(
            'map', 'base_link', rospy.Time(), rospy.Duration(60.0))

        rospy.logwarn('wait for tf succeeded ')

        # pos_xm0 =self.tf_listener.transformPoint('base_footprint',pos_xm0)
        # pos_xm0 =self.tf_listener.transformPoint('odom',pos_xm0)
        pos_xm0 = self.tf_listener.transformPoint('map', pos_xm0)
        rospy.logwarn('tf point succeeded ')

        # the angle should also transform to the map frame
        # qs =self.tf_listener.transformQuaternion('base_link',qs)

        # qs =self.tf_listener.transformQuaternion('base_footprint',qs)
        # qs =self.tf_listener.transformQuaternion('odom',qs)
        qs = self.tf_listener.transformQuaternion('map', qs)
        rospy.logwarn('tf quaternion succeeded ')
        pos_xm = Pose(pos_xm0.point, qs.quaternion)
        return pos_xm

    def data_deal_new(self, pos_xm0):

        person_x = pos_xm0.point.x
        person_y = pos_xm0.point.y
        angle = atan2(person_y, person_x)

        new_header = Header()
        new_header.frame_id = 'base_link'
        pos_xm0.header = new_header
        # change
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header = pos_xm0.header
        qs.quaternion = self.q

        pos_xm = Pose(pos_xm0.point, qs.quaternion)
        pos_xm.position.z = 10

        return pos_xm

    def data_deal_lose_left_sign(self, pos_xm):
        pos_xm = Pose()
        pos_xm.position.x = -9
        return pos_xm

    def data_deal_lose_right_sign(self, pos_xm):
        pos_xm = Pose()
        pos_xm.position.x = -12
        return pos_xm


class NavStackSpeaial(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],  # preemeted
                       input_keys=['people_pos'])
        self.nav_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        # try:
        # except:
        #     rospy.logerr('tf_transform is wrong')
        #     return 'error'
        try:
            getattr(userdata, 'people_pos')
        except:
            rospy.logerr('No param specified ')
            return 'error'
        else:
            pos_xm = userdata.people_pos[-1]
            rospy.logwarn(pos_xm)
            self.nav_client.wait_for_server(rospy.Duration(120))

            clear_client = rospy.ServiceProxy(
                '/move_base/clear_costmaps', Empty)
            # clear_client = rospy.ServiceProxy('/move_base_simple/goal', Empty)
            req = EmptyRequest()
            res = clear_client.call()
            rospy.loginfo('6666666666')
            self.nav_thing(pos_xm)
            return 'succeeded'

    def nav_thing(self, pos_xm):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"

        goal.target_pose.pose = pos_xm
        goal.target_pose.header.stamp = rospy.Time.now()

        self.nav_client.send_goal(goal)
        deta = 9999
        nav_counter = 0
        # while self.nav_client.get_state() != GoalStatus.SUCCEEDED and nav_counter < 150:
        while deta >= 0.6 and nav_counter < 150:
            nav_counter += 1
            (trans, rot) = self.tf_listener.lookupTransform(
                'map', 'base_link', rospy.Time())
            deta = sqrt(pow(trans[0] - pos_xm.position.x, 2) +
                        pow(trans[1] - pos_xm.position.y, 2))
            rospy.logerr("deta:" + str(deta))

            # goal.target_pose.pose.orientation.z = rot[2]
            # goal.target_pose.pose.orientation.w = rot[3]
            if nav_counter <= 10:
                self.nav_client.send_goal(goal)
                rospy.loginfo('65555555555555555556')
            if self.preempt_requested():
                rospy.logerr('preempted')
                return 'succeeded'
            else:

                pass
            rospy.sleep(0.5)
        (trans, rot) = self.tf_listener.lookupTransform(
            'map', 'base_link', rospy.Time())
        rospy.logerr("deta:" + str(deta))
        # self.nav_client.send_goal(goal)
        goal.target_pose.pose.position.x = trans[0]
        goal.target_pose.pose.position.y = trans[1]

        goal.target_pose.pose.orientation.z = (
            pos_xm.position.x - trans[0]) / deta
        goal.target_pose.pose.orientation.w = (
            pos_xm.position.y - trans[1]) / deta
        # rospy.logerr("5555")
        self.nav_client.send_goal(goal)
        rospy.loginfo(goal.target_pose.pose)
        # rospy.logerr("6666")

        if self.nav_client.get_goal_status_text() == 'Goal reached.':
            rospy.loginfo("nav task executes successfully ^_^")
            return 'succeeded'
        else:
            rospy.logerr("xm cannot find the way  T_T")
            return 'aborted'


class NavStackSpeaial2(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],  # preemeted
                       input_keys=['pos_xm'])
        self.nav_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        # try:
        # except:
        #     rospy.logerr('tf_transform is wrong')
        #     return 'error'
        try:
            getattr(userdata, 'pos_xm')
        except:
            rospy.logerr('No param specified ')
            return 'error'
        else:
            rospy.logwarn(pos_xm)
            self.nav_client.wait_for_server(rospy.Duration(120))

            clear_client = rospy.ServiceProxy(
                '/move_base/clear_costmaps', Empty)
            # clear_client = rospy.ServiceProxy('/move_base_simple/goal', Empty)
            req = EmptyRequest()
            res = clear_client.call()
            rospy.loginfo('6666666666')
            self.nav_thing(pos_xm)
            return 'succeeded'

    def nav_thing(self, pos_xm):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"

        goal.target_pose.pose = pos_xm
        goal.target_pose.header.stamp = rospy.Time.now()

        self.nav_client.send_goal(goal)
        deta = 9999
        nav_counter = 0
        # while self.nav_client.get_state() != GoalStatus.SUCCEEDED and nav_counter < 150:
        while deta >= 0.6 and nav_counter < 150:
            nav_counter += 1
            (trans, rot) = self.tf_listener.lookupTransform(
                'map', 'base_link', rospy.Time())
            deta = sqrt(pow(trans[0] - pos_xm.position.x, 2) +
                        pow(trans[1] - pos_xm.position.y, 2))
            rospy.logerr("deta:" + str(deta))

            # goal.target_pose.pose.orientation.z = rot[2]
            # goal.target_pose.pose.orientation.w = rot[3]
            if nav_counter <= 10:
                self.nav_client.send_goal(goal)
            if self.preempt_requested():
                rospy.logerr('preempted')
                return 'succeeded'
            else:

                pass
            rospy.sleep(0.5)
        (trans, rot) = self.tf_listener.lookupTransform(
            'map', 'base_link', rospy.Time())
        rospy.logerr("deta:" + str(deta))
        # self.nav_client.send_goal(goal)
        # goal.target_pose.pose.position.x = trans[0]
        # goal.target_pose.pose.position.y = trans[1]

        # goal.target_pose.pose.orientation.z = (pos_xm.position.x - trans[0]) / deta
        # goal.target_pose.pose.orientation.w = (pos_xm.position.y - trans[1]) / deta
        # rospy.logerr("5555")
        # self.nav_client.send_goal(goal)
        # rospy.loginfo(goal.target_pose.pose)
        # rospy.logerr("6666")

        if self.nav_client.get_goal_status_text() == 'Goal reached.':
            rospy.loginfo("nav task executes successfully ^_^")
            return 'succeeded'
        else:
            rospy.logerr("xm cannot find the way  T_T")
            return 'aborted'


class PositionTransform(State):  # 人的坐标转换
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'error'],
                       io_keys=['people_pos'])
        self.tf_camera_listener = tf.TransformListener()
        self.tf_camera_listener.waitForTransform(
            '/map', '/base_link', rospy.Time(), rospy.Duration(1.0))

    def execute(self, userdata):
        try:
            (trans, rot) = self.tf_camera_listener.lookupTransform(
                '/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo("tf Error")
            return 'error'
        rospy.loginfo(trans)
        rospy.loginfo(rot)
        rospy.loginfo(userdata.people_pos[-1])
        x = rot[0]
        y = rot[1]
        z = rot[2]
        w = rot[3]

        r = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        r = r / math.pi * 180

        rospy.logerr(r)
        rospy.logerr(math.sin(r * math.pi / 180))
        rospy.logerr(math.cos(r * math.pi / 180))

        tempPose = Pose()
        xBefore = trans[0] + userdata.people_pos[-1].position.x * math.cos(
            r * math.pi / 180) + userdata.people_pos[-1].position.y * math.sin(r * math.pi / 180)
        yBefore = trans[1] + userdata.people_pos[-1].position.x * math.sin(
            r * math.pi / 180) + userdata.people_pos[-1].position.y * math.cos(r * math.pi / 180)
        tempPose.position.x = trans[0] + (userdata.people_pos[-1].position.x - 0.75) * math.cos(
            r * math.pi / 180) + userdata.people_pos[-1].position.y * math.sin(r * math.pi / 180)
        tempPose.position.y = trans[1] + (userdata.people_pos[-1].position.x - 0.75) * math.sin(
            r * math.pi / 180) + userdata.people_pos[-1].position.y * math.cos(r * math.pi / 180)
        tempPose.position.z = 0
        r = math.atan((yBefore - tempPose.position.y) /
                      (xBefore - tempPose.position.x))
        sinr = math.sin(r)
        cosr = math.cos(r)
        w = cosr
        z = sinr

        tempPose.orientation.z = z
        tempPose.orientation.w = w
        userdata.people_pos[-1] = tempPose
        return 'succeeded'


class PositionJudge(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],  # preemeted
                       io_keys=['people_pos'])

    def execute(self, userdata):
        if len(userdata.people_pos) == 1:
            return 'succeeded'
        try:
            if self.positionOnList(userdata.people_pos[:-1], userdata.people_pos[-1]):
                del userdata.people_pos[-1]
                return 'error'
            return 'succeeded'
        except:
            rospy.loginfo('PositionJudge try again')
            return 'aborted'

    def positionOnList(people_pos, temp_pos):
        # 如果该坐标在客人列表中，返回True，否则返回False
        for index in people_pos:
            if temp_pos < index.position.x+0.5 and temp_pos > index.position.x-0.5 and temp_pos < index.position.y+0.5 and temp_pos > index.position.y-0.5:
                return True
        return False


class CheckGiveObject(State):  # 计算给人物品的任务是否已经完成了3次
    def __init__(self):
        State.__init__(self,
                       # nav_grasp is something interesting
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['current_turn_num'],
                       input_keys=['turn'])

    def execute(self, userdata):
        try:
            turn = userdata.turn
            current_turn_num = userdata.current_turn_num
            rospy.loginfo('turn: ', turn)
            rospy.loginfo('current_turn_num: ', current_turn_num)
        except:
            rospy.logerr('No param specified')
            return 'error'
        userdata.current_turn_num += current_turn_num

        if userdata.current_turn_num >= userdata.turn:
            return 'succeeded'

        return 'aborted'


class OpenDoorDetect(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'])
        self.xm_doorDetect = rospy.ServiceProxy(
            'door_detect', xm_doorDetect)

    def execute(self, userdata):
        try:
            a = subprocess.Popen(
                ['python3.9', '/home/xm/catkin_ws/src/xm_vision/src/scripts/new/open_the_door.py', '-d', 'true'], shell=False)
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
                self.killPro()
                return 'aborted'
        except:
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        try:
            self.xm_doorDetect.wait_for_service(timeout=30.0)
            req = xm_doorDetectRequest()
            req.door_detect = 0
            rospy.logwarn(req)

            res = self.xm_doorDetect.call(req)
            rospy.logwarn(res)
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        if res.situation == 1:
            self.killPro()
            return 'succeeded'
        else:
            self.killPro()
            return 'aborted'

    def killPro(self):
        try:
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill', '-9', pid], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w') as f:
                f.write('')

        except Exception, e:
            rospy.logerr('No such process ')

# 返回人的位置和一个映射

# 找到所有人并把摄像头Point坐标转换为全局的Pose坐标


# class FindAllPeople(State):
#     def __init__(self):
#         State.__init__(self,
#                        outcomes=['succeeded', 'aborted', 'error'],z == 0:
#                 rospy.logwarn("\ncan't find object!2")
#                 self.killPro()
#                        io_keys=['people_pos', 'name_pos_dict'])
#         self.people_find = rospy.ServiceProxy(
#             'get_ThreeGuestsposition', xm_ThreeGuestsDetect)

#     def execute(self, userdata):

#         try:
#             a = subprocess.Popen(
#                 ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/Serveguest/three_guest.py', '-d', 'true'], shell=False)
#             with open("/home/xm/vision_pid/people_tracking.txt", 'w+') as f:
#                 if f.read() == '':
#                     print('-------------------')
#                     print('pid', a.pid)
#                     print('pid', a.pid)
#                     print('pid', a.pid)z == 0:
#                 rospy.logwarn("\ncan't find object!2")
#                 self.killPro()
#                     print('pid', a.pid)
#                     print('-------------------')
#                     f.write(str(a.pid))
#             rospy.sleep(2.0)
#             print a.poll()
#             if a.returncode != None:
#                 a.wait()
#                 self.killPro()
#                 return 'aborted'
#         except:
#             rospy.logerr('No param specified')
#             self.killPro()
#             return 'error'

#         try:
#             self.people_find.wait_for_service(timeout=30.0)

#             req = xm_ThreeGuestsDetectRequest()z == 0:
#                 rospy.logwarn("\ncan't find object!2")
#                 self.killPro()
#             req.people_id = 0
#             rospy.logwarn(req)

#             res = self.people_find.call(req)
#             rospy.logwarn(res)
#             userdata.people_pos.append(res.pos1)
#             userdata.people_pos.append(res.pos2)
#             userdata.people_pos.append(res.pos3)
#             rospy.loginfo("get people!!!")
#             if res.pos1.point.x == 0 and res.pos1.point.y == 0 and res.pos1.point.z == 0:
#                 rospy.logwarn("\ncan't find object!1")
#                 self.killPro()
#                 return 'aborted'

#             if res.pos2.point.x == 0 and res.pos2.point.y == 0 and res.pos2.point.z == 0:
#                 rospy.logwarn("\ncan't find object!2")
#                 self.killPro()
#                 return 'aborted'

#             if res.pos3.point.x == 0 and res.pos3.point.y == 0 and res.pos3.point.z == 0:
#                 rospy.logwarn("\ncan't find object!3")
#                 self.killPro()
#                 return 'aborted'

#         except Exception, e:
#             rospy.logerr(e)
#             rospy.logwarn('bad call the service')
#             self.killPro()
#             return 'aborted'

#         current_pos=userdata.start_waypoint
#         userdata.people_pos = self.peoplePositionTransform(userdata.people_pos,current_pos)

    #     rospy.logwarn(userdata.people_pos)
    #     self.killPro()
    #     return 'succeeded'

    # def killPro(self):
    #     try:
    #         pid = get_pid("people_tracking")
    #         subprocess.Popen(['kill', '-9', pid], shell=False)
    #         with open("/home/xm/vision_pid/people_tracking.txt", 'w') as f:
    #             f.write('')

    #     except Exception, e:
    #         rospy.logerr('No such process ')

    # def peoplePositionTransform(self,people_pos,current_pos):

    #     rospy.loginfo(current_pos)
    #     x = current_pos.orientation.x
    #     y = current_pos.orientation.y
    #     z = current_pos.orientation.z
    #     w = current_pos.orientation.w

    #     r = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    #     r = r / math.pi * 180

        # rospy.logerr(r)
        # rospy.logerr(math.sin(r * math.pi / 180))
        # rospy.logerr(math.cos(r * math.pi / 180))

        # people_pos_list = []
        
        # for index in people_pos:
        #     tempPose = Pose()
        #     xBefore = current_pos.position.x + index.point.x * math.cos(
        #         r * math.pi / 180) + index.point.y * math.sin(r * math.pi / 180)
        #     yBefore = current_pos.position.y + index.point.x * math.sin(
        #         r * math.pi / 180) + index.point.y * math.cos(r * math.pi / 180)
        #     tempPose.position.x = current_pos.position.x + (index.point.x - 0.75) * math.cos(
        #         r * math.pi / 180) + index.point.y * math.sin(r * math.pi / 180)
        #     tempPose.position.y = current_pos.position.y + (index.point.x - 0.75) * math.sin(
        #         r * math.pi / 180) + index.point.y * math.cos(r * math.pi / 180)
        #     tempPose.position.z = 0
        #     r = math.atan((yBefore - tempPose.position.y) /
        #                   (xBefore - tempPose.position.x))
        #     sinr = math.sin(r)
        #     cosr = math.cos(r)
        #     w = cosr
        #     z = sinr
        #     tempPose.orientation.z = z
        #     tempPose.orientation.w = w
        #     people_pos_list.append(tempPose)
        # return people_pos_list


# 识别三个人的情况
class FindAllPeople2(State):
    '''
    no use
    根据图像传过来的数据,确定抓取时xm的位置
    up_down变量为1,则使用上面的摄像头;up_down变量为0,则使用下面的摄像头。
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted', 'error'],
                        io_keys=['people_pos'])
        self.tf_listener = tf.TransformListener()
        self.people_find = rospy.ServiceProxy(
            'get_ThreeGuestsposition', xm_ThreeGuestsDetect)

    def execute(self,userdata):

        try:
            a = subprocess.Popen(
                ['python3.9', '/home/xm/catkin_ws/src/xm_vision/src/scripts/Serveguest/three_guest.py', '-d', 'True'], shell=False)
                # ['python3.9', '/home/xm/catkin_ws/src/xm_vision/src/scripts/new/find_people.py', '-d', 'True'], shell=False)

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
                self.killPro()
                return 'aborted'
        except:
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        try:
            self.people_find.wait_for_service(timeout=30.0)

            req = xm_ThreeGuestsDetectRequest()
            req.people_id = 0
            rospy.logerr(12345678999)
            rospy.logerr(req)
             

            # if req.people_id == 0:
            #     self.killPro()
            #     rospy.logerr(2222222222222222233)
            #     return 'aborted'


            res = self.people_find.call(req)
            rospy.logerr(res)

            rospy.loginfo("get people!!!")
            if res.pos1.point.x == 0 and res.pos1.point.y == 0 and res.pos1.point.z == 0:
                rospy.logwarn("\ncan't find object!1")
                self.killPro()
                return 'aborted'

            # if res.pos2.point.x == 0 and res.pos2.point.y == 0 and res.pos2.point.z == 0:
            #     rospy.logwarn("\ncan't find object!2")
            #     self.killPro()
            #     return 'aborted'

            # if res.pos3.point.x == 0 and res.pos3.point.y == 0 and res.pos3.point.z == 0:
            #     rospy.logwarn("\ncan't find object!3")
            #     self.killPro()
            #     return 'aborted'

            userdata.people_pos.append(res.pos1)
            userdata.people_pos.append(res.pos2)
            userdata.people_pos.append(res.pos3)
            userdata.people_pos.append(res.pos4)

        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        # current_pos=userdata.start_waypoint
        # userdata.people_pos = self.peoplePositionTransform(userdata.people_pos,current_pos)
        
        rospy.logerr(userdata.people_pos)
        self.killPro()

        # 下面开始进行tf变换

        tempPose=[]
        for index in userdata.people_pos:
            # camera_link = "camera_link_up"
            camera_link = "camera_link"
            target_camera_point = index
            target_camera_point.header.frame_id = camera_link
            print("target_camera_point:",target_camera_point)
            self.tf_listener.waitForTransform('base_link',camera_link,rospy.Time(),rospy.Duration(10.0))
            target_base_point = self.tf_listener.transformPoint('base_link',target_camera_point)
            #                           y
            #               ————————————————————————————
            #               |                       |
            #               |                   |
            #               |               |
            #          x    |  alpha    |
            #               | ~~~   |
            #               |   |
            #               |
            print("target_base_point:",target_base_point)
            x = target_base_point.point.x
            y = target_base_point.point.y

            distance = math.sqrt(x**2+y**2)
            alpha =  math.atan(y/x)
            if index.point.x==0 and index.point.y==0 and index.point.z==0:
                move_in_line = distance
                #  1.2为距离人的距离
            elif distance >3.0:
                move_in_line = distance - 0.8
            else:
                move_in_line = distance - 0.8
            
            rospy.logwarn("111111111distance is:"+str(distance))
            rospy.logwarn("222222222move_in_line is:"+str(move_in_line))

            pick_base_point = PointStamped()
            pick_base_point.header.frame_id = 'base_link'
            pick_base_point.point.x = move_in_line*math.cos(alpha)
            pick_base_point.point.y = move_in_line*math.sin(alpha) 

            target_camera_point_new = PointStamped()
            target_camera_point_new.header.frame_id = 'base_link'
            target_camera_point_new.point.x = (distance-move_in_line)*math.cos(alpha)
            target_camera_point_new.point.y = (distance-move_in_line)*math.sin(alpha) 
            target_camera_point_new.point.z = target_base_point.point.z


            self.tf_listener.waitForTransform(camera_link,'base_link',rospy.Time(),rospy.Duration(10.0))
            index = self.tf_listener.transformPoint(camera_link,target_camera_point_new)
            print('target_camera_point new',index)
            
            qs = QuaternionStamped()
            qs.header.frame_id = 'base_link'
            qs.quaternion = Quaternion(*quaternion_from_euler(0,0,alpha))
            
            self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
            pick_base_point =self.tf_listener.transformPoint('map',pick_base_point)
            
            qs =self.tf_listener.transformQuaternion('map',qs)
            tempPose.append(Pose(pick_base_point.point,qs.quaternion))

        userdata.people_pos=tempPose
        rospy.logwarn(userdata.people_pos)
        return 'succeeded'

    def killPro(self):
        try:
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill', '-9', pid], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w') as f:
                f.write('')

        except Exception, e:
            rospy.logerr('No such process ')


class GetTaskH(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target', 'action', 'answer'])

        self.speech_client = rospy.ServiceProxy('speech_core', speech_to_smach)

    def execute(self, userdata):
        try:
            getattr(userdata, 'target')
            getattr(userdata, 'action')
        except:
            rospy.logerr('No param specified')
            return 'error'
        try:
            self.speech_client.wait_for_service(timeout=10)
        # command = 1用于gpsr
            response = self.speech_client.call(command=5)
            if len(response.object)== 0 :
                return 'aborted'
            print(response.action)
            print(response.object)
        except:
            rospy.logerr('wrong in call the service')
            return 'error'
        userdata.target.append(response.object[0])
        rospy.logwarn(userdata.target)

        return 'succeeded'


class GetRoom(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['living_room_pos', 'dining_room_pos',
                                   'kitchen_room_pos', 'bedroom_pos'],
                       io_keys=['room_pos','room_name'])

        # self.speech_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        # rospy.wait_for_service('speech_core')
        self.speech_client = rospy.ServiceProxy('speech_core', speech_to_smach)

    def execute(self, userdata):
        try:
            self.speech_client.wait_for_service(timeout=10)
        # command = 1用于gpsr
            response = self.speech_client.call(command=5)
            if len(response.object)==0:
                rospy.logwarn('please speak again')
                return 'aborted'
            print(response.object)
            room_name = response.object[0]
        except:
            rospy.logerr('wrong in call the service')
            return 'error'

        if room_name == 'kitchen':
            userdata.room_pos = userdata.kitchen_room_pos
            userdata.room_name = room_name
        elif room_name == 'living_room':
            userdata.room_pos = userdata.living_room_pos
            userdata.room_name = room_name
        elif room_name == 'dining_room':
            userdata.room_pos = userdata.dining_room_pos
            userdata.room_name = room_name
        elif room_name == 'bedroom':
            userdata.room_pos = userdata.bedroom_pos
        else:
            rospy.logerr('can not find the room ')
            return 'aborted'
        rospy.logwarn(userdata.room_pos)
        return 'succeeded'


class CheckCurrentPerson(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded','finish', 'error'],
                       input_keys=['target', 'turn', 'people_pos', 'people_name'],
                       io_keys=['present_guest_turn', 'present_guest_pos', 'name_pos_dict', 'pos_name_dict'])

    def execute(self, userdata):
        if len(userdata.target) >= userdata.turn:
            rospy.logwarn(userdata.people_name)
            rospy.logwarn(userdata.target)
            for i in range(userdata.turn):
                people_name = userdata.people_name[i]
                target=userdata.target[i]
                userdata.name_pos_dict[people_name] = userdata.target[i]
                userdata.pos_name_dict[target] = userdata.people_name[i]
            return 'finish'

        if len(userdata.target) > userdata.present_guest_turn:
            userdata.present_guest_turn += 1
        userdata.present_guest_pos = userdata.people_pos[userdata.present_guest_turn]
        return 'succeeded'


# 返回识别的人名,并将人名记录到people_name中
class FaceRecognition1(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['people_name'])
        self.xm_faceRecognition1 = rospy.ServiceProxy(
            'get_name', xm_FaceDetect)

    def execute(self, userdata):
        try:
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/face_recognition-master/face_recognition.py', '-d', 'true'], shell=False)
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
            rospy.sleep(1.0)
            print a.poll()
            if a.returncode != None:
                a.wait()
                self.killPro()
                return 'aborted'
        except:
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        try:
            self.xm_faceRecognition1.wait_for_service(timeout=30.0)

            req = xm_FaceDetectRequest()
            req.people_id = 0
            rospy.logwarn(req)

            res = self.xm_faceRecognition1.call(req)
            userdata.people_name.append(res.people_name)

        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'
        rospy.logwarn(res)
        self.killPro()
        return 'succeeded'

    def killPro(self):
        try:
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill', '-9', pid], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w') as f:
                f.write('')

        except Exception, e:
            rospy.logerr('No such process ')


# 返回识别的人名,并将人名记录到people_name中
class FaceRecognition(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['current_people_name'])
        self.xm_faceRecognition1 = rospy.ServiceProxy(
            'get_name', xm_FaceDetect)

    def execute(self, userdata):
        try:
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/face_recognition/face_recognition.py', '-d', 'true'], shell=False)
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
            rospy.sleep(1.0)
            print a.poll()
            if a.returncode != None:
                a.wait()
                self.killPro()
                return 'aborted'
        except:
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        try:
            self.xm_faceRecognition1.wait_for_service(timeout=30.0)

            req = xm_FaceDetectRequest()
            req.people_id = 0
            rospy.logwarn(req)

            res = self.xm_faceRecognition1.call(req)
            rospy.logwarn(res)
            if res.people_name == userdata.current_people_name:
                rospy.logwarn(res)
                self.killPro()
                return 'succeeded'
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'error'   
        self.killPro()
        return 'aborted'

    def killPro(self):
        try:
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill', '-9', pid], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w') as f:
                f.write('')

        except Exception, e:
            rospy.logerr('No such process ')


class FindAllObject(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['current_turn_num'],
                       io_keys=['target', 'current_object_name'],
                       output_keys=['camera_point1'])
        self.xm_findobject = rospy.ServiceProxy(
            'get_position', xm_ObjectDetect)

    def execute(self, userdata):
        try:
            # 调可以看远处的节点
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_vison_server_remote.py', '-d', 'true'], shell=False)
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
        # 给机械臂传的Point类型的值
        goal = Point()
        try:
            name = userdata.target[userdata.current_turn_num]
            userdata.current_object_name = name
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
        userdata.target = name
        userdata.camera_point1 = res.object.pos
        self.killPro()
        return 'succeeded'

    def killPro(self):
        try:

            pid = get_pid("people_tracking")
            subprocess.Popen(['kill', '-9', pid], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w') as f:
                f.write('')

        except Exception, e:
            rospy.logerr('No such process ')


class GetPickPos1(State):
    '''
    no use
    根据图像传过来的数据，确定抓取时xm的位置
    up_down变量为1，则使用上面的摄像头；up_down变量为0，则使用下面的摄像头。
    只适合上面摄像头使用
    '''

    def __init__(self):
        State.__init__(self, outcomes=['succeeded','aborted' ,'error'],
                       input_keys=['camera_point1'],
                       io_keys=['object_current_pos'])
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        try:
            target_camera_point = userdata.camera_point1
            target_camera_point.header.frame_id = "camera_link_up"
            print("target_camera_point:", target_camera_point)
            self.tf_listener.waitForTransform(
                'base_link', 'camera_link_up', rospy.Time(), rospy.Duration(10.0))
            target_base_point = self.tf_listener.transformPoint(
                'base_link', target_camera_point)
            #                           y
            #               ————————————————————————————
            #               |                       |
            #               |                   |
            #               |               |
            #          x    |  alpha    |
            #               | ~~~   |
            #               |   |
            #               |
            print("target_base_point:", target_base_point)
            x = target_base_point.point.x
            y = target_base_point.point.y

            distance = math.sqrt(x**2+y**2)
            alpha = math.atan(y/x)
            #  状态机的userdata.distance 需要确定
            # 此处的2为导航结束后机器人距离物品的距离
            robot_object_distance = 2
            if target_camera_point.x**2+target_camera_point.y**2 > 4:
                move_in_line = distance - robot_object_distance

            print("distance is:"+str(distance))
            print("move_in_line is:"+str(move_in_line))

            pick_base_point = PointStamped()
            pick_base_point.header.frame_id = 'base_link'
            pick_base_point.point.x = move_in_line*math.cos(alpha)
            pick_base_point.point.y = move_in_line*math.sin(alpha)

            target_camera_point_new = PointStamped()
            target_camera_point_new.header.frame_id = 'base_link'
            target_camera_point_new.point.x = (
                distance-move_in_line)*math.cos(alpha)
            target_camera_point_new.point.y = (
                distance-move_in_line)*math.sin(alpha)
            target_camera_point_new.point.z = target_base_point.point.z

            self.tf_listener.waitForTransform(
                'camera_link_up', 'base_link', rospy.Time(), rospy.Duration(10.0))
            userdata.camera_point1 = self.tf_listener.transformPoint(
                'camera_link_up', target_camera_point_new)
            print('target_camera_point new', userdata.camera_point1)

            qs = QuaternionStamped()
            qs.header.frame_id = 'base_link'
            qs.quaternion = Quaternion(*quaternion_from_euler(0, 0, alpha))

            self.tf_listener.waitForTransform(
                'map', 'base_link', rospy.Time(), rospy.Duration(60.0))
            pick_base_point = self.tf_listener.transformPoint(
                'map', pick_base_point)

            qs = self.tf_listener.transformQuaternion('map', qs)
            userdata.object_current_pos = Pose(
                pick_base_point.point, qs.quaternion)
        except:
            rospy.logwarn('tf translation is wrong')
            return 'aborted'
        return 'succeeded'

# 找到物品对应的人名，和第一次要去的位置
# class FindGuest(State):  # 计算客人位置
#     def __init__(self):
#         State.__init__(self,
#                        outcomes=['succeeded', 'aborted', 'error'],
#                        io_keys=['people_pos','current_people_name','current_people_pos'],
#                        input_keys=['current_guest_num','people_name','target','turn','name_pos_dict'])

#     def execute(self, userdata):
#         try:
#             people_pos = userdata.people_pos
#             current_guest_num = userdata.current_guest_num
#             rospy.loginfo('people_pos: ', people_pos)
#             rospy.loginfo('current_guest_num: ', current_guest_num)

#             userdata.current_people_name=userdata.people_name[userdata.current_guest_num]
#             userdata.current_people_pos = userdata.name_pos_dict[userdata.current_people_name]
#             if
#         except:
#             rospy.logerr('No param specified')
#             return 'error'

#         rospy.loginfo('current_people_name: ', userdata.current_people_name)
#         rospy.loginfo('current_people_pos: ', userdata.current_people_pos)
#         return 'succeeded'


class FindGuest(State):  # 计算客人位置
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'error'],
                       io_keys=['people_pos', 'current_people_name',
                                'current_people_pos'],
                       input_keys=['target', 'turn', 'pos_gone_bool'])

    def execute(self, userdata):
        try:
            people_pos = userdata.people_pos
            # current_guest_num = userdata.current_guest_num
            rospy.loginfo('people_pos: ', people_pos)
            # rospy.loginfo('current_guest_num: ', current_guest_num)

            # userdata.current_people_name = userdata.people_name[userdata.current_guest_num]

            NUM = 0
            for i in range(userdata.turn):
                if userdata.pos_gone_bool[i] == 0:
                    NUM = i
                    break
            userdata.current_people_pos = userdata.people_pos[NUM]
            rospy.loginfo('current_people_name: ', userdata.current_people_name)
            rospy.loginfo('current_people_pos: ', userdata.current_people_pos)
        except:
            rospy.logerr('No param specified')
            return 'succeeded'

        return 'succeeded'


# 重新选择位置，修改为下一个位置，如果是最后一个位置，直接递给他
class FindGuestAgain(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded','finish', 'error'],
                       input_keys=['target', 'turn',
                                   'people_pos', 'people_name'],
                       io_keys=['present_guest_turn', 'present_guest_pos', 'current_people_pos', 'pos_gone_bool'])

    def execute(self, userdata):
        try:
            num = 0
            for i in range(userdata.turn):
                if userdata.people_pos[i] == userdata.current_people_pos:
                    num = i
                    break
            # 如果是最后一个地点，直接递给他。
            if num == 2:
                return 'finish'
            for i in range(num+1, userdata.turn):
                if userdata.pos_gone_bool[i] == 0:
                    userdata.current_people_pos = userdata.people_pos[i]
                    return 'succeeded'

        except:
            rospy.logerr('no param')
            return 'error'


class CheckCurrentObjectNum(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'finish','error'],
                       input_keys=['target', 'turn',
                                   'people_pos', 'people_name'],
                       io_keys=['current_turn_num', 'present_guest_pos', 'current_people_pos', 'pos_gone_bool','find_object_count'])

    def execute(self, userdata):

        try:
            userdata.current_turn_num += 1
            userdata.find_object_count=1
            if userdata.current_turn_num >= userdata.turn:
                return 'finish'
            return 'succeeded'

        except:
            rospy.logerr('no param')
            return 'error'


# 此节点只针对一个客人的情况
class FindAllPeople3(State):
    '''
    no use
    根据图像传过来的数据,确定抓取时xm的位置
    up_down变量为1,则使用上面的摄像头;up_down变量为0,则使用下面的摄像头。
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted', 'error'],
                        io_keys=['people_pos'])
        self.tf_listener = tf.TransformListener()
        self.people_find = rospy.ServiceProxy(
            'get_ThreeGuestsposition', xm_ThreeGuestsDetect)

    def execute(self,userdata):

        try:
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/Serveguest/three_guest.py', '-d', 'true'], shell=False)
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
                self.killPro()
                return 'aborted'
        except:
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        try:
            self.people_find.wait_for_service(timeout=30.0)

            req = xm_ThreeGuestsDetectRequest()
            req.people_id = 0
            rospy.logwarn(req)

            res = self.people_find.call(req)
            rospy.logwarn(res)
            userdata.people_pos.append(res.pos1)
            userdata.people_pos.append(res.pos2)
            userdata.people_pos.append(res.pos3)
            rospy.loginfo("get people!!!")
            if res.pos1.point.x == 0 and res.pos1.point.y == 0 and res.pos1.point.z == 0:
                rospy.logwarn("\ncan't find object!1")
                self.killPro()
                return 'aborted'


        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        # current_pos=userdata.start_waypoint
        # userdata.people_pos = self.peoplePositionTransform(userdata.people_pos,current_pos)

        rospy.logwarn(userdata.people_pos)
        self.killPro()

        # 下面开始进行tf变换

        tempPose=[]
        for index in userdata.people_pos:
            camera_link = "camera_link"
            target_camera_point = index
            target_camera_point.header.frame_id = camera_link
            print("target_camera_point:",target_camera_point)
            self.tf_listener.waitForTransform('base_link',camera_link,rospy.Time(),rospy.Duration(10.0))
            target_base_point = self.tf_listener.transformPoint('base_link',target_camera_point)
            #                           y
            #               ————————————————————————————
            #               |                       |
            #               |                   |
            #               |               |
            #          x    |  alpha    |
            #               | ~~~   |
            #               |   |
            #               |
            print("target_base_point:",target_base_point)
            x = target_base_point.point.x
            y = target_base_point.point.y

            distance = math.sqrt(x**2+y**2)
            alpha =  math.atan(y/x)
            #  1.0为距离人的距离
            move_in_line = distance - 1.0
            
            print("distance is:"+str(distance))
            print("move_in_line is:"+str(move_in_line))

            pick_base_point = PointStamped()
            pick_base_point.header.frame_id = 'base_link'
            pick_base_point.point.x = move_in_line*math.cos(alpha)
            pick_base_point.point.y = move_in_line*math.sin(alpha) 

            target_camera_point_new = PointStamped()
            target_camera_point_new.header.frame_id = 'base_link'
            target_camera_point_new.point.x = (distance-move_in_line)*math.cos(alpha)
            target_camera_point_new.point.y = (distance-move_in_line)*math.sin(alpha) 
            target_camera_point_new.point.z = target_base_point.point.z


            self.tf_listener.waitForTransform(camera_link,'base_link',rospy.Time(),rospy.Duration(10.0))
            index = self.tf_listener.transformPoint(camera_link,target_camera_point_new)
            print('target_camera_point new',index)
            
            qs = QuaternionStamped()
            qs.header.frame_id = 'base_link'
            qs.quaternion = Quaternion(*quaternion_from_euler(0,0,alpha))
            
            self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
            pick_base_point =self.tf_listener.transformPoint('map',pick_base_point)
            
            qs =self.tf_listener.transformQuaternion('map',qs)
            tempPose.append(Pose(pick_base_point.point,qs.quaternion))

        userdata.people_pos=tempPose
        rospy.logwarn(userdata.people_pos)
        return 'succeeded'

    def killPro(self):
        try:
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill', '-9', pid], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w') as f:
                f.write('')

        except Exception, e:
            rospy.logerr('No such process ')

class FindObjectPos(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted','error'],
                       input_keys=['room_object_pos1','room_object_pos2'],
                       io_keys=['room_pos_num','room_object_pos','camera_num'])

    def execute(self, userdata):
        
        try:
            if userdata.room_pos_num ==1:
                userdata.room_object_pos = userdata.room_object_pos1
                rospy.logwarn(userdata.room_object_pos)
                return 'succeeded'
            elif userdata.room_pos_num ==2:
                userdata.room_object_pos = userdata.room_object_pos2
                rospy.logwarn(userdata.room_object_pos)
                return 'succeeded'
        except:
            rospy.logwarn('FindObjectPos error')
            return 'error'

class TryFindObject1(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted','giveup','next', 'error'],
                       input_keys=['turn','target'],
                       output_keys=['target_camera_point'],
                       io_keys=['current_object_name','room_pos_num'])
        self.xm_findobject = rospy.ServiceProxy('get_3position', xm_3ObjectDetect)

    def execute(self, userdata):
        try:
            print(userdata.target[0])
            # a = subprocess.Popen(['python3.9','/home/xm/catkin_ws/src/xm_vision/src/scripts/new/test.py','-d','True','-t','Chip'] ,shell =False)

            a = subprocess.Popen(['python3.9','/home/xm/catkin_ws/src/xm_vision/src/scripts/new/know_and_findplace.py','-d','True','-t','%s' %userdata.target[0]] ,shell =False)
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
                self.killPro()
                return 'aborted'
        except:
             rospy.logerr('No param specified')
             self.killPro()
             return 'error'

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)
            req = xm_3ObjectDetectRequest()
            req.object_name1 = userdata.target[0]
            req.object_name2 = userdata.target[1]
            req.object_name3 = userdata.target[2]
            # rospy.logwarn(req)
            
            res = self.xm_findobject.call(req)
            rospy.logwarn(res)
            if res.pos.point.x != 0 or res.pos.point.y != 0 or res.pos.point.z != 0:
                rospy.loginfo("find object!")
            else:
                rospy.loginfo("can't find object!")
                # if userdata.room_pos_num==1 and userdata.camera_num==0:
                #     userdata.camera_num+=1
                #     self.killPro()
                #     return 'aborted'
                # elif userdata.room_pos_num==1 and userdata.camera_num==1:
                #     userdata.camera_num=0
                #     userdata.room_pos_num=2
                    
                # elif userdata.room_pos_num==2 and userdata.camera_num==0:
                #     userdata.camera_num+=1
                #     self.killPro()
                #     return 'aborted'
                # elif userdata.room_pos_num==2 and userdata.camera_num==1:
                #     userdata.camera_num=0
                #     userdata.room_pos_num=1
                #     self.killPro()
                #     return 'giveup'
                self.killPro()
                return 'next'
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        # rospy.logwarn('1111111111111111111')
        userdata.current_object_name = res.object_name
        # userdata.target_camera_point = res.pos
        self.killPro()
        rospy.logwarn(userdata.current_object_name)
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

class TryFindObject2(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted','giveup','next', 'error'],
                       input_keys=['turn','target'],
                       output_keys=['target_camera_point'],
                       io_keys=['current_object_name','room_pos_num'])
        self.xm_findobject = rospy.ServiceProxy('get_3position', xm_3ObjectDetect)

    def execute(self, userdata):
        try:
            print(userdata.target[1])
            a = subprocess.Popen(['python3.9','/home/xm/catkin_ws/src/xm_vision/src/scripts/new/know_and_findplace.py','-d','true','-t','%s' %userdata.target[1]] ,shell =False)
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
                self.killPro()
                return 'aborted'
        except:
             rospy.logerr('No param specified')
             self.killPro()
             return 'error'

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)
            req = xm_3ObjectDetectRequest()
            req.object_name1 = userdata.target[0]
            req.object_name2 = userdata.target[1]
            req.object_name3 = userdata.target[2]
            rospy.logwarn(req)
            
            res = self.xm_findobject.call(req)
            rospy.logwarn(res)
            if res.pos.point.x != 0 or res.pos.point.y != 0 or res.pos.point.z != 0:
                rospy.loginfo("find object!")
            else:
                rospy.loginfo("can't find object!")
                # if userdata.room_pos_num==1 and userdata.camera_num==0:
                #     userdata.camera_num+=1
                #     self.killPro()
                #     return 'aborted'
                # elif userdata.room_pos_num==1 and userdata.camera_num==1:
                #     userdata.camera_num=0
                #     userdata.room_pos_num=2
                    
                # elif userdata.room_pos_num==2 and userdata.camera_num==0:
                #     userdata.camera_num+=1
                #     self.killPro()
                #     return 'aborted'
                # elif userdata.room_pos_num==2 and userdata.camera_num==1:
                #     userdata.camera_num=0
                #     userdata.room_pos_num=1
                #     self.killPro()
                #     return 'giveup'
                self.killPro()
                return 'next'
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        userdata.current_object_name = res.object_name
        userdata.target_camera_point = res.pos
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


class TryFindObject3(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted','giveup','next', 'error'],
                       input_keys=['turn','target'],
                       output_keys=['target_camera_point'],
                       io_keys=['current_object_name','room_pos_num'])
        self.xm_findobject = rospy.ServiceProxy('get_3position', xm_3ObjectDetect)

    def execute(self, userdata):
        try:
            print(userdata.target[2])
            a = subprocess.Popen(['python3.9','/home/xm/catkin_ws/src/xm_vision/src/scripts/new/know_and_findplace.py','-d','true','-t','%s' %userdata.target[2]] ,shell =False)
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
                self.killPro()
                return 'aborted'
        except:
             rospy.logerr('No param specified')
             self.killPro()
             return 'error'

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)
            req = xm_3ObjectDetectRequest()
            req.object_name1 = userdata.target[0]
            req.object_name2 = userdata.target[1]
            req.object_name3 = userdata.target[2]
            rospy.logwarn(req)
            
            res = self.xm_findobject.call(req)
            rospy.logwarn(res)
            if res.pos.point.x != 0 or res.pos.point.y != 0 or res.pos.point.z != 0:
                rospy.loginfo("find object!")
            else:
                rospy.loginfo("can't find object!")
                # if userdata.room_pos_num==1 and userdata.camera_num==0:
                #     userdata.camera_num+=1
                #     self.killPro()
                #     return 'aborted'
                # elif userdata.room_pos_num==1 and userdata.camera_num==1:
                #     userdata.camera_num=0
                #     userdata.room_pos_num=2
                    
                # elif userdata.room_pos_num==2 and userdata.camera_num==0:
                #     userdata.camera_num+=1
                #     self.killPro()
                #     return 'aborted'
                # elif userdata.room_pos_num==2 and userdata.camera_num==1:
                #     userdata.camera_num=0
                #     userdata.room_pos_num=1
                #     self.killPro()
                #     return 'giveup'
                self.killPro()
                return 'next'
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        userdata.current_object_name = res.object_name
        userdata.target_camera_point = res.pos
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

class TryFindObject4(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted','giveup','next', 'error'],
                       input_keys=['turn','target'],
                       output_keys=['target_camera_point'],
                       io_keys=['current_object_name','room_pos_num'])
        self.xm_findobject = rospy.ServiceProxy('get_3position', xm_3ObjectDetect)

    def execute(self, userdata):
        try:
            print(userdata.target[3])
            a = subprocess.Popen(['python3.9','/home/xm/catkin_ws/src/xm_vision/src/scripts/new/know_and_findplace.py','-d','true','-t','%s' %userdata.target[3]] ,shell =False)
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
                self.killPro()
                return 'aborted'
        except:
             rospy.logerr('No param specified')
             self.killPro()
             return 'error'

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)
            req = xm_3ObjectDetectRequest()
            req.object_name1 = userdata.target[0]
            req.object_name2 = userdata.target[1]
            req.object_name3 = userdata.target[2]
            req.object_name4 = userdata.target[3]
            rospy.logwarn(req)
            
            res = self.xm_findobject.call(req)
            rospy.logwarn(res)
            if res.pos.point.x != 0 or res.pos.point.y != 0 or res.pos.point.z != 0:
                rospy.loginfo("find object!")
            else:
                rospy.loginfo("can't find object!")
                # if userdata.room_pos_num==1 and userdata.camera_num==0:
                #     userdata.camera_num+=1
                #     self.killPro()
                #     return 'aborted'
                # elif userdata.room_pos_num==1 and userdata.camera_num==1:
                #     userdata.camera_num=0
                #     userdata.room_pos_num=2
                    
                # elif userdata.room_pos_num==2 and userdata.camera_num==0:
                #     userdata.camera_num+=1
                #     self.killPro()
                #     return 'aborted'
                # elif userdata.room_pos_num==2 and userdata.camera_num==1:
                #     userdata.camera_num=0
                #     userdata.room_pos_num=1
                #     self.killPro()
                #     return 'giveup'
                self.killPro()
                return 'next'
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        userdata.current_object_name = res.object_name
        userdata.target_camera_point = res.pos
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

class FindSpeak(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['find_sentences', 'current_object_name'])

        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            self.string = str(userdata.find_sentences)+str(userdata.current_object_name)
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, self.string)
                rospy.sleep(1.0)
                return 'succeeded'

            except:
                return 'aborted'

# 注册人名1,返回
class FaceRegister1(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['people_name'])
        self.xm_faceRecognition1 = rospy.ServiceProxy('get_name', xm_FaceDetect)

    def execute(self, userdata):
        try:
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/face_recognition/register_face111.py'], shell=False)
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
            rospy.sleep(1.0)
            print a.poll()
            if a.returncode != None:
                a.wait()
                self.killPro()
                return 'aborted'
        except:
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        try:
            self.xm_faceRecognition1.wait_for_service(timeout=30.0)
            req = xm_FaceDetectRequest()
            req.people_id = 0
            rospy.logwarn(req)

            res = self.xm_faceRecognition1.call(req)
            userdata.people_name.append(res.people_name)

        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'
        rospy.logwarn(res)
        self.killPro()
        return 'succeeded'

    def killPro(self):
        try:
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill', '-9', pid], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w') as f:
                f.write('')

        except Exception, e:
            rospy.logerr('No such process ')


# 注册人名2,返回
class FaceRegister2(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['people_name'])
        self.xm_faceRecognition1 = rospy.ServiceProxy('get_name', xm_FaceDetect)

    def execute(self, userdata):
        try:
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/face_recognition/register_face222.py'], shell=False)
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
            rospy.sleep(1.0)
            print a.poll()
            if a.returncode != None:
                a.wait()
                self.killPro()
                return 'aborted'
        except:
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        try:
            self.xm_faceRecognition1.wait_for_service(timeout=30.0)

            req = xm_FaceDetectRequest()
            req.people_id = 0
            rospy.logwarn(req)

            res = self.xm_faceRecognition1.call(req)
            userdata.people_name.append(res.people_name)

        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'
        rospy.logwarn(res)
        self.killPro()
        return 'succeeded'

    def killPro(self):
        try:
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill', '-9', pid], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w') as f:
                f.write('')

        except Exception, e:
            rospy.logerr('No such process ')


# 注册人名3,返回
class FaceRegister3(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['people_name'])
        self.xm_faceRecognition1 = rospy.ServiceProxy('get_name', xm_FaceDetect)

    def execute(self, userdata):
        try:
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/face_recognition/register_face333.py'], shell=False)
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
            rospy.sleep(1.0)
            print a.poll()
            if a.returncode != None:
                a.wait()
                self.killPro()
                return 'aborted'
        except:
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        try:
            self.xm_faceRecognition1.wait_for_service(timeout=30.0)

            req = xm_FaceDetectRequest()
            req.people_id = 0
            rospy.logwarn(req)

            res = self.xm_faceRecognition1.call(req)
            userdata.people_name.append(res.people_name)

        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'
        rospy.logwarn(res)
        self.killPro()
        return 'succeeded'

    def killPro(self):
        try:
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill', '-9', pid], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w') as f:
                f.write('')

        except Exception, e:
            rospy.logerr('No such process ')

# 注册人名4,返回
class FaceRegister4(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['people_name'])
        self.xm_faceRecognition1 = rospy.ServiceProxy('get_name', xm_FaceDetect)

    def execute(self, userdata):
        try:
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/face_recognition/register_face444.py'], shell=False)
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
            rospy.sleep(1.0)
            print a.poll()
            if a.returncode != None:
                a.wait()
                self.killPro()
                return 'aborted'
        except:
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        try:
            self.xm_faceRecognition1.wait_for_service(timeout=30.0)

            req = xm_FaceDetectRequest()
            req.people_id = 0
            rospy.logwarn(req)

            res = self.xm_faceRecognition1.call(req)
            userdata.people_name.append(res.people_name)

        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'
        rospy.logwarn(res)
        self.killPro()
        return 'succeeded'

    def killPro(self):
        try:
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill', '-9', pid], shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt", 'w') as f:
                f.write('')

        except Exception, e:
            rospy.logerr('No such process ')
