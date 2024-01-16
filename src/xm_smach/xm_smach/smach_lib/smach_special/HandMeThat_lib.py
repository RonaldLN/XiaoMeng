#!/usr/bin/env python
# encoding:utf8
import re
from turtle import pos, position
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
        print("pid:         ",pid)
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
        pos.orientation.w=1

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


class GetPersonPosition2(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['people_pos','detect2_turn'])
        self.xm_findperson2 = rospy.ServiceProxy(
            'judge_guest', xm_judgeguest)

    def execute(self, userdata):
        try:
            a = subprocess.Popen(
                ['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/Serveguest/judge_guest.py', '-d', 'true'], shell=False)
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
            self.xm_findperson2.wait_for_service(timeout=30.0)

            req = xm_judgeguestRequest()
            req.num = 0
            rospy.logwarn(req)

            res = self.xm_findperson2.call(req)

            if res.coordinate.point.x != 999 or res.coordinate.point.y != -999 or res.coordinate.point.z != -999:
                rospy.loginfo("find people!")
            if res.coordinate.point.x == 999 and res.coordinate.point.y == -999 and res.coordinate.point.z == -999:
                rospy.loginfo("Restart to detect person!")
                userdata.detect2_turn+=1
                self.killPro()
                if userdata.detect2_turn>3 :
                    return 'error'
                return 'aborted'
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'error'

        # if res.object.pos.point.x == -10.000 or res.object.pos.point.x == 10.000 or res.object.pos.point.x == 5:
        #     rospy.logerr('find nothing')
        #     self.killPro()
        #     return 'aborted'

        rospy.logwarn(res)
        # 传给底盘的人坐标直接使用摄像头的坐标
        # 这里应该有问题，应该有tf变换，但不会使用，测试有问题的话，留给叶总
        # pos = Pose()
        # pos.position.x = res.coordinate.point.x
        # pos.position.y = res.coordinate.point.y
        # pos.position.z = res.coordinate.point.z

        # userdata.people_pos = userdata.people_pos[-1]
        self.killPro()
        userdata.detect2_turn=0
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


class GetTaskH(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target', 'action', 'answer'])

        # self.speech_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        # rospy.wait_for_service('speech_core')
        self.speech_client = rospy.ServiceProxy('speech_core', speech_to_smach)

    def execute(self, userdata):
        try:
            getattr(userdata, 'target')
            getattr(userdata, 'action')
            getattr(userdata, 'answer')
        except:
            rospy.logerr('No param specified')
            return 'error'
        try:
            self.speech_client.wait_for_service(timeout=10)
        # command = 1用于gpsr
            response = self.speech_client.call(command=8)
            print(response.action)
            print(response.object)
        except:
            rospy.logerr('wrong in call the service')
            return 'error'
        # if response.action[0] == 'stop':
        #     rospy.logerr('response wrong!')
        #     return 'aborted'
        if response.num > 0:
            # userdata.action.append(response.action[0])
            # userdata.target.append(response.object[0])
            userdata.action.append(response.action)
            userdata.target.append(response.object)
            userdata.answer = response.answer
            rospy.logwarn(userdata.action)
            rospy.logwarn(userdata.target)
            rospy.logwarn(userdata.answer)
            # if userdata.action[1] == 'grasp':
            # something interest here
            # userdata.action[0] = 'nav_grasp'
            # userdata.action.append('place')
            # userdata.task_num += 1
            return 'succeeded'
        else:
            return 'aborted'


class NextDoH(State):  # 跳转action
    def __init__(self):
        State.__init__(self,
                       # nav_grasp is something interesting
                       outcomes=['succeeded', 'aborted', 'go', 'error'],
                       input_keys=['action', 'task_num'],
                       io_keys=['current_task'])

    def execute(self, userdata):
        try:
            action = userdata.action
            current_task = userdata.current_task
            task_num = userdata.task_num
        except:
            rospy.logerr('No param specified')
            return 'error'
        userdata.current_task += 1

        # 测试运行成功
        # current_task目前测使的次数
        # task_num 测试总次数
        if userdata.current_task == task_num:
            return 'succeeded'

        current_action = action[userdata.current_task]

        if current_action == 'go':
            return 'go'

        else:
            # no avaiable action find
            # userdata.current_task_out -1
            userdata.current_task -= 1
            return 'aborted'


class Check_Receive_Task(State):  # 跳转action
    def __init__(self):
        State.__init__(self,
                       # nav_grasp is something interesting
                       outcomes=['succeeded', 'continue', 'error'],
                       input_keys=['action', 'turn'])

    def execute(self, userdata):
        try:
            action = userdata.action
            turn = userdata.turn
        except:
            rospy.logerr('No param specified')
            return 'error'

        if len(action) == turn:
            return 'succeeded'
        else:
            return 'continue'


class CheckPeopleOrObject(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['people', 'object'],
                       input_keys=['current_target'])

    def execute(self, userdata):
        if userdata.current_target == "person":
            return 'people'
        else:
            return 'object'


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
        self.xm_findobject = rospy.ServiceProxy('get_position_object', xm_ObjectDetect)

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
            name.replace(' ' , '' , name.count(' '))   
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
                       io_keys=['target','current_turn_num'],
                       output_keys=['target_camera_point', 'object_state', 'table_depth'])
        self.xm_findobject = rospy.ServiceProxy('get_position_object', xm_ObjectDetect)

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
            name.replace(' ' , '' , name.count(' '))   
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


class FindGuest(State):  # 计算客人位置
    def __init__(self):
        State.__init__(self,
                       # nav_grasp is something interesting
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['current_people_pos'],
                       input_keys=['people_pos', 'current_guest_num'])

    def execute(self, userdata):
        try:
            people_pos = userdata.people_pos
            current_guest_num = userdata.current_guest_num
            rospy.loginfo('people_pos: ', people_pos)
            rospy.loginfo('current_guest_num: ', current_guest_num)
        except:
            rospy.logerr('No param specified')
            return 'error'
        userdata.current_people_pos = people_pos[current_guest_num]
        rospy.loginfo('current_people_pos: ', userdata.current_people_pos)
        return 'succeeded'


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

        # YeQingxin de Daima
        while deta >= 0.6 and nav_counter < 150:
            nav_counter += 1
            (trans, rot) = self.tf_listener.lookupTransform(
            'map', 'base_link', rospy.Time())
            deta = sqrt(pow(trans[0] - pos_xm.position.x, 2) +
                        pow(trans[1] - pos_xm.position.y, 2))
            rospy.logerr("deta:" + str(deta))
            
            # goal.target_pose.pose.orientation.z = rot[2]
            # goal.target_pose.pose.orientation.w = rot[3]
            if nav_counter <= 10 :
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

        goal.target_pose.pose.orientation.z = (pos_xm.position.x - trans[0]) / deta
        goal.target_pose.pose.orientation.w = (pos_xm.position.y - trans[1]) / deta
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
            if nav_counter <= 10 :
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
        self.tf_camera_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))

    def execute(self, userdata):
        try:
            (trans, rot)= self.tf_camera_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
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
        rospy.logerr(math.sin(r * math.pi /180))
        rospy.logerr(math.cos(r * math.pi /180))


        tempPose= Pose()
        xBefore = trans[0]+ userdata.people_pos[-1].position.x * math.cos(r * math.pi /180) + userdata.people_pos[-1].position.y * math.sin(r * math.pi /180)
        yBefore = trans[1] + userdata.people_pos[-1].position.x  *  math.sin(r * math.pi /180) + userdata.people_pos[-1].position.y * math.cos(r * math.pi /180)
        tempPose.position.x = trans[0]+ (userdata.people_pos[-1].position.x - 0.75) * math.cos(r * math.pi /180) + userdata.people_pos[-1].position.y * math.sin(r * math.pi /180)
        tempPose.position.y = trans[1] + (userdata.people_pos[-1].position.x - 0.75) *  math.sin(r * math.pi /180) + userdata.people_pos[-1].position.y * math.cos(r * math.pi /180)
        tempPose.position.z=0
        r = math.atan((yBefore - tempPose.position.y) / (xBefore - tempPose.position.x))
        sinr = math.sin(r)
        cosr = math.cos(r)
        w = cosr
        z = sinr

        tempPose.orientation.z=z
        tempPose.orientation.w=w
        userdata.people_pos[-1]=tempPose
        return 'succeeded'

class PositionJudge(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],  # preemeted
                       io_keys=['people_pos'])

    def execute(self, userdata):
        if len(userdata.people_pos)==1 :
            return 'succeeded'
        try:
            if self.positionOnList(userdata.people_pos[:-1],userdata.people_pos[-1]) :
                del userdata.people_pos[-1]
                return 'error'
            return 'succeeded'
        except:
            rospy.loginfo('PositionJudge try again')
            return 'aborted'
    def positionOnList(people_pos,temp_pos):
        # 如果该坐标在客人列表中，返回True，否则返回False
        for index in people_pos:
            if temp_pos<index.position.x+0.5 and temp_pos>index.position.x-0.5 and temp_pos<index.position.y+0.5 and temp_pos>index.position.y-0.5 :
                return True
        return False