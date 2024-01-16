#!/usr/bin/env python
# encoding:utf8


'''
Author: yishui
Date: 2022-11-25
LastEditTime: 2022-11-25 22:54:18
LastEditors: yishui
Description: Codes for Receptionist
FilePath: ~/catkin_ws/src/xm_smach/xm_smach/tests/yishui/Receptionist
'''


import rospy
from std_msgs.msg import String, Int32, Bool, Header
from smach import StateMachine
from smach_ros import IntrospectionServer
from smach_common.common import *
from tf.transformations import quaternion_matrix
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from control_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from copy import deepcopy

def get_pid(name):
    pid = ''
    with open("/home/xm/vision_pid/{}.txt".format(name)) as f:
        pid = f.read()
    return pid


class Receptionist_Platdown(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded', 'error'])
        self.plat_client = actionlib.SimpleActionClient("/plat_controller/follow_joint_trajectory", GripperCommandAction)
    
    def execute(self,userdata):
        goal = GripperCommandGoal()
        goal.command.position = -0.07
        rospy.logwarn("^^^^^^^^^^^^^^")
        self.plat_client.wait_for_server()
        self.plat_client.send_goal(goal)
        rospy.logwarn("**************8")
        self.plat_client.wait_for_result(rospy.Duration(10.0))
        rospy.loginfo("...set the plat") 

        return 'succeeded'

class GetNameAndDrink(State):
    '''
    获取名字和饮料并插入列表
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        io_keys =['name_list','drink_list'])

        self.speech_client = rospy.ServiceProxy('speech_core', speech_to_smach)

    def execute(self,userdata):
        try:
            getattr(userdata, 'name_list')
            getattr(userdata, 'drink_list')
        except:
            rospy.logerr('no param')
            return 'aborted'
        else :
            #example :  I am Tom and I want ice tea.
            res = self.speech_client.call(command=4)
            rospy.logwarn(res)

            name = res.name
            drink = res.drink
            if drink == '' or name == '':
                return 'aborted'
            rospy.logwarn(name)
            rospy.logwarn(drink)
            userdata.name_list.append(name)
            userdata.drink_list.append(drink)
            rospy.logwarn(userdata.name_list)
            rospy.logwarn(userdata.drink_list)
            return 'succeeded'

class Point_Guest(State):
    '''
    指向的机械臂姿态
    '''
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['angle'])
        self.arm_client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    def execute(self,userdata):
        
        trajectory = JointTrajectory()
        joint_names = ["arm_joint_0", "arm_joint_1", "arm_joint_2","arm_joint_3"]	
        trajectory.joint_names = joint_names

        pos_demo = JointTrajectoryPoint()
        pos_demo.positions = [0.0 for i in joint_names]
        pos_demo.velocities = [0.0 for i in joint_names]
        pos_demo.accelerations = [0.0 for i in joint_names]
        pos_demo.time_from_start = rospy.Duration(5.0)

        arm_waypoints = list()
        joints = [userdata.angle,-0.8067,0.4,0]
        pos = deepcopy(pos_demo)
        pos.positions = joints
        trajectory.points.append(pos)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result(rospy.Duration(60.0))

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
        joint_names = ["arm_joint_0", "arm_joint_1", "arm_joint_2","arm_joint_3"]	
        trajectory.joint_names = joint_names

        pos_demo = JointTrajectoryPoint()
        pos_demo.positions = [0.0 for i in joint_names]
        pos_demo.velocities = [0.0 for i in joint_names]
        pos_demo.accelerations = [0.0 for i in joint_names]
        pos_demo.time_from_start = rospy.Duration(5.0)

        arm_waypoints = list()
        joints = [userdata.angle,0,1.57,0]

        pos = deepcopy(pos_demo)
        pos.positions = joints
        trajectory.points.append(pos)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result(rospy.Duration(60.0))

        return 'succeeded'

class GetHostWord(State):
    '''
    生成介绍的语句
    '''
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['name_list','drink_list'],
                        output_keys = ['host_sentence'])
    def execute(self, userdata):
        try:
            host_sentene = ""
            
            name_list = deepcopy(userdata.name_list)
            drink_list = deepcopy(userdata.drink_list)
            
            rospy.logwarn(name_list)
            rospy.logwarn(drink_list)

            host_name = name_list.pop(0)
            host_drink = drink_list.pop(0)

            host_sentene = " "+ host_name + " is the host and he likes " + host_drink+"."

            userdata.host_sentence = host_sentene
            return 'succeeded'
        except Exception as  e:
            rospy.logerr(e)
            return 'aborted'

class GetGuestWord(State):
    '''
    生成介绍的语句
    '''
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['name_list','drink_list'],
                        output_keys = ['guest_sentence'])
    def execute(self, userdata):
        try:
            guest_sentene = ""
            
            name_list = deepcopy(userdata.name_list)
            drink_list = deepcopy(userdata.drink_list)
            
            rospy.logwarn(name_list)
            rospy.logwarn(drink_list)
            
            recepition_people_name = name_list.pop(1)
            recepition_people_drink = drink_list.pop(1)

            
            guest_sentene = " this is "+recepition_people_name+" and he likes " + recepition_people_drink+"."
            
            userdata.guest_sentence = guest_sentene
            return 'succeeded'
        except Exception as  e:
            rospy.logerr(e)
            return 'aborted'

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
            receptionist_sentence = ""
            
            name_list = deepcopy(userdata.name_list)
            drink_list = deepcopy(userdata.drink_list)
            
            rospy.logwarn(name_list)
            rospy.logwarn(drink_list)
            
            recepition_people_name = name_list.pop()
            recepition_people_drink = drink_list.pop()

            
            receptionist_sentence = " this is "+recepition_people_name+" and he likes " + recepition_people_drink+"."

            userdata.receptionist_sentence = receptionist_sentence
            return 'succeeded'
        except Exception as  e:
            rospy.logerr(e)
            return 'aborted'

class TakeAPhoto(State):
    
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['seat_angle_list','index','des'],
                        output_keys = ['index','des'])
        pi = 3.1415926535
        #rospy.wait_for_service('receptionist')
        self.xm_takePhoto = rospy.ServiceProxy('receptionist_take_photo', xm_takePhoto)
        #self.index = 0
        #self.xm_getAngle = rospy.ServiceProxy('receptionist', Int32)
        #rospy.Subscriber('follow', xm_FollowPerson, self.move_base_follow_cb)
        #rospy.init_node('receptionist_subscriber', anonymous=True)

    def execute(self,userdata):
        try:
            a = subprocess.Popen(['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/Receptionist/mrsupw_take_a_photo.py','-d','true'] ,shell =False)
            with open("/home/xm/vision_pid/receptionist.txt",'w+') as f:
                print('-------------------')
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('-------------------')
                f.write(str(a.pid))
            #rospy.sleep(7.0)
            # rospy.sleep(10000.0)
            print("!!!")
            #rospy.spin()
            print("@@@@@@@@@@@@")
            # a.wait()
            # if a.poll() != 0:
            #     rospy.logerr('error')
        except Exception as e:
            print(e)
            rospy.logerr('Subscriber process error')
            return 'error'
        
        try:
            self.xm_takePhoto.wait_for_service(timeout=30.0)
            #req = xm_getAngleRequest()
            res = self.xm_takePhoto.call(0)
            rospy.logwarn(res)
            userdata.des = res.des
            

            pid = ''
            with open('/home/xm/vision_pid/receptionist.txt') as f:
                pid = f.read()
            print('killing!',pid)
            print('killing!',pid)
            print('killing!',pid)
            subprocess.Popen(['kill -9 {}'.format(pid)],shell=True)
            rospy.sleep(1.0)
            return 'succeeded'
        except Exception as e:
            rospy.logerr(e)
            return 'aborted'

class GetGuestAngle(State):
    
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['seat_angle_list','index'],
                        output_keys = ['index','seat_angle'])
        pi = 3.1415926535
        #rospy.wait_for_service('receptionist')
        self.xm_getAngle = rospy.ServiceProxy('receptionist', xm_getPeopleAngle)
        #self.index = 0
        #self.xm_getAngle = rospy.ServiceProxy('receptionist', Int32)
        #rospy.Subscriber('follow', xm_FollowPerson, self.move_base_follow_cb)
        #rospy.init_node('receptionist_subscriber', anonymous=True)

    def execute(self,userdata):
        try:
            a = subprocess.Popen(['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/Receptionist/mrsupw_receptionist_local.py','-d','true'] ,shell =False)
            with open("/home/xm/vision_pid/receptionist.txt",'w+') as f:
                print('-------------------')
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('-------------------')
                f.write(str(a.pid))
            #rospy.sleep(7.0)
            # rospy.sleep(10000.0)
            print("!!!")
            #rospy.spin()
            print("@@@@@@@@@@@@")
            # a.wait()
            # if a.poll() != 0:
            #     rospy.logerr('error')
        except Exception as e:
            print(e)
            rospy.logerr('Subscriber process error')
            return 'error'
        
        try:
            self.xm_getAngle.wait_for_service(timeout=30.0)
            req = xm_getPeopleAngleRequest()
            req.name = 'guest'
            res = self.xm_getAngle.call(req)
            userdata.index = res.index
            self.seat_angle_list = userdata.seat_angle_list
            
            rospy.logwarn(userdata.index)
            userdata.seat_angle = self.seat_angle_list[userdata.index]
            pid = ''
            with open('/home/xm/vision_pid/receptionist.txt') as f:
                pid = f.read()
            print('killing!',pid)
            print('killing!',pid)
            print('killing!',pid)
            subprocess.Popen(['kill -9 {}'.format(pid)],shell=True)
            rospy.sleep(1.0)
            return 'succeeded'
        except Exception as e:
            rospy.logerr(e)
            return 'aborted'

class GetHost1Angle(State):
    
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['seat_angle_list','index'],
                        output_keys = ['index','seat_angle'])
        pi = 3.1415926535
        #rospy.wait_for_service('receptionist')
        self.xm_getAngle = rospy.ServiceProxy('receptionist', xm_getPeopleAngle)
        #self.index = 0
        #self.xm_getAngle = rospy.ServiceProxy('receptionist', Int32)
        #rospy.Subscriber('follow', xm_FollowPerson, self.move_base_follow_cb)
        #rospy.init_node('receptionist_subscriber', anonymous=True)

    def execute(self,userdata):
        try:
            a = subprocess.Popen(['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/Receptionist/mrsupw_receptionist_local.py','-d','true'] ,shell =False)
            with open("/home/xm/vision_pid/receptionist.txt",'w+') as f:
                print('-------------------')
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('-------------------')
                f.write(str(a.pid))
            #rospy.sleep(7.0)
            # rospy.sleep(10000.0)
            print("!!!")
            #rospy.spin()
            print("@@@@@@@@@@@@")
            # a.wait()
            # if a.poll() != 0:
            #     rospy.logerr('error')
        except Exception as e:
            print(e)
            rospy.logerr('Subscriber process error')
            return 'error'
        
        try:
            self.xm_getAngle.wait_for_service(timeout=30.0)
            #req = xm_getAngleRequest()
            req = xm_getPeopleAngleRequest()
            req.name = 'host1'
            res = self.xm_getAngle.call(req)
            userdata.index = res.index
            self.seat_angle_list = userdata.seat_angle_list
            
            rospy.logwarn(userdata.index)
            userdata.seat_angle = self.seat_angle_list[userdata.index]
            pid = ''
            with open('/home/xm/vision_pid/receptionist.txt') as f:
                pid = f.read()
            print('killing!',pid)
            print('killing!',pid)
            print('killing!',pid)
            subprocess.Popen(['kill -9 {}'.format(pid)],shell=True)
            rospy.sleep(1.0)
            return 'succeeded'
        except Exception as e:
            rospy.logerr(e)
            return 'aborted'

class GetHost2Angle(State):
    
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['seat_angle_list','index'],
                        output_keys = ['index','seat_angle'])
        pi = 3.1415926535
        #rospy.wait_for_service('receptionist')
        self.xm_getAngle = rospy.ServiceProxy('receptionist', xm_getPeopleAngle)
        #self.index = 0
        #self.xm_getAngle = rospy.ServiceProxy('receptionist', Int32)
        #rospy.Subscriber('follow', xm_FollowPerson, self.move_base_follow_cb)
        #rospy.init_node('receptionist_subscriber', anonymous=True)

    def execute(self,userdata):
        try:
            a = subprocess.Popen(['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/Receptionist/mrsupw_receptionist_local.py','-d','true'] ,shell =False)
            with open("/home/xm/vision_pid/receptionist.txt",'w+') as f:
                print('-------------------')
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('-------------------')
                f.write(str(a.pid))
            #rospy.sleep(7.0)
            # rospy.sleep(10000.0)
            print("!!!")
            #rospy.spin()
            print("@@@@@@@@@@@@")
            # a.wait()
            # if a.poll() != 0:
            #     rospy.logerr('error')
        except Exception as e:
            print(e)
            rospy.logerr('Subscriber process error')
            return 'error'
        
        try:
            self.xm_getAngle.wait_for_service(timeout=30.0)
            req = xm_getPeopleAngleRequest()
            req.name = 'host2'
            res = self.xm_getAngle.call(req)
            userdata.index = res.index
            self.seat_angle_list = userdata.seat_angle_list
            
            rospy.logwarn(res)
            userdata.seat_angle = self.seat_angle_list[userdata.index]
            pid = ''
            with open('/home/xm/vision_pid/receptionist.txt') as f:
                pid = f.read()
            print('killing!',pid)
            print('killing!',pid)
            print('killing!',pid)
            subprocess.Popen(['kill -9 {}'.format(pid)],shell=True)
            rospy.sleep(1.0)
            return 'succeeded'
        except Exception as e:
            rospy.logerr(e)
            return 'aborted'

class GetSeatAngle(State):
    
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['seat_angle_list','index'],
                        output_keys = ['index','seat_angle'])
        pi = 3.1415926535
        #rospy.wait_for_service('receptionist')
        self.xm_getAngle = rospy.ServiceProxy('receptionist', xm_getPeopleAngle)
        #self.index = 0
        #self.xm_getAngle = rospy.ServiceProxy('receptionist', Int32)
        #rospy.Subscriber('follow', xm_FollowPerson, self.move_base_follow_cb)
        #rospy.init_node('receptionist_subscriber', anonymous=True)

    def execute(self,userdata):
        try:
            a = subprocess.Popen(['python3', '/home/xm/catkin_ws/src/xm_vision/src/scripts/Receptionist/mrsupw_receptionist_local.py','-d','true'] ,shell =False)
            with open("/home/xm/vision_pid/receptionist.txt",'w+') as f:
                print('-------------------')
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('-------------------')
                f.write(str(a.pid))
            rospy.sleep(7.0)
            # rospy.sleep(10000.0)
            print("!!!")
            #rospy.spin()
            print("@@@@@@@@@@@@")
            # a.wait()
            # if a.poll() != 0:
            #     rospy.logerr('error')
        except Exception as e:
            print(e)
            rospy.logerr('Subscriber process error')
            return 'error'
        
        try:
            self.xm_getAngle.wait_for_service(timeout=30.0)
            #req = xm_getAngleRequest()
            res = self.xm_getAngle.call('empty')
            userdata.index = res.index
            self.seat_angle_list = userdata.seat_angle_list
            
            rospy.logwarn(userdata.index)
            userdata.seat_angle = self.seat_angle_list[userdata.index]
            pid = ''
            with open('/home/xm/vision_pid/receptionist.txt') as f:
                pid = f.read()
            print('killing!',pid)
            print('killing!',pid)
            print('killing!',pid)
            subprocess.Popen(['kill -9 {}'.format(pid)],shell=True)
            rospy.sleep(1.0)
            return 'succeeded'
        except Exception as e:
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

        except Exception as e:
            rospy.logerr(e)
            userdata.point_sentence = "please seat in the sofa"
            return 'error'

# 返回识别的人名,并将人名记录到映射中
class FaceRecognition1(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['name_list'],
                       io_keys=['face_name_dict'])
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
            rospy.logwarn(res)
            userdata.face_name_dict[userdata.name_list[-1]]=res.people_name
            rospy.logwarn(userdata.face_name_dict)
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

# 返回识别的人名,并且比对是否为主人
class FaceRecognition2(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['true', 'false','aborted', 'error'],
                       input_keys=['name_list'],
                       io_keys=['face_name_dict'])
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
            rospy.logwarn(res)
            # 如果识别出的名字和主人名字相同，返回true
            if res.people_name == userdata.name_list[0]:
                self.killPro()
                return 'true'
            else:
                self.killPro()
                return 'false'
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
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




class FindTwoPeople(State):
    '''
    no use
    根据图像传过来的数据，确定抓取时xm的位置
    up_down变量为1，则使用上面的摄像头；up_down变量为0，则使用下面的摄像头。
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

            rospy.loginfo("get people!!!")
            if res.pos1.point.x == 0 and res.pos1.point.y == 0 and res.pos1.point.z == 0:
                rospy.logwarn("\ncan't find object!1")
                self.killPro()
                return 'aborted'

            if res.pos2.point.x == 0 and res.pos2.point.y == 0 and res.pos2.point.z == 0:
                rospy.logwarn("\ncan't find object!2")
                self.killPro()
                return 'aborted'

            # if res.pos3.point.x == 0 and res.pos3.point.y == 0 and res.pos3.point.z == 0:
            #     rospy.logwarn("\ncan't find object!3")
            #     self.killPro()
            #     return 'aborted'

            userdata.people_pos.append(res.pos1)
            userdata.people_pos.append(res.pos2)

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
            camera_link = "camera_link_up"
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
            elif distance >3.3:
                move_in_line = distance - 2.3
            else:
                move_in_line = distance - 2
            
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

class IntroduceHost(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['name_list','drink_list'])

        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)
    
    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            sentences = 'the host name is '+ userdata.name_list[0]+'and he likes '+userdata.drink_list[0]
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, sentences)
                rospy.sleep(1.0)
                return 'succeeded'
              
            except:
                return 'aborted'

class IntroduceGuest1(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['name_list','drink_list'])

        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)
    
    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            sentences = 'the first guest name is '+ userdata.name_list[0]+'and he likes'+userdata.drink_list[0]
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, sentences)
                rospy.sleep(1.0)
                return 'succeeded'
              
            except:
                return 'aborted'

class IntroduceGuest2(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['name_list','drink_list'])

        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)
    
    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            sentences = 'the second guest name is '+ userdata.name_list[1]+'and he likes'+userdata.drink_list[1]
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, sentences)
                rospy.sleep(1.0)
                return 'succeeded'
              
            except:
                return 'aborted'

class NavStackReceptionist(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],  # preemeted
                       input_keys=['pos_xm','med_pos'])
        self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        try:
            getattr(userdata, 'pos_xm')
            getattr(userdata, 'med_pos')
        except:
            rospy.logerr('No param specified ')
            return 'error'
        else:
            rospy.logwarn(userdata.pos_xm)
            rospy.logwarn(userdata.med_pos)
            self.nav_client.wait_for_server(rospy.Duration(120))

            clear_client = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            # clear_client = rospy.ServiceProxy('/move_base_simple/goal', Empty)    
            req = EmptyRequest()
            res = clear_client.call()
            print(9999999999999)
            self.nav_thing(userdata.med_pos)
            self.nav_thing(userdata.pos_xm)

            return 'succeeded'

    def nav_thing(self, pos_xm):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = pos_xm
        goal.target_pose.header.stamp = rospy.Time.now()
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        
        self.nav_client.send_goal(goal)


        # self.pubulisher = rospy.Publisher(
        #     '/move_base_simple/goal', Pose, queue_size=1)

        # self.pose = PoseStamped()
        # self.pose.header.stamp = rospy.Time.now()
        # self.pose.header.frame_id = "map"

        # self.pose.pose = pos_xm
        # print("!!!!!!!!!!!!!!!!!!!!!!!!!!")
        # print(self.pose)
        # self.pubulisher.publish(self.pose)
        # self.pubulisher.publish({
        #     'header':self.pose.header,
        #     'point':self.pose.pose
        # })


        nav_counter = 0
        while self.nav_client.get_state() != GoalStatus.SUCCEEDED and nav_counter < 300:
            nav_counter += 1
            print("123456789")
            # self.nav_client.send_goal(goal)
            if self.preempt_requested():
                rospy.logerr('preempted')
                return 'succeeded'
            else:
                
                pass
            rospy.sleep(0.1)
            

        # (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time())
        # time.sleep(2)
        self.tf_listener.waitForTransform('map','base_link',rospy.Time.now(),rospy.Duration(4.0))

        (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))

        # (trans, rot) = self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(), rospy.Duration(1.0))
        deta = sqrt(pow(trans[0] - pos_xm.position.x, 2) + pow(trans[1] - pos_xm.position.y, 2))
        rospy.logerr("deta:" + str(deta))

        if self.nav_client.get_goal_status_text() == 'Goal reached.':
            rospy.loginfo("nav task executes successfully ^_^")
            return 'succeeded'
        else:
            rospy.logerr("xm cannot find the way  T_T")
            return 'aborted'