#!/usr/bin/env python
# encoding:utf8

import rospy
from smach import State, UserData
from smach_ros import SimpleActionState, MonitorState,ServiceState
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from xm_msgs.srv import *
import subprocess
import time
import tf
from geometry_msgs.msg import *
from math import *
from std_msgs.msg import *
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from copy import deepcopy
from math import pi
# the vital state used in whoiswho smach 


# the face_reco state 
# the position is a list of PointStamped()
# we cannot do any job for reading the output_keys
# io_keys can avoid the problem of reading or writing only of keys
# the name_id is the Id need we to recognize which is int32
# we should deal the data in this state, or the data from cv may be error
# then you will see that the type of navstack can be all Pose()
def killPro():
    try:
        pid_str = subprocess.check_output('ps -aux | grep people_identify.py' , shell= True)
        pid_str1 = pid_str.splitlines()[0].split()[1]
        rospy.logwarn(pid_str1)
        subprocess.call('kill -9 '+pid_str1 , shell = True)
    except Exception,e:
        rospy.logerr('No such process ')

class FaceReco(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error','again','train_error','turn_l','turn_r'],
                        input_keys = ['name_id' , 'distance'],
                        io_keys = ['position'],
                        output_keys = ['num_list'])
        #service name should be specified when used 
        self.face_reco_client = rospy.ServiceProxy('get_position',xm_ObjectDetect)
        self.tf_listener =tf.TransformListener()
        self.ps = Pose()
        self.position =list()
        self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel',Twist,queue_size=1)

    def execute(self,userdata):
        rospy.loginfo('face recongize begin')
        req  = xm_ObjectDetectRequest()
        self.distance = userdata.distance
        try:
            name_id = userdata.name_id
            subprocess.call("xterm -e rosrun xm_vision people_identify.py &",shell=True)
            rospy.sleep(2.0)
        except:
            rospy.logerr('No param specified')
            return 'error'
        req.object_name ="person"
        req.people_id = name_id
        # call the cv service
        try:
            self.face_reco_client.wait_for_service(10.0)
            res= self.face_reco_client.call(req)
            print res

            
                #return 'aborted'
            try:
                pid_str = subprocess.check_output('ps -aux | grep people_identify.py' , shell= True)
                pid_str1 = pid_str.splitlines()[0].split()[1]
                rospy.logwarn(pid_str1)
                subprocess.call('kill -9 '+pid_str1 , shell = True)
            except Exception,e:
                rospy.logerr('No such process ')

        except Exception,e:
            

            rospy.logerr('can not call!!!')
            rospy.logerr(e)
            rospy.sleep(5.0)
            return 'aborted'
        else:
            
            if len(res.object)==0 :
                killPro()
                return 'aborted'
            res.object.sort(key = lambda obj:obj.pos.point.x)
            if res.object[0].state ==0:
                self.true_person = list()
                self.error_num_l = list()
                self.error_num_r = list()

                if(name_id == -1  and len(res.object)<3):
                    rospy.logerr('people<3')
                    self.ahead_justice()
                    killPro()
                    return 'aborted'
                

                for i in range(len(res.object)):
                    print res.object[i].pos.point.x
                    if res.object[i].pos.point.x >= -11.0 and res.object[i].pos.point.x <= 11.0 :
                    ######################这个地方处理很有问题目前先这么决定，以后一定要改########################
                        if abs(res.object[i].pos.point.x - 10.0) < 0.00001 and abs(res.object[i].pos.point.y - 10.0 < 0.00001) and abs(res.object[i].pos.point.z - 10.0 < 0.00001):
                            rospy.logerr("right")
                            obj = self.right_justice(res.object[i])
                            self.data_deal(obj.pos ,self.distance)
                            self.true_person.append(obj)
                            # return 'turn_r'
                        elif abs(res.object[i].pos.point.x + 10.0 < 0.00001) and abs(res.object[i].pos.point.y + 10.0 < 0.00001) and abs(res.object[i].pos.point.z + 10.0 < 0.00001):
                            rospy.logerr("left")
                            obj = self.left_justice(res.object[i])
                            self.data_deal(obj.pos,self.distance)
                            self.true_person.append(obj)
                            # return 'turn_l'
                        else:
                            rospy.logerr("ok")
                            rospy.logwarn("i")
                            rospy.logwarn(i)
                            rospy.logwarn(res.object[i].pos)

                            if(name_id == -1 and res.object[i].name == 2):
                                rospy.logerr('name_id = 0?')
                                rospy.logerr(res.object[i].name)
                                self.data_deal(res.object[i].pos,self.distance-0.2)
                            else:
                                self.data_deal(res.object[i].pos ,self.distance)
                                
                            self.true_person.append(res.object[i])
                            self.position.append(self.ps)
                            rospy.logwarn(self.ps) 
                    # self.position.sort(key= lambda Pose: Pose.position.y)
                if len(self.true_person)==0:
                    killPro()
                    return 'aborted'
                if name_id !=-1:
                    
                    userdata.position =self.position.pop()
                    rospy.logerr(name_id)
                else:
                    self.out_list = [int(obj.name) for obj in self.true_person]
                    print self.out_list
                    # emmmmmm所以这个操作其实有点孬，用于pop()，详见GetID，翻转一次来源于GetValue
                    # if len(self.out_list)>1:
                        # self.out_list.reverse()
                    helper = self.out_list
                    # else:
                    #     helper = self.out_list
                    userdata.num_list = helper
                    rospy.logerr(helper)
                    userdata.position.extend(self.position)
                
                killPro()
                return 'succeeded'
            elif res.object[0].state ==-1:
                rospy.logwarn("I will recognize again")
                self.ahead_justice()
                killPro()
                return 'again'
            elif res.object[0].state ==-2:
                rospy.logwarn("the train may cause error")
                killPro()
                return 'train_error'
            elif res.object[0].state ==-3:
                rospy.logwarn("the position is not fit,turn left")
                killPro()
                return 'turn_l'
            elif res.object[0].state ==-4:
                rospy.logwarn("the position is not fit, turn right")
                killPro()
                return 'turn_r'
            else :
                killPro()
                return 'aborted'
    def left_justice(self,obj):
        sub_req = xm_ObjectDetectRequest()
        sub_req.object_name ="person"
        sub_req.people_id = obj.name
        angular_speed = 1.0
        self.turn = Twist()
        self.turn.linear.x = 0.0
        self.turn.linear.y = 0.0
        self.turn.linear.z = 0.0
        self.turn.angular.x = 0.0
        self.turn.angular.y = 0.0
        self.turn.angular.z = angular_speed

        goal_angle = 3.1415926/8
        angular_duration = goal_angle/angular_speed
        #发布频率
        rate = 50
        r = rospy.Rate(rate)
        ticks = int(goal_angle*rate)+5
        for i in range(ticks):
            self.cmd_vel.publish(self.turn)
            r.sleep()        
        self.face_reco_client.wait_for_service(10.0)
        sub_res= self.face_reco_client.call(sub_req)
        if sub_res.object[0].state == 0:
            return sub_res.object
        elif sub_res.object[0].state== -3:
            return left_justice(obj)
        elif sub_res.object[0].state == -4:
            return right_justice(obj)
        else:
            raise Exception("xm meet wrong when justice")
    
    def ahead_justice(self):
        
        angular_speed = 0.1
        self.turn = Twist()
        self.turn.linear.x = 0.1
        self.turn.linear.y = 0.0
        self.turn.linear.z = 0.0
        self.turn.angular.x = 0.0
        self.turn.angular.y = 0.0
        self.turn.angular.z = 0.0

        goal_angle = 0.1
        angular_duration = goal_angle/angular_speed
        #发布频率
        rate = 50
        r = rospy.Rate(rate)
        ticks = int(goal_angle*rate)+5
        for i in range(ticks):
            self.cmd_vel.publish(self.turn)
            r.sleep()        
        


    def right_justice(self,obj):
        sub_req = xm_ObjectDetectRequest()
        sub_req.object_name ="person"
        sub_req.people_id = obj.name
        angular_speed = 1.0
        self.turn = Twist()
        self.turn.linear.x = 0.0
        self.turn.linear.y = 0.0
        self.turn.linear.z = 0.0
        self.turn.angular.x = 0.0
        self.turn.angular.y = 0.0
        self.turn.angular.z = angular_speed

        goal_angle = -3.1415926/8
        angular_duration = abs(goal_angle/angular_speed)
        #发布频率
        rate = 50
        r = rospy.Rate(rate)
        ticks = abs(int(goal_angle*rate))+5
        for i in range(ticks):
            self.cmd_vel.publish(self.turn)
            r.sleep() 
        self.face_reco_client.wait_for_service(10.0)
        sub_res= self.face_reco_client.call(sub_req)
        if sub_res.object[0].state == 0:
            return sub_res.object
        elif sub_res.object[0].state== -3:
            return left_justice(obj)
        elif sub_res.object[0].state == -4:
            return right_justice(obj)
        else:
            raise Exception("xm meet wrong when justice")
    def data_deal(self,pos_xm,distance):
        # the simple deal for data from the cv
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        angle = atan2(person_y, person_x)
        person_x = person_x - distance*cos(angle)
        person_y = person_y -distance*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0
        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        # change 
        q_angle = quaternion_from_euler(0,0,angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header  =pos_xm.header
        qs.quaternion = self.q
      
        # self.tf_listener.waitForTransform('base_footprint','camera_link',rospy.Time(),rospy.Duration(60.0))  
        # self.tf_listener.waitForTransform('odom','base_footprint',rospy.Time(),rospy.Duration(60.0))    
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    
        
        
        # pos_xm =self.tf_listener.transformPoint('base_footprint',pos_xm)
        # pos_xm =self.tf_listener.transformPoint('odom',pos_xm)
        pos_xm =self.tf_listener.transformPoint('map',pos_xm)
        rospy.logwarn('tf point succeeded ')    
        
        # the angle should also transform to the map frame
        # qs =self.tf_listener.transformQuaternion('base_link',qs)
    
        # qs =self.tf_listener.transformQuaternion('base_footprint',qs)
        # qs =self.tf_listener.transformQuaternion('odom',qs)
        qs =self.tf_listener.transformQuaternion('map',qs)
        rospy.logwarn('tf quaternion succeeded ')    
        self.ps = Pose(pos_xm.point,qs.quaternion)



class FaceReco3(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error','again','train_error','turn_l','turn_r'],
                        input_keys = ['name_id' , 'distance'],
                        io_keys = ['position'],
                        output_keys = ['num_list'])
        #service name should be specified when used 
        self.face_reco_client = rospy.ServiceProxy('get_position',xm_ObjectDetect)
        self.tf_listener =tf.TransformListener()
        self.ps = Pose()
        self.position =list()
        self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel',Twist,queue_size=1)

    def execute(self,userdata):
        rospy.loginfo('face recongize begin')
        req  = xm_ObjectDetectRequest()
        self.distance = userdata.distance
        try:
            name_id = userdata.name_id
            subprocess.call("xterm -e rosrun xm_vision people_identify.py &",shell=True)
            rospy.sleep(2.0)
        except:
            rospy.logerr('No param specified')
            return 'error'
        req.object_name ="person"
        req.people_id = name_id
        # call the cv service
        try:
            self.face_reco_client.wait_for_service(10.0)
            res= self.face_reco_client.call(req)
            print res

            
                #return 'aborted'
            try:
                pid_str = subprocess.check_output('ps -aux | grep people_identify.py' , shell= True)
                pid_str1 = pid_str.splitlines()[0].split()[1]
                rospy.logwarn(pid_str1)
                subprocess.call('kill -9 '+pid_str1 , shell = True)
            except Exception,e:
                rospy.logerr('No such process ')

        except Exception,e:
            

            rospy.logerr('can not call!!!')
            rospy.logerr(e)
            rospy.sleep(5.0)
            return 'aborted'
        else:
            
            if len(res.object)==0 :
                killPro()
                return 'aborted'
            res.object.sort(key = lambda obj:obj.pos.point.x)
            if res.object[0].name_id == -1:
                rospy.logerr('error detected')
                    
                killPro()
                self.ahead_justice()
                return 'aborted'

            if res.object[0].state ==0:
                self.true_person = list()
                self.error_num_l = list()
                self.error_num_r = list()

                if(name_id == 4  and len(res.object)<3):
                    rospy.logerr('error detected')
                    
                    killPro()
                    self.ahead_justice()
                    return 'aborted'
                

                for i in range(len(res.object)):
                    print res.object[i].pos.point.x
                    if res.object[i].pos.point.x >= -11.0 and res.object[i].pos.point.x <= 11.0 :
                    ######################这个地方处理很有问题目前先这么决定，以后一定要改########################
                        if abs(res.object[i].pos.point.x - 10.0) < 0.00001 and abs(res.object[i].pos.point.y - 10.0 < 0.00001) and abs(res.object[i].pos.point.z - 10.0 < 0.00001):
                            rospy.logerr("right")
                            obj = self.right_justice(res.object[i])
                            self.data_deal(obj.pos ,self.distance)
                            self.true_person.append(obj)
                            # return 'turn_r'
                        elif abs(res.object[i].pos.point.x + 10.0 < 0.00001) and abs(res.object[i].pos.point.y + 10.0 < 0.00001) and abs(res.object[i].pos.point.z + 10.0 < 0.00001):
                            rospy.logerr("left")
                            obj = self.left_justice(res.object[i])
                            self.data_deal(obj.pos,self.distance)
                            self.true_person.append(obj)
                            # return 'turn_l'
                        else:
                            rospy.logerr("ok")
                            rospy.logwarn("i")
                            rospy.logwarn(i)
                            rospy.logwarn(res.object[i].pos)

                            if(name_id == -1 and res.object[i].name == 2):
                                rospy.logerr('name_id = 0?')
                                rospy.logerr(res.object[i].name)
                                self.data_deal(res.object[i].pos,self.distance-0.2)
                            else:
                                self.data_deal(res.object[i].pos ,self.distance)
                                
                            self.true_person.append(res.object[i])
                            self.position.append(self.ps)
                            rospy.logwarn(self.ps) 
                    # self.position.sort(key= lambda Pose: Pose.position.y)
                if len(self.true_person)==0:
                    killPro()
                    return 'aborted'
                if name_id !=4:
                    
                    userdata.position =self.position.pop()
                    rospy.logerr(name_id)
                else:
                    self.out_list = [int(obj.name) for obj in self.true_person]
                    print self.out_list
                    # emmmmmm所以这个操作其实有点孬，用于pop()，详见GetID，翻转一次来源于GetValue
                    # if len(self.out_list)>1:
                        # self.out_list.reverse()
                    helper = self.out_list
                    # else:
                    #     helper = self.out_list
                    userdata.num_list = helper
                    rospy.logerr(helper)
                    userdata.position.extend(self.position)
                
                killPro()
                return 'succeeded'
            elif res.object[0].state ==-1:
                rospy.logwarn("I will recognize again")
                self.ahead_justice()
                killPro()
                return 'again'
            elif res.object[0].state ==-2:
                rospy.logwarn("the train may cause error")
                killPro()
                return 'train_error'
            elif res.object[0].state ==-3:
                rospy.logwarn("the position is not fit,turn left")
                killPro()
                return 'turn_l'
            elif res.object[0].state ==-4:
                rospy.logwarn("the position is not fit, turn right")
                killPro()
                return 'turn_r'
            else :
                killPro()
                return 'aborted'
    def left_justice(self,obj):
        sub_req = xm_ObjectDetectRequest()
        sub_req.object_name ="person"
        sub_req.people_id = obj.name
        angular_speed = 1.0
        self.turn = Twist()
        self.turn.linear.x = 0.0
        self.turn.linear.y = 0.0
        self.turn.linear.z = 0.0
        self.turn.angular.x = 0.0
        self.turn.angular.y = 0.0
        self.turn.angular.z = angular_speed

        goal_angle = 3.1415926/8
        angular_duration = goal_angle/angular_speed
        #发布频率
        rate = 50
        r = rospy.Rate(rate)
        ticks = int(goal_angle*rate)+5
        for i in range(ticks):
            self.cmd_vel.publish(self.turn)
            r.sleep()        
        self.face_reco_client.wait_for_service(10.0)
        sub_res= self.face_reco_client.call(sub_req)
        if sub_res.object[0].state == 0:
            return sub_res.object
        elif sub_res.object[0].state== -3:
            return left_justice(obj)
        elif sub_res.object[0].state == -4:
            return right_justice(obj)
        else:
            raise Exception("xm meet wrong when justice")
    
    def ahead_justice(self):
        
        angular_speed = 0.1
        self.turn = Twist()
        self.turn.linear.x = 0.1
        self.turn.linear.y = 0.0
        self.turn.linear.z = 0.0
        self.turn.angular.x = 0.0
        self.turn.angular.y = 0.0
        self.turn.angular.z = 0.0

        goal_angle = 0.1
        angular_duration = goal_angle/angular_speed
        #发布频率
        rate = 50
        r = rospy.Rate(rate)
        ticks = int(goal_angle*rate)+5
        for i in range(ticks):
            self.cmd_vel.publish(self.turn)
            r.sleep()        
        


    def right_justice(self,obj):
        sub_req = xm_ObjectDetectRequest()
        sub_req.object_name ="person"
        sub_req.people_id = obj.name
        angular_speed = 1.0
        self.turn = Twist()
        self.turn.linear.x = 0.0
        self.turn.linear.y = 0.0
        self.turn.linear.z = 0.0
        self.turn.angular.x = 0.0
        self.turn.angular.y = 0.0
        self.turn.angular.z = angular_speed

        goal_angle = -3.1415926/8
        angular_duration = abs(goal_angle/angular_speed)
        #发布频率
        rate = 50
        r = rospy.Rate(rate)
        ticks = abs(int(goal_angle*rate))+5
        for i in range(ticks):
            self.cmd_vel.publish(self.turn)
            r.sleep() 
        self.face_reco_client.wait_for_service(10.0)
        sub_res= self.face_reco_client.call(sub_req)
        if sub_res.object[0].state == 0:
            return sub_res.object
        elif sub_res.object[0].state== -3:
            return left_justice(obj)
        elif sub_res.object[0].state == -4:
            return right_justice(obj)
        else:
            raise Exception("xm meet wrong when justice")
    def data_deal(self,pos_xm,distance):
        # the simple deal for data from the cv
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        angle = atan2(person_y, person_x)
        person_x = person_x - distance*cos(angle)
        person_y = person_y -distance*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0
        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        # change 
        q_angle = quaternion_from_euler(0,0,angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header  =pos_xm.header
        qs.quaternion = self.q
      
        # self.tf_listener.waitForTransform('base_footprint','camera_link',rospy.Time(),rospy.Duration(60.0))  
        # self.tf_listener.waitForTransform('odom','base_footprint',rospy.Time(),rospy.Duration(60.0))    
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    
        
        
        # pos_xm =self.tf_listener.transformPoint('base_footprint',pos_xm)
        # pos_xm =self.tf_listener.transformPoint('odom',pos_xm)
        pos_xm =self.tf_listener.transformPoint('map',pos_xm)
        rospy.logwarn('tf point succeeded ')    
        
        # the angle should also transform to the map frame
        # qs =self.tf_listener.transformQuaternion('base_link',qs)
    
        # qs =self.tf_listener.transformQuaternion('base_footprint',qs)
        # qs =self.tf_listener.transformQuaternion('odom',qs)
        qs =self.tf_listener.transformQuaternion('map',qs)
        rospy.logwarn('tf quaternion succeeded ')    
        self.ps = Pose(pos_xm.point,qs.quaternion)

        
# state return the name and the thing heard , use for remember face and the information
# serivice name 'name_get'
# serivice type xm_Speech_meaning
# mode ==false mean that is the 2nd time reco

class NameAndThing(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        output_keys =['name','target'])

        self.client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
    def execute(self,userdata):
        try:
            a=1
        except:
            rospy.logerr('ros is error, please buy a new computer')
            return 'error'
        # name and target recognize
        try:
            self.client.wait_for_service(timeout=10)
        except:
            rospy.logerr('xm_speech_meaning service is error')
            return 'aborted'
        else :
            #example :  I am Tom and I want ice tea.
            res = self.client.call(command=3)
            rospy.logwarn(res)
            # the name from the speech_node is the form of list
            name = res.name.pop()
            rospy.logerr(name)
            target = res.object.pop()
            userdata.name = name
            userdata.target = target
            self.string_ ='I know that you are '+str(name)+'and you want '+str(target)
            print self.string_
            self.speak_client.wait_for_service(timeout=10.0)
            # self.speak_client.call(self.string_)
            # rospy.sleep(2.0)

            speech_bool = self.speak_client.call(self.string_)
            if speech_bool.flag == 1:
                subprocess.call(["play","tts_sample.wav"])
            elif speech_bool.flag == 0:
                subprocess.call("espeak -vf5 -s 100 '%(a)s'"%{'a':str(self.string_)} , shell = True)
            else:
                rospy.logerr('the response error')
                return 'error'
            return 'succeeded'

# simple state check the people_position if empty
class CheckEmpty(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        input_keys =['list'])
    
    def execute(self,userdata):
        try:
            length = len(userdata.list)
        except:
            rospy.logerr('No param specified')
            return 'error'
        else:
            if length ==0 :
                return 'aborted'
            else:
                return 'succeeded'

# this state is used for generating the sentences that shows xm understands the speaker
class CheckName(State):
    def __init__(self):
        State.__init__(self,outcomes = ['succeeded','aborted','error'],
                        input_keys =['name_list','name_id','name'])
    def execute(self, userdata):
        try:
            getattr(userdata,'name_list')
            getattr(userdata,'name_id')
            getattr(userdata,'name')
        except:
            rospy.logerr('No params specified')
            return 'error'
        if userdata.name ==userdata.name_list[userdata.name_id]:
            return 'succeeded'
        else:
            return 'aborted'

# this state used for extract the position of each person
# get the last element of the list
class GetValue(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error'],
                        input_keys =['element_list','indice'],
                        output_keys =['element'])
        self.element_list = list()
        self.mode = True
    def execute(self, userdata):
        try:
            getattr(userdata,'element_list')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            rospy.logwarn(userdata.element_list)
            if self.mode ==True:
                self.element_list = userdata.element_list
                self.mode = not self.mode
            try:
                userdata.element = self.element_list.pop()
            except:
                rospy.logerr('pop from empty list')
                return 'aborted'
            return 'succeeded'

class GetValue2(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error'],
                        input_keys =['element_list1' , 'indice' , 'element_list2'],
                        output_keys =['element1' , 'element2'])
        self.element_list = list()
        self.mode = True
    def execute(self, userdata):
        try:
            getattr(userdata,'element_list1')
            getattr(userdata, 'element_list2')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            indice = userdata.indice
            rospy.logwarn(userdata.element_list1)
            rospy.logwarn(userdata.element_list2)
            rospy.logwarn(userdata.indice)
            # if self.mode ==True:
            #     self.element_list = userdata.element_list
            #     self.mode = not self.mode
            # try:
            #     userdata.element = self.element_list.pop()
            if(indice >= len(userdata.element_list1)):
                return 'aborted'

            userdata.element1 = userdata.element_list1[indice] 
            userdata.element2 = userdata.element_list2[indice]
            # except:
            #     rospy.logerr('pop from empty list')
            #     return 'aborted'
            return 'succeeded'


class GetValue3(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error'],
                        input_keys =['element_list1' , 'indice' , 'element_list2','element_list3'],
                        output_keys =['element1' , 'element2' , 'element3'])
        self.element_list = list()
        self.mode = True
    def execute(self, userdata):
        try:
            getattr(userdata,'element_list1')
            getattr(userdata, 'element_list2')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            indice = userdata.indice
            rospy.logwarn(userdata.element_list1)
            rospy.logwarn(userdata.element_list2)
            rospy.logwarn(userdata.element_list3)
            rospy.logwarn(userdata.indice)
            # if self.mode ==True:
            #     self.element_list = userdata.element_list
            #     self.mode = not self.mode
            # try:
            #     userdata.element = self.element_list.pop()
            if(indice >= len(userdata.element_list3)):
                return 'aborted'

            userdata.element1 = userdata.element_list1[indice] 
            userdata.element2 = userdata.element_list2[indice]
            userdata.element3 = userdata.element_list3[indice]
            # except:
            #     rospy.logerr('pop from empty list')
            #     return 'aborted'
            return 'succeeded'


# this state used for joining the name and target in a list
# insert in the head of the list
class NameInList(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        input_keys=['name','target'],
                        io_keys=['name_list','target_list'])
        self.name_list = list()
        self.target_list = list()
    def execute(self, userdata):
        try:
            getattr(userdata,'name')
            getattr(userdata,'target')
            self.name_list = userdata.name_list
            self.target_list = userdata.target_list
        except Exception,e:
            rospy.logerr('No params specified')
            rospy.logerr(e)
            return 'error'
        else:
            # first insert
            # 2 list with the same order
            self.name_list.append(userdata.name)            
            self.target_list.append(userdata.target)
            userdata.name_list =self.name_list
            userdata.target_list =self.target_list
            print self.target_list
            print self.name_list
            return 'succeeded'
        # oh hehe 
        if False:
            return 'aborted'

# this state may look quitly silly , so if you find good way please rewrite 
# get the last of the list
class GetId(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        input_keys =['input_list','num_list'],
                        output_keys =['output_id','num_list'])
        self.input_list = list()
        self.mode = True
    
    def execute(self,userdata):
        try:
            getattr(userdata,'input_list')
            getattr(userdata,'num_list')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            print userdata.input_list
            print userdata.num_list
            if self.mode ==True:
                self.mode =False
                self.input_list = deepcopy(userdata.input_list)
            else:
                pass
            try:
                num = userdata.num_list.pop()
                print num
                userdata.output_id = int(num)
                return 'succeeded'
            except :
                rospy.logerr('pop from empty list')
                return 'aborted'
                

class GetId2(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        input_keys =['num_list'],
                        output_keys =['output_id','num_list'])
        self.input_list = list()
        self.mode = True
    
    def execute(self,userdata):
        try:
            #getattr(userdata,'input_list')
            getattr(userdata,'num_list')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            #print userdata.input_list
            # print userdata.num_list
            # if self.mode ==True:
            #     self.mode =False
            #     self.input_list = deepcopy(userdata.input_list)
            # else:
            #     pass
            try:
                
                num = userdata.num_list.pop()

                while(num == -1):
                    num = userdata.num_list.pop()
                
                print num
                userdata.output_id = int(num)
                return 'succeeded'
            except :
                rospy.logerr('pop from empty list')
                return 'aborted'

# this state is used for rosrun the xm_speech node 
# donnot ask me why the smach code need to run the node 
# I donnot want to say -_-
# class RunNode(State):
#     def __init__(self):
#         State.__init__(self,outcomes =['succeeded','aborted'])
    
#     def execute(self,userdata):
#         try:
#             subprocess.call("xterm -e rosrun xm_speech xm_speech_client_demo.py &", shell = True)
#         except:
#             return 'aborted'
#         else:
#             return 'succeeded'

# get the name

class NameHehe(State):
    def __init__(self):
        State.__init__(self,outcomes =["succeeded",'aborted','error'],
                            input_keys =['name_id','target_name','target_list'],
                            output_keys =['sentences'])
    
    def execute(self,userdata):
        try:
            getattr(userdata,'name_id')
            getattr(userdata,'name_list')
            getattr(userdata,'target_list')
            
        except:
            rospy.logerr("No params specified")
            return 'error'
        else:
            print userdata.name_list
            print userdata.name_id
            sentences = 'your name is '+userdata.name_list.pop() +' and' +'you want ' +userdata.target_list.pop()
            userdata.sentences = sentences
            print sentences
            return 'succeeded'
        if False:
            return 'aborted'

class NameHehe2(State):
    def __init__(self):
        State.__init__(self,outcomes =["succeeded",'aborted','error'],
                            input_keys =['name_id','goal_name','goal_target'],
                            output_keys =['sentences'])
    
    def execute(self,userdata):
        try:
            getattr(userdata,'name_id')
            getattr(userdata,'goal_name')
            getattr(userdata,'goal_target')
            
        except:
            rospy.logerr("No params specified")
            return 'error'
        else:
            
            sentences = 'your name is '+userdata.goal_name +' and' +'you want ' +userdata.goal_target
            userdata.sentences = sentences
            rospy.logwarn(sentences)
            return 'succeeded'
        if False:
            return 'aborted'

# generate the pdf of the object-recognize
class GenePdf(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'])
        self.cv_client = rospy.ServiceProxy('get_object', xm_ObjectDetect)
    
    def execute(self,userdata):
        pass

# this state is used for generating a empty name for order
class EmptyName(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'],
                            output_keys=['target','name'])
    def execute(self,userdata):
        try:
            rospy.sleep(40.0)
        except:
            return 'aborted'
        userdata.target = ''
        userdata.name = ''
        return 'succeeded'

#  clean the costmap
class ClearMap(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            # subprocess.call(["rosservice","call","/move_base/clear_costmaps"])
            subprocess.call('rosservice call /move_base/clear_costmaps "{}" ' , shell = True)
            
        except:
            return 'aborted'
        return 'succeeded'
