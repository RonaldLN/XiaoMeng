#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import subprocess
from smach_ros import *
import actionlib
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
import tf
from copy import deepcopy
from std_msgs.msg import *
class FaceReco(State):
    '''
    name_id = -1时，识别三个人\n
    name_id != -1 时，识别特定编号的人\n
    返回正对人的位置列表\n
    outcomes =['succeeded','aborted','error','again','train_error','turn_l','turn_r']\n
    input_keys = ['distance','name_id']\n
    io_keys = ['position']\n
    output_keys = ['num_list']\n
    '''
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error','again','train_error','turn_l','turn_r'],
                        input_keys = ['distance','name_id'],
                        io_keys = ['position'],
                        output_keys = ['num_list'])
        #service name should be specified when used 
        self.face_reco_client = rospy.ServiceProxy('get_position',xm_ObjectDetect)      #图像获取物体信息
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
        req.people_id = userdata.name_id
        rospy.logwarn(req)
        
        #请求图像服务
        try:
            self.face_reco_client.wait_for_service(10.0)
            res= self.face_reco_client.call(req)
            print res
            
            self.killPro()

        except Exception,e:
            rospy.logerr('can not call!!!')
            rospy.logerr(e)
            rospy.sleep(5.0)
            return 'aborted'
        else:
            
            if len(res.object)==0 :
                self.killPro()
                return 'aborted'
            res.object.sort(key = lambda obj:obj.pos.point.y,reverse=True)  #按照位置从右到左排序

            #第一个人的状态正确
            if res.object[0].state ==0:
                self.true_person = list()
                self.error_num_l = list()
                self.error_num_r = list()
                
                #如果是找所有人，人数小于三，后退
                if(name_id == -1  and len(res.object)<3):
                    rospy.logerr('people<3')
                    self.ahead_justice()
                    self.killPro()
                    return 'aborted'
                
                #对视野中所有的人进行处理
                for i in range(len(res.object)):
                    print res.object[i].pos.point.x
                    #第一个人的位置信息合格
                    if res.object[i].pos.point.x >= -11.0 and res.object[i].pos.point.x <= 11.0 :
                    ######################这个地方处理很有问题目前先这么决定，以后一定要改########################
                        #向右
                        if abs(res.object[i].pos.point.x - 10.0) < 0.00001 and abs(res.object[i].pos.point.y - 10.0 < 0.00001) and abs(res.object[i].pos.point.z - 10.0 < 0.00001):
                            #修正obj的位置
                            rospy.logerr("right")
                            obj = self.right_justice(res.object[i])
                            self.data_deal(obj.pos ,self.distance)
                            self.true_person.append(obj)
                        #向左true
                        elif abs(res.object[i].pos.point.x + 10.0 < 0.00001) and abs(res.object[i].pos.point.y + 10.0 < 0.00001) and abs(res.object[i].pos.point.z + 10.0 < 0.00001):
                            rospy.logerr("left")
                            obj = self.left_justice(res.object[i])
                            self.data_deal(obj.pos,self.distance)
                            self.true_person.append(obj)
                        else:
                            rospy.logerr("ok")
                            rospy.logwarn("pos of the person have been found")
                            rospy.logwarn(i)
                            rospy.logwarn(res.object[i].pos)
                            
                            #最后一个人
                            if(name_id == -1 and res.object[i].name == 1):
                                rospy.logerr('name_id = 0?')
                                rospy.logerr(res.object[i].name)
                                #修正第二个人的位置
                                self.data_deal(res.object[i].pos,self.distance)
                            else:
                                #修正第一个和第三个人的位置
                                self.data_deal(res.object[i].pos ,self.distance)

                            #将正确的人的信息插入到 true_person    
                            self.true_person.append(res.object[i])
                            self.position.append(self.ps)
                            rospy.logwarn(self.ps) 

                #人的信息列表为空
                if len(self.true_person)==0:
                    self.killPro()
                    return 'aborted'

                #寻找特定人，弹出人的位置
                if name_id !=-1:  
                    userdata.position =self.position.pop()
                    rospy.logerr(name_id)
                
                #寻找所有人
                else:
                    # out_list: 2 1 0   num_list 2 1 0
                    self.out_list = [int(obj.name) for obj in self.true_person]
                    print self.out_list
                    userdata.num_list = self.out_list
                    rospy.logerr(self.out_list)
                    userdata.position.extend(self.position)
                    rospy.logwarn(userdata.position)
                
                self.killPro()
                return 'succeeded'
            elif res.object[0].state ==-1:
                rospy.logwarn("I will recognize again")
                self.ahead_justice()
                self.killPro()
                return 'again'
            elif res.object[0].state ==-2:
                rospy.logwarn("the train may cause error")
                self.killPro()
                return 'train_error'
            elif res.object[0].state ==-3:
                rospy.logwarn("the position is not fit,turn left")
                self.killPro()
                return 'turn_l'
            elif res.object[0].state ==-4:
                rospy.logwarn("the position is not fit, turn right")
                self.killPro()
                return 'turn_r'
            else :
                self.killPro()
                return 'aborted'
    def left_justice(self,obj):
        '''
        向左边旋转，直到视野中人的信息准确
        '''
        sub_req = xm_ObjectDetectRequest()
        sub_req.object_name ="person"
        sub_req.people_id = obj.name

        self.turn(3.1415926/8)
    
        self.face_reco_client.wait_for_service(10.0)
        sub_res= self.face_reco_client.call(sub_req)
        if sub_res.object[0].state == 0:
            return sub_res.object
        elif sub_res.object[0].state== -3:
            return self.left_justice(obj)
        elif sub_res.object[0].state == -4:
            return self.right_justice(obj)
        else:
            raise Exception("xm meet wrong when justice")
   
    def right_justice(self,obj):
        '''
        向右边旋转，直到视野中人的信息准确
        '''
        sub_req = xm_ObjectDetectRequest()
        sub_req.object_name ="person"
        sub_req.people_id = obj.name
        
        #向右转22.5度    
        self.turn(-3.1415926/8)
        
        self.face_reco_client.wait_for_service(10.0)
        sub_res= self.face_reco_client.call(sub_req)
        if sub_res.object[0].state == 0:
            return sub_res.object
        elif sub_res.object[0].state== -3:
            return self.left_justice(obj)
        elif sub_res.object[0].state == -4:
            return self.right_justice(obj)
        else:
            raise Exception("xm meet wrong when justice")

    def ahead_justice(self):
        '''
        太近了->后退
        '''
        move_len = -0.1
        self.turn = Twist()
        self.turn.linear.x = 0.2 * int(abs(move_len)/move_len)
        self.turn.linear.y = 0.0
        self.turn.linear.z = 0.0
        self.turn.angular.x = 0.0
        self.turn.angular.y = 0.0
        self.turn.angular.z = 0.0

        angular_duration = abs(move_len/0.2)
        # 发布频率
        rate = 50
        r = rospy.Rate(rate)
        
        # 用1s进行旋转
        rospy.logwarn(angular_duration*rate)
        ticks = abs(int(angular_duration*rate))
        for i in range(ticks):
            self.cmd_vel.publish(self.turn)
            rospy.logwarn(i)
            r.sleep()
        rospy.sleep(1.0)
            
    def data_deal(self,pos_xm,distance):
        '''
        获得距离distance距离的位置，修改pos_xm和self.ps
        '''
        # the simple deal for data from the cv
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y
        angle = math.atan2(person_y, person_x)
        person_x = person_x - distance*math.cos(angle)
        person_y = person_y -distance*math.sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0
        new_header =Header()
        new_header.frame_id = 'kinect2_rgb_link'
        pos_xm.header = new_header
        # change 
        q_angle = quaternion_from_euler(0,0,angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header  =pos_xm.header
        qs.quaternion = self.q
      
   
        self.tf_listener.waitForTransform('map','kinect2_rgb_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    
        

        pos_xm =self.tf_listener.transformPoint('map',pos_xm)
        rospy.logwarn('tf point succeeded ')    

        #修改self.ps
        qs =self.tf_listener.transformQuaternion('map',qs)
        rospy.logwarn('tf quaternion succeeded ')    
        self.ps = Pose(pos_xm.point,qs.quaternion)
        self.ps.position.z = 0
        
    def turn(self,goal_angle):
        '''
        旋转
        '''
        angular_speed = 1.0
        self.turn = Twist()
        self.turn.linear.x = 0.0
        self.turn.linear.y = 0.0
        self.turn.linear.z = 0.0
        self.turn.angular.x = 0.0
        self.turn.angular.y = 0.0
        self.turn.angular.z = angular_speed

        angular_duration = goal_angle/angular_speed
        #发布频率
        rate = 50
        r = rospy.Rate(rate)
        ticks = int(goal_angle*rate)+5
        for i in range(ticks):
            self.cmd_vel.publish(self.turn)
            r.sleep()        

    def killPro(self):
        try:
            pid_str = subprocess.check_output('ps -aux | grep people_identify.py' , shell= True)
            pid_str1 = pid_str.splitlines()[0].split()[1]
            rospy.logwarn(pid_str1)
            subprocess.call('kill -9 '+pid_str1 , shell = True)
        except Exception,e:
            rospy.logerr('No such process ')


class GetValue(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error'],
                        input_keys =['element_list'],
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
            #只有在第一次的时候，初始化element_list*
            if self.mode ==True:
                self.element_list = userdata.element_list
                self.mode = not self.mode
            try:
                userdata.element = self.element_list.pop()
            except:
                rospy.logerr('pop from empty list')
                return 'aborted'
            return 'succeeded'

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
            elif speech_bool.flag == 2:
                pass
            else:
                rospy.logerr('the response error')
                return 'error'
            return 'succeeded'

class CheckTurn(State):
    '''
    用于判断循环的一个状态
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','continue','error'],
                        io_keys =['num'])

    def execute(self,userdata):
        try:
            getattr(userdata, 'num')
        except:
            rospy.logerr('no param')
            return 'error'
        else :
            if userdata.num <=3:
                userdata.num +=1
                return 'continue'
            elif userdata.num ==4:
                return 'succeeded'
            else:
                return 'error'

class NameInList(State):
    '''
    将名字和想要的物体信息插入到列表中,顺序进行了翻转
    '''
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

class GetId(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        input_keys =['input_list'],
                        output_keys =['output_id'],
                        io_keys = ['num_list'])
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
            #第一次识别
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
                
class GenerateInformation(State):
    def __init__(self):
        State.__init__(self,outcomes =["succeeded",'aborted','error'],
                            input_keys =['name_id','name_list','target_list'],
                            output_keys =['sentences'])
        self.mode = True
    
    def execute(self,userdata):
        try:
            getattr(userdata,'name_id')
            getattr(userdata,'name_list')
            getattr(userdata,'target_list')
            
        except:
            rospy.logerr("No params specified")
            return 'error'
        else:
            if self.mode:
                userdata.name_list.reverse()
                userdata.target_list.reverse()
                self.mode = False
            print userdata.name_list
            print userdata.name_id
            sentences = 'your name is '+userdata.name_list.pop() +' and' +'you want ' +userdata.target_list.pop()
            userdata.sentences = sentences
            print sentences
            return 'succeeded'
        if False:
            return 'aborted'

class RebootCameraService(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            self.killPro()
        except Exception,e:
            rospy.logerr('No such process ')
        try:
            subprocess.call("xterm -e rosrun xm_vision people_identify.py &",shell=True)
        except Exception,e:
            rospy.logerr('rosrun xm_vision people_identifiy.py failed! ')
            return 'aborted'
        else:
            return 'succeeded'

    def killPro():
        pid_str = subprocess.check_output('ps -aux | grep people_identify.py' , shell= True)
        pid_str1 = pid_str.splitlines()[0].split()[1]
        rospy.logwarn(pid_str1)
        subprocess.call('kill -9 '+pid_str1 , shell = True)
