#!/usr/bin/env python
# encoding:utf8
# this module only contants the simple states usually used, the smach_ros we will directly use in the scripts
# userdata of smach-container includes the whole userdata interface ,you can achieve the different interface by try-except function
# no param specified error should be raise by state itself so it is easy for us to debug
import rospy
from smach import State,UserData,StateMachine
from smach_ros import SimpleActionState, ServiceState, MonitorState
from xm_msgs.srv import *
from speech.srv import *
from pydub import AudioSegment#音频预处理
from pydub.playback import play#用于播放音频
#import xm_speech.srv
from xm_msgs.msg import *
from geometry_msgs.msg import *
from time import sleep
from math import *
import tf 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import subprocess
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from std_msgs.msg import Bool,Header
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import os

def get_pid(name):
    pid = ''
    with open("/home/xm/vision_pid/{}.txt".format(name)) as f:
        pid = f.read()
    return pid

class CheckStop(State):
    def __init__(self):
        State.__init__(self,
                        outcomes = ['remeber','stop','aborted'],
                        input_keys = ['PT_LIST','mission'],
                        output_keys= ['PT_LIST','mission'])
    
        self.target_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        self.tf_listener = tf.TransformListener()
    def execute(self,userdata):
        try:
            while not rospy.is_shutdown():
                self.target_client.wait_for_service(timeout = 10)
                self.response = self.target_client.call(command = 6)
                self.action = self.response.action
                self.object = self.response.object
                self.target = self.response.target
                rospy.logwarn(self.action)
                if len(self.object) == 1:
                    # subprocess.call("touch /home/ye/Recognition/kinect2/dummy_excution_final &" , shell = True)

                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')

                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
   
                    
                    userdata.PT_LIST[str(self.object[0])] = Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    return 'remeber'
                elif len(self.action)>0 and self.action[0] == 'stop' and len(userdata.PT_LIST) >= 3:
                    rospy.logwarn('I will stop!!')
                    rospy.logwarn('go shopping!!!!!!!!!!')
                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')
                    self.mission = {}
                    for obj in self.object:
                        self.mission[obj] = userdata.PT_LIST[obj]
                    
                    userdata.mission = self.mission
                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
                    rospy.logerr(userdata.mission)
                    rospy.logerr(userdata.PT_LIST)
                    userdata.PT_LIST['cashier'] =  Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    return 'stop'
                elif len(self.object) == 3:
                    rospy.logwarn('go shopping!!!!!!!!!!')
                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')
                    self.mission = {}
                    for obj in self.object:
                        self.mission[obj] = userdata.PT_LIST[obj]
                    userdata.mission = self.mission
                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
                    rospy.logerr(userdata.mission)
                    rospy.logerr(userdata.PT_LIST)
                    userdata.mission['cashier'] =  Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    return 'stop'
        except Exception,e:
            rospy.logerr('xm meet wrong when get the target')
            rospy.logerr(e)
            return 'aborted'

class CheckStop2(State):
    def __init__(self):
        State.__init__(self,
                        outcomes = ['remeber','stop','aborted'],
                        input_keys = ['PT_LIST','mission'],
                        output_keys= ['PT_LIST','mission'])
    
        # self.target_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        self.tf_listener = tf.TransformListener()
        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)

    def execute(self,userdata):
        try:
            while not rospy.is_shutdown():
                self.speak_client.wait_for_service(timeout = 10)
                self.response = self.speak_client.call(command = 5)
                self.action = self.response.action
                self.object = self.response.object
                self.target = self.response.gesture
                rospy.logwarn(self.action)
                rospy.logerr(self.object)
                rospy.logwarn("len"+str(len(userdata.PT_LIST)))
                if len(self.object) == 1:
                    # subprocess.call("touch /home/ye/Recognition/kinect2/dummy_excution_final &" , shell = True)

                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')

                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))

                    if len(self.target)!= 0:
                        rospy.logerr("target")
                        rospy.logerr(self.target)
                        eulerVec = euler_from_quaternion(rot)
                        eulerVec2 = eulerVec[2]
                        if self.target[0] == 'left':
                            
                            eulerVec2 = eulerVec[2]+pi/2
                            
                        elif self.target[0] == 'right':
                            eulerVec2 = eulerVec[2]-pi/2

                        rot = quaternion_from_euler(eulerVec[0] , eulerVec[1] , eulerVec2)
                            

                    
                    userdata.PT_LIST[str(self.object[0])] = Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))

                    if len(self.target)!= 0:
                        self.speak_sentence = 'Here is the ' + self.object[0] + ' and it is in your ' + self.target
                    else:
                        self.speak_sentence = 'Here is the ' + self.object[0]

                    self.speak_client.wait_for_service(timeout=10.0)
                    self.speak_client.call(2, self.speak_sentence)
                    return 'remeber'
                    
                elif len(self.action)>0 and self.action[0] == 'stop' and len(userdata.PT_LIST)>=4:
                    rospy.logwarn('I will stop!!')
                    rospy.logwarn('go shopping!!!!!!!!!!')
                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')
                    self.mission = {}
                    for obj in self.object:
                        self.mission[obj] = userdata.PT_LIST[obj]
                    userdata.mission = self.mission
                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
                    
                    userdata.PT_LIST['cashier'] =  Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    rospy.logerr(userdata.PT_LIST)

                    pid = get_pid("people_tracking")
                    subprocess.Popen(['kill','-9',pid],shell=False)
                    with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                        f.write('')
                    print("sleepiiiiiiiiing!!!!!!!!!!!")

                    return 'stop'
                elif len(self.object) == 4:
                    rospy.logwarn('go shopping!!!!!!!!!!')
                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')
                    self.mission = {}
                    for obj in self.object:
                        self.mission[obj] = userdata.PT_LIST[obj]
                    userdata.mission = self.mission
                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
                    
                    userdata.PT_LIST['cashier'] =  Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    rospy.logerr(userdata.PT_LIST)
                    return 'stop'
                # else:
                #     rospy.logwarn("FUCK :")
                #     return 'remeber'
        except Exception,e:
            rospy.logerr('xm meet wrong when get the target')
            rospy.logerr(e)
            return 'aborted'
        
# 运行跟随人的节点
class RunNode(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_vison_publisher.py','-d','true'] ,shell =False)
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
            # a.wait()
            print a.poll()
            if a.returncode != None:
                a.wait()
                return 'aborted'
        except Exception as e:
            print('exception:',e)
            rospy.logerr('people_tracking node error')
            return 'aborted'
        return 'succeeded'


class GetMission(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'],output_keys = ['things_list'])
        self.client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
    def execute(self,userdata):
        try:
            self.client.wait_for_service(timeout=10)
            self.response = self.client.call(command=6)
            self.object = self.response.object
            userdata.things_list = self.object
            return 'succeeded'
        except Exception,e:
            rospy.logwarn(e)
            return 'aborted'

class GetTarget(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','finish'],
                        output_keys = ['pos_xm','mission_name'],
                        input_keys = ['mission'])
    def execute(self,userdata):
        try:
            self.mission = userdata.mission
            rospy.logwarn(self.mission)
        except Exception,e:
            rospy.logerr('no  specific value')
            rospy.logerr(e)
        try: 
            if len(self.mission) >1:
                key_list = self.mission.keys()
                for i in key_list:
                    if i != 'cashier':
                        thing = self.mission.pop(i)
                        userdata.mission_name = i
                        userdata.pos_xm = thing
                        rospy.logwarn('finish one mission')
                        return 'succeeded'
            elif len(self.mission) == 1:
                thing = self.mission['cashier']
                userdata.pos_xm = thing
                rospy.logwarn('complete the mission')
                return 'finish'
        except Exception,e:
            rospy.logerr(e)
            return 'aborted'

class GetSignal(State):
    def __init__(self):
        State.__init__(self,outcomes = ['succeeded','aborted'])
    
        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)

    def execute(self,userdata):
        try:
            self.speak_client.wait_for_service(timeout = 10)
            self.response = self.speak_client.call(command = 5)
            rospy.logwarn(self.response.action)
            if self.response.action[0] == 'follow':
                return 'succeeded'
            else:
                return 'aborted'
        except Exception,e:
            rospy.logerr('xm meet wrong when get follow signal')
            rospy.logerr(e)
            return 'aborted'

class FindPeople():
    def __init__(self):
        self.find_people_ = MonitorState('follow',
                                        xm_FollowPerson,
                                        self.people_cb,
                                        max_checks =5,
                                        output_keys=['pos_xm'])
        self.tf_listener = tf.TransformListener()

# 如果相机找到人,这个状态将会返回False
# 相反,如果在五个循环里相机都没找到人,将会返回True
# msg传入主题的数据
    def people_cb(self,userdata,msg):
        print msg
        if msg is not None:
            try:
                self.tmp_pos = msg.position
                rospy.logwarn(self.tmp_pos)
                rospy.logwarn('finding people........')
                if self.tmp_pos.point.x==10 and tmp_pos.point.y == 10 and tmp_pos.point.z == 10:
                    rospy.logwarn('no people')
                    pass
            #如果得到人的坐标信息返回移动的位置
                elif self.get_distance(self.tmp_pos)>0.5 and self.get_distance(self.tmp_pos)<=4.0 :
                    rospy.loginfo('i will move')
                    ps =self.data_deal(self.tmp_pos)
                    userdata.pos_xm = ps
                    return False
                elif self.get_distance(self.tmp_pos) < 0.5:
                    rospy.loginfo('i will not move')
                    ps = self.data_deal_turn(self.tmp_pos)
                    userdata.pos_xm = ps
                    return False
                else:
                    rospy.logerr('the person is out of the range')
                    return True
            except:
                rospy.logerr(e)
                return True
            
        else:
            raise Exception('MsgNotFind')
    def get_distance(self,pos_xm):
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x

        return  hypot(person_x,person_y)
    def data_deal_turn(self,pos_xm):
        #图像到导航的坐标转换
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        #计算人和xm连线与视线正前方夹角
        angle = atan2(person_y,person_x)
        #初始化xm现在的位置用于之后得到base_link在全局坐标系中的位置
        person_x = person_x - 1.0*cos(angle)
        person_y = person_y - 1.0*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0
        new_header = Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        #从角度得到四元数
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        rospy.logwarn(qs)
        #等待tf的信息
        self.tf_listener.waitForTransform('map', 'base_link',rospy.Time(), rospy.Duration(60.0))
        rospy.logwarn('get the tf message')
        #利用tf信息转化坐标
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')
        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps
#返回xm经过处理后的Pose()
    def data_deal(self,pos_xm):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        
        #计算方位角
        angle = atan2(person_y, person_x)
        #这里是为了到人的面前进行问题回答
#        person_x = person_x - (hypot(person_x,person_y)-0.2)*cos(angle)
#        person_y = person_y - (hypot(person_x,person_y)-0.2)*sin(angle)
        person_x = person_x - 0.8*cos(angle)
        person_y = person_y - 0.8*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0

        # init the stamped of the Header
        # 初始化Header
        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header

        #一个四元数描述了旋转轴和旋转角度
        #绕z轴旋转angle角度
        #这个函数从欧拉旋转（绕x轴旋转角，绕y轴旋转角，绕z轴旋转角）变换到四元数表示旋转
        #对给定的旋转轴(a,b,c)和一个角度theta对应四元数
        #q = (a*sin(theta/2), b*sin(theta/2), c*sin(theta/2), cos(theta))
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        rospy.loginfo(self.q)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    

        #pos_xm是一个Point()
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')    

        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps

# 延续(苟)一段时间的简单状态,rec为延续(苟)的时间
class Wait(State):
    def __init__(self):
        State.__init__(self, 
        outcomes=["succeeded",'error'],
        input_keys=['rec'])

    def execute(self, userdata):
        try:
            self.rec = userdata.rec
        except:
            rospy.logerr('no param specified')
            return 'error'
        else:  
            rospy.sleep(userdata.rec)
            return "succeeded"

class Wait_trace(State):
    def __init__(self):
        State.__init__(self, 
        outcomes=["succeeded",'error','preempted'])

    def execute(self, userdata):
        for i in range(0,20):
            if self.preempt_requested():
                return 'preempted'
            else:
                rospy.sleep(0.1)
        return "succeeded"

class RunNode_img(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            a = subprocess.Popen('xterm -e rosrun xm_vision image_test &',shell =True)
        except:
            rospy.logerr('people_tracking node error')
            return 'aborted'
        return 'succeeded'

class GetCashierPos(State):
    def __init__(self):
        State.__init__(self,
                        outcomes=['succeeded','aborted'],
                        output_keys=['pose'],
                        input_keys=['PT_LIST','pose'])

    def execute(self ,userdata):
        try:
            getattr(userdata , 'PT_LIST')

        except:
            rospy.logerr('No param')
            return 'aborted'
        
        try:
                userdata.pose = userdata.PT_LIST['cashier']
                rospy.logwarn(userdata.pose)
                return 'succeeded'
        
        except Exception,e:
            rospy.logwarn(e)
            return 'aborted'

class GetValueFir(State):
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
            if self.mode ==True:
                self.element_list = userdata.element_list
                self.mode = not self.mode
            try:
                userdata.element = self.element_list.pop()
            except:
                rospy.logerr('pop from empty list')
                return 'aborted'
            return 'succeeded'

class GetValueSec(State):
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
            if self.mode ==True:
                self.element_list = userdata.element_list
                self.mode = not self.mode
            try:
                userdata.element = self.element_list[1]
            except:
                rospy.logerr('pop from empty list')
                return 'aborted'
            return 'succeeded'

class GetValueThird(State):
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
            if self.mode ==True:
                self.element_list = userdata.element_list
                self.mode = not self.mode
            try:
                userdata.element = self.element_list[0]
            except:
                rospy.logerr('pop from empty list')
                return 'aborted'
            return 'succeeded'

class FindTwoPeople_1(State):
    def __init__(self):
        State.__init__(self , outcomes=['succeeded' ,'aborted' ,'error'] ,
                            output_keys=['people_position','people_position_1'])
        self.tf_listener = tf.TransformListener()
        self.find_peo = rospy.ServiceProxy('find_two_persons' , xm_findTwoP)
    def execute(self , userdata):
        
        try:
            # if os.path.exists("/home/domestic/ssd_pytorch/class_name.txt"):
            #     os.system("rm -rf /home/domestic/ssd_pytorch/class_name.txt")
            #     os.system("cp -r /home/domestic/ssd_pytorch/people/class_name.txt /home/domestic/ssd_pytorch/")
            # if os.path.exists("/home/domestic/ssd_pytorch/class_num.txt"):
            #     os.system("rm -rf /home/domestic/ssd_pytorch/class_num.txt")
            #     os.system("cp -r /home/domestic/ssd_pytorch/people/class_num.txt /home/domestic/ssd_pytorch/")
            
            # subprocess.Popen(
            #        "xterm -e rosrun xm_vision object_detect.py &", shell=True)

            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/Shopping/mrsupw_find_two_persons.py','-d','true'] ,shell =False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w+') as f:
                print('-------------------')
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('-------------------')
                f.write(str(a.pid))


            for i in range(5):
                self.find_peo.wait_for_service(10)
                self.req = xm_findTwoPRequest()
                self.req.object_name = 'people'
                self.req.people_id = 2
                self.req.flag = 0
                res = self.find_peo.call(self.req)
                #rospy.logwarn("#######")

                if(len(res.object) > 0):
                    break
            
            try:
                pid = get_pid("people_tracking")
                subprocess.Popen(['kill','-9',pid],shell=False)
                # with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                #     f.write('')
                print("sleepiiiiiiiiing!!!!!!!!!!!")
                #rospy.sleep(3.0)

            except Exception,e:
                rospy.logerr('No such process ')
                #return 'preempted'

            if(i >= 5 ):
                rospy.logerr('find No people')
                return 'aborted'

            rospy.logerr(res.object)
            if(len(res.object) != 2):
                rospy.logerr('not enough people')
                return 'aborted'
            
            self.people_position = list()
            for i in range(2):
                if res.object[i].pos.point.x == -10.000 or res.object[i].pos.point.y == -10.000 or res.object[0].pos.point.z == -10.000:
                    rospy.logerr('find too far')
                    return 'aborted'

                self.tmp_pos =  res.object[i].pos

                rospy.logwarn(self.tmp_pos)
                if self.tmp_pos.point.x == -10 or self.tmp_pos.point.y == -5 or self.tmp_pos.point.z == 10:
                    rospy.logwarn('no people')
                    return 'aborted'

                elif self.get_distance(self.tmp_pos) >= 0.5:
                    rospy.loginfo('i will move')
                    ps = self.data_deal(self.tmp_pos)
                    self.people_position.append(ps)
                    #return 'succeeded'
                elif self.get_distance(self.tmp_pos) < 0.5:
                    rospy.loginfo('i will not move')
                    ps = self.data_deal_turn(self.tmp_pos)
                    self.people_position.append(ps)
                    #return 'succeeded'

            rospy.logwarn(self.people_position)
            userdata.people_position = self.people_position
            userdata.people_position_1 = self.people_position
            return 'succeeded'
            
        except Exception as e:
            #FDKiller()
            rospy.logerr(e)
            return 'error'

        


    def get_distance(self,pos_xm):
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x

        return  hypot(person_x,person_y)
    def data_deal_turn(self,pos_xm):
        #图像到导航的坐标转换
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        #计算人和xm连线与视线正前方夹角
        angle = atan2(person_y,person_x)
        #初始化xm现在的位置用于之后得到base_link在全局坐标系中的位置
        pos_xm.point.x = 0
        pos_xm.point.y = 0
        pos_xm.point.z = 0
        new_header = Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        #从角度得到四元数
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        rospy.logwarn(qs)
        #等待tf的信息
        self.tf_listener.waitForTransform('map', 'base_link',rospy.Time(), rospy.Duration(60.0))
        rospy.logwarn('get the tf message')
        #利用tf信息转化坐标
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')
        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps
#返回xm经过处理后的Pose()
    def data_deal(self,pos_xm):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
        person_x = pos_xm.point.x - 0.3
        person_y = pos_xm.point.y
        
        #计算方位角
        angle = atan2(person_y, person_x)
        #这里是为了到人的面前进行问题回答
        #person_x = person_x - (hypot(person_x,person_y)-0.2)*cos(angle)
        #person_y = person_y - (hypot(person_x,person_y)-0.2)*sin(angle)
        #person_x = person_x - 0.6*cos(angle)
        #person_y = person_y - 0.6*sin(angle)
        # pos_xm.point.x = person_x
        # pos_xm.point.y =person_y
        # pos_xm.point.z =0

        pos_xm.point.x = person_x - 0.8
        pos_xm.point.y =person_y 
        pos_xm.point.z =0  
        # init the stamped of the Header
        # 初始化Header
        new_header =Header()
        #new_header.frame_id = 'camera_base'
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header

        #一个四元数描述了旋转轴和旋转角度
        #绕z轴旋转angle角度
        #这个函数从欧拉旋转（绕x轴旋转角，绕y轴旋转角，绕z轴旋转角）变换到四元数表示旋转
        #对给定的旋转轴(a,b,c)和一个角度theta对应四元数
        #q = (a*sin(theta/2), b*sin(theta/2), c*sin(theta/2), cos(theta))
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        rospy.loginfo(self.q)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        #self.tf_listener.waitForTransform('map','camera_base',rospy.Time(),rospy.Duration(60.0))    
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    
         #pos_xm是一个Point()
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')    

        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps

class FindTwoPeople_2(State):
    def __init__(self):
        State.__init__(self , outcomes=['succeeded' ,'aborted' ,'error'] ,
                            input_keys=['indice'],
                            output_keys=['people_position','people_position_1'])
        self.tf_listener = tf.TransformListener()
        self.find_peo = rospy.ServiceProxy('find_two_persons' , xm_findTwoP)
    def execute(self , userdata):
        
        try:
            # if os.path.exists("/home/domestic/ssd_pytorch/class_name.txt"):
            #     os.system("rm -rf /home/domestic/ssd_pytorch/class_name.txt")
            #     os.system("cp -r /home/domestic/ssd_pytorch/people/class_name.txt /home/domestic/ssd_pytorch/")
            # if os.path.exists("/home/domestic/ssd_pytorch/class_num.txt"):
            #     os.system("rm -rf /home/domestic/ssd_pytorch/class_num.txt")
            #     os.system("cp -r /home/domestic/ssd_pytorch/people/class_num.txt /home/domestic/ssd_pytorch/")
            
            # subprocess.Popen(
            #        "xterm -e rosrun xm_vision object_detect.py &", shell=True)

            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/Shopping/mrsupw_find_two_persons.py','-d','true'] ,shell =False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w+') as f:
                print('-------------------')
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('pid',a.pid)
                print('-------------------')
                f.write(str(a.pid))


            for i in range(5):
                self.find_peo.wait_for_service(10)
                self.req = xm_findTwoPRequest()
                self.req.object_name = 'people'
                self.req.people_id = 2
                self.req.flag = 1
                res = self.find_peo.call(self.req)

                if(len(res.object) > 0):
                    break
            
            try:
                pid = get_pid("people_tracking")
                subprocess.Popen(['kill','-9',pid],shell=False)
                # with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                #     f.write('')
                print("sleepiiiiiiiiing!!!!!!!!!!!")
                #rospy.sleep(3.0)

            except Exception,e:
                rospy.logerr('No such process ')
                #return 'preempted'

            if(i >= 5 ):
                rospy.logerr('find No people')
                return 'aborted'

            rospy.logerr(res.object)
            if(len(res.object) != 2):
                rospy.logerr('not enough people')
                return 'aborted'
            
            self.people_position = list()
            for i in range(2):
                if res.object[i].pos.point.x == -10.000 or res.object[i].pos.point.y == -10.000 or res.object[0].pos.point.z == -10.000:
                    rospy.logerr('find too far')
                    return 'aborted'

                self.tmp_pos =  res.object[i].pos

                rospy.logwarn(self.tmp_pos)
                if self.tmp_pos.point.x == -10 or self.tmp_pos.point.y == -5 or self.tmp_pos.point.z == 10:
                    rospy.logwarn('no people')
                    return 'aborted'

                elif self.get_distance(self.tmp_pos) >= 0.5:
                    rospy.loginfo('i will move, here is the point before tf: ')
                    rospy.logwarn(self.tmp_pos)
                    ps = self.data_deal(self.tmp_pos)
                    rospy.loginfo('here is the point after tf: ')
                    rospy.logwarn(ps)
                    self.people_position.append(ps)
                    #return 'succeeded'
                elif self.get_distance(self.tmp_pos) < 0.5:
                    rospy.loginfo('i will not move')
                    ps = self.data_deal_turn(self.tmp_pos)
                    self.people_position.append(ps)
                    #return 'succeeded'

            rospy.logwarn('here is the point list:')
            rospy.logwarn(self.people_position)
            userdata.people_position = self.people_position
            userdata.people_position_1 = self.people_position
            return 'succeeded'
            
        except Exception as e:
            #FDKiller()
            rospy.logerr(e)
            return 'error'

        


    def get_distance(self,pos_xm):
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x

        return  hypot(person_x,person_y)
    def data_deal_turn(self,pos_xm):
        #图像到导航的坐标转换
        person_x = pos_xm.point.z
        person_y = pos_xm.point.x
        #计算人和xm连线与视线正前方夹角
        angle = atan2(person_y,person_x)
        #初始化xm现在的位置用于之后得到base_link在全局坐标系中的位置
        pos_xm.point.x = 0
        pos_xm.point.y = 0
        pos_xm.point.z = 0
        new_header = Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        #从角度得到四元数
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        rospy.logwarn(qs)
        #等待tf的信息
        self.tf_listener.waitForTransform('map', 'base_link',rospy.Time(), rospy.Duration(60.0))
        rospy.logwarn('get the tf message')
        #利用tf信息转化坐标
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')
        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps
#返回xm经过处理后的Pose()
    def data_deal(self,pos_xm):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
        person_x = pos_xm.point.x - 0.3
        person_y = pos_xm.point.y
        
        #计算方位角
        angle = atan2(person_y, person_x)
        #这里是为了到人的面前进行问题回答
        #person_x = person_x - (hypot(person_x,person_y)-0.2)*cos(angle)
        #person_y = person_y - (hypot(person_x,person_y)-0.2)*sin(angle)
        #person_x = person_x - 0.6*cos(angle)
        #person_y = person_y - 0.6*sin(angle)
        # pos_xm.point.x = person_x
        # pos_xm.point.y =person_y
        # pos_xm.point.z =0

        pos_xm.point.x = person_x - 0.8
        pos_xm.point.y =person_y 
        pos_xm.point.z =0  
        # init the stamped of the Header
        # 初始化Header
        new_header =Header()
        #new_header.frame_id = 'camera_base'
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header

        #一个四元数描述了旋转轴和旋转角度
        #绕z轴旋转angle角度
        #这个函数从欧拉旋转（绕x轴旋转角，绕y轴旋转角，绕z轴旋转角）变换到四元数表示旋转
        #对给定的旋转轴(a,b,c)和一个角度theta对应四元数
        #q = (a*sin(theta/2), b*sin(theta/2), c*sin(theta/2), cos(theta))
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        # rospy.loginfo(self.q)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        #self.tf_listener.waitForTransform('map','camera_base',rospy.Time(),rospy.Duration(60.0))    
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    
         #pos_xm是一个Point()
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')    

        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps

class GetValue_new(State):
    def __init__(self):
        State.__init__(self, outcomes =['succeeded','aborted','error'],
                        input_keys =['element_list','indice','person_position'],
                        output_keys =['person_position'])
        self.element_list = list()
        self.mode = True
    def execute(self, userdata):
        try:
            getattr(userdata,'element_list')
            getattr(userdata,'indice')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            rospy.logwarn('the element list is : ')
            rospy.logwarn(userdata.element_list)
            if self.mode ==True:
                self.element_list = userdata.element_list
                self.mode = not self.mode
            try:
                rospy.logwarn('the indice is :' + str(userdata.indice))
                if(userdata.indice == 2):
                    rospy.loginfo('indice == 2')
                    userdata.person_position = userdata.element_list[1]
                    rospy.logwarn(userdata.element_list[1])
                    rospy.loginfo('the person position is :')
                    rospy.logerr(userdata.person_position)
                else :
                    rospy.loginfo('indice == 4')
                    userdata.person_position = userdata.element_list[0]
                    rospy.logwarn(userdata.element_list[0])
                    rospy.loginfo('the person position is :')
                    rospy.logerr(userdata.person_position)
            except:
                rospy.logerr('pop from empty list')
                return 'aborted'
            return 'succeeded'
