#! /usr/bin/env python
#encoding:utf8

import rospy
from smach import State, UserData, StateMachine
from smach_ros import SimpleActionState, ServiceState, MonitorState
from xm_msgs.srv import *
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
from std_msgs.msg import Bool, Header
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
import os


def get_pid(name):
    pid = ''
    with open("/home/xm/vision_pid/{}.txt".format(name)) as f:
        pid = f.read()
    return pid

def FDKiller():
    try:
        pid_str = subprocess.check_output('ps -aux | grep object_detect.py' , shell= True)
        pid_str1 = pid_str.splitlines()[0].split()[1]
        rospy.logwarn(pid_str1)
        subprocess.call('kill '+pid_str1 , shell = True)


    except Exception,e:
        rospy.logerr('No such process ')
###new
class FindObject(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['name'],
                       output_keys=['object_pos' , 'object_map_point','object_state'],
                       io_keys=['objmode'])
        self.xm_findobject = rospy.ServiceProxy(
            '/get_position', xm_ObjectDetect)
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        goal = Point()
        try:
            name =str( userdata.name)
            name.replace(' ' , '' , name.count(' '))

            # rospy.logerr(os.path.exists("/home/domestic/ssd_pytorch/class_name.txt"))
            # if os.path.exists("/home/domestic/ssd_pytorch/class_name.txt"):
            #     os.system("rm -rf /home/domestic/ssd_pytorch/class_name.txt")
            #     os.system("cp -r /home/domestic/ssd_pytorch/object/class_name.txt /home/domestic/ssd_pytorch/")
            # if os.path.exists("/home/domestic/ssd_pytorch/class_num.txt"):
            #     os.system("rm -rf /home/domestic/ssd_pytorch/class_num.txt")
            #     os.system("cp -r /home/domestic/ssd_pytorch/object/class_num.txt /home/domestic/ssd_pytorch/")
            
            subprocess.Popen(
                "xterm -e rosrun xm_vision object_detect.py &", shell=True)
            

            # subprocess.Popen(
            #     "xterm -e rosrun xm_vision object_detect &", shell=True)
        except:
            rospy.logerr('No param specified')
            FDKiller()
            return 'error'

        if name == 'tooth brush':
            userdata.objmode = 0
            rospy.logerr('hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh')
        else:
            userdata.objmode = 1
            rospy.logerr(str(userdata.objmode))

        for i in range(1):
            try:
                self.xm_findobject.wait_for_service(timeout=30.0)
                req = xm_ObjectDetectRequest()
                req.object_name = name
                req.people_id = 0
                rospy.logwarn(name)
                rospy.logwarn(req)
                res = self.xm_findobject.call(req)
                
                rospy.logerr("oooooooo")
                if len(res.object) != 0:
                    break
            except Exception, e:
                rospy.logerr(e)
                FDKiller()
                return 'aborted'

        FDKiller()


        if i >= 1 or len(res.object) <= 0:
            rospy.logerr('result wrong')
            return 'aborted'
        if res.object[0].pos.point.x == -10.000 or res.object[0].pos.point.x == 10.000 or res.object[0].pos.point.x == 5:
            rospy.logerr('find nothing')
            return 'aborted'


        rospy.logwarn(res.object[0])
        try:
            pid_str = subprocess.check_output('ps -aux | grep object_detect.py' , shell= True)
            pid_str1 = pid_str.splitlines()[0].split()[1]
            rospy.logwarn(pid_str1)
            subprocess.call('kill '+pid_str1 , shell = True)


        except Exception,e:
            rospy.logerr('No such process ')
        

        object_pos = PointStamped()
        object_pos.point.x = res.object[0].pos.point.z - 0.11
        object_pos.point.y = res.object[0].pos.point.x - 0.125
        object_pos.point.z = 0.917-res.object[0].pos.point.y
        object_pos.header.frame_id = 'base_link'

        userdata.object_pos = object_pos
        userdata.object_state = res.object[0].state

    
        ##new  add object_map_point userdata
        rospy.logwarn('cam_point')
        rospy.logwarn(res.object[0].pos)
        cam_point = res.object[0].pos
        self.tf_listener.waitForTransform('map' , 'base_link' , rospy.Time() , rospy.Duration(0.1) )
        
        map_point = self.tf_listener.transformPoint('map' , object_pos)
        rospy.logwarn('map_point')
        rospy.logwarn(map_point)

        userdata.object_map_point = map_point


        

        return 'succeeded'

####new
class ObjectPointTrans(State):
    def __init__(self):
        State.__init__(self, outcomes = ['succeeded' , 'aborted'] ,
                        input_keys = ['object_map_point'],
                        output_keys = ['object_base_point'])
        
        self.tf_listener = TransformListener()

    def execute(self , userdata):
        
        try:
            getattr(userdata  , 'object_map_point')
        
        except Exception , e:
            rospy.logerr('no cam_pos remapped')
            return 'aborted'

        try:
            map_point = userdata.object_map_point
            rospy.logwarn(map_point)

            self.waitForTransform('base_link' , 'map' , rospy.Time(), rospy.Duration(0.1) )
            base_point = transformPoint('base_link' , map_point)
            rospy.logwarn(base_point)

            userdata.object_base_point = base_point

        except Exception,e:
            rospy.logerr(e)
            return 'aborted'


class FindPeople2(State):
    def __init__(self):
        State.__init__(self , outcomes=['valid' ,'invalid' ,'preempted'] ,
                            output_keys=['pos_xm'])
        self.tf_listener = tf.TransformListener()
        self.find_peo = rospy.ServiceProxy('get_position' , xm_ObjectDetect)
    def execute(self , userdata):
        
        try:
            # if os.path.exists("/home/domestic/ssd_pytorch/class_name.txt"):
            #     os.system("rm -rf /home/domestic/ssd_pytorch/class_name.txt")
            #     os.system("cp -r /home/domestic/ssd_pytorch/people/class_name.txt /home/domestic/ssd_pytorch/")
            # if os.path.exists("/home/domestic/ssd_pytorch/class_num.txt"):
            #     os.system("rm -rf /home/domestic/ssd_pytorch/class_num.txt")
            #     os.system("cp -r /home/domestic/ssd_pytorch/people/class_num.txt /home/domestic/ssd_pytorch/")
            
            subprocess.Popen(
                    "xterm -e rosrun xm_vision object_detect.py &", shell=True)


            for i in range(1):
                self.find_peo.wait_for_service(10)
                self.req = xm_ObjectDetectRequest()
                self.req.object_name = 'person'
                self.req.people_id = 3
                res = self.find_peo.call(self.req)

                if(len(res.object) > 0):
                    break
            
            try:
                pid_str = subprocess.check_output('ps -aux | grep object_detect.py' , shell= True)
                pid_str1 = pid_str.splitlines()[0].split()[1]
                rospy.logwarn(pid_str1)
                subprocess.call('kill -9 '+pid_str1 , shell = True)


            except Exception,e:
                rospy.logerr('No such process ')
                return 'preempted'

            if i >= 1 or len(res.object) <= 0:
                rospy.logerr('find No people')
                return 'valid'

            if res.object[0].pos.point.x == -10.000 or res.object[0].pos.point.y == -10.000 or res.object[0].pos.point.z == -10.000:
                rospy.logerr('find too far')
                return 'valid'

            self.tmp_pos =  res.object[0].pos

            rospy.logwarn(self.tmp_pos)
            if self.tmp_pos.point.x == -10 or self.tmp_pos.point.y == -5 or self.tmp_pos.point.z == 10:
                rospy.logwarn('no people')
                return 'valid'

            elif self.get_distance(self.tmp_pos) >= 0.5:
                rospy.loginfo('i will move')
                ps = self.data_deal(self.tmp_pos)
                userdata.pos_xm = ps
                #return 'succeeded'
            elif self.get_distance(self.tmp_pos) < 0.5:
                rospy.loginfo('i will not move')
                ps = self.data_deal_turn(self.tmp_pos)
                userdata.pos_xm = ps
                #return 'succeeded'

            return 'invalid'
            
        except Exception,e:
            FDKiller()
            rospy.logerr(e)
            return 'preempted'

        


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


        

class FindPeople(State):
    def __init__(self):
        State.__init__(self , outcomes=['valid' ,'invalid' ,'preempted'] ,
                            output_keys=['pos_xm'])
        self.tf_listener = tf.TransformListener()
        self.find_peo = rospy.ServiceProxy('get_position' , xm_ObjectDetect)
    def execute(self , userdata):
        
        try:
            # if os.path.exists("/home/domestic/ssd_pytorch/class_name.txt"):
            #     os.system("rm -rf /home/domestic/ssd_pytorch/class_name.txt")
            #     os.system("cp -r /home/domestic/ssd_pytorch/people/class_name.txt /home/domestic/ssd_pytorch/")
            # if os.path.exists("/home/domestic/ssd_pytorch/class_num.txt"):
            #     os.system("rm -rf /home/domestic/ssd_pytorch/class_num.txt")
            #     os.system("cp -r /home/domestic/ssd_pytorch/people/class_num.txt /home/domestic/ssd_pytorch/")
            
            subprocess.Popen(
                    "xterm -e rosrun xm_vision object_detect.py &", shell=True)


            for i in range(1):
                self.find_peo.wait_for_service(10)
                self.req = xm_ObjectDetectRequest()
                self.req.object_name = 'person'
                self.req.people_id = 1
                res = self.find_peo.call(self.req)

                if(len(res.object) > 0):
                    break
            
            try:
                pid_str = subprocess.check_output('ps -aux | grep object_detect.py' , shell= True)
                pid_str1 = pid_str.splitlines()[0].split()[1]
                rospy.logwarn(pid_str1)
                subprocess.call('kill -9 '+pid_str1 , shell = True)


            except Exception,e:
                rospy.logerr('No such process ')
                return 'preempted'

            if i >= 1 or len(res.object) <= 0:
                rospy.logerr('find No people')
                return 'valid'

            if res.object[0].pos.point.x == -10.000 or res.object[0].pos.point.y == -10.000 or res.object[0].pos.point.z == -10.000:
                rospy.logerr('find too far')
                return 'valid'

            self.tmp_pos =  res.object[0].pos

            rospy.logwarn(self.tmp_pos)
            if self.tmp_pos.point.x == -10 or self.tmp_pos.point.y == -5 or self.tmp_pos.point.z == 10:
                rospy.logwarn('no people')
                return 'valid'

            elif self.get_distance(self.tmp_pos) >= 0.5:
                rospy.loginfo('i will move')
                ps = self.data_deal(self.tmp_pos)
                userdata.pos_xm = ps
                #return 'succeeded'
            elif self.get_distance(self.tmp_pos) < 0.5:
                rospy.loginfo('i will not move')
                ps = self.data_deal_turn(self.tmp_pos)
                userdata.pos_xm = ps
                #return 'succeeded'

            return 'invalid'
            
        except Exception,e:
            FDKiller()
            rospy.logerr(e)
            return 'preempted'

        


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



class FindTwoPeople(State):
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
                res = self.find_peo.call(self.req)

                if(len(res.object) > 0):
                    break
            
            try:
                pid = get_pid("people_tracking")
                subprocess.Popen(['kill','-9',pid],shell=False)
                # with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                #     f.write('')
                print("sleepiiiiiiiiing!!!!!!!!!!!")

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
            
        except Exception,e:
            FDKiller()
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

# class FindPeople():
    
#     def __init__(self):
        
# class FindPeople():
#     def __init__(self):
#         self.find_people_ = MonitorState('follow',
#                                         xm_FollowPerson,
#                                         self.people_cb,
#                                         max_checks =5,
#                                         output_keys=['pos_xm'])

#         self.tf_listener = tf.TransformListener()

# # 如果相机找到人,这个状态将会返回False
# # 相反,如果在五个循环里相机都没找到人,将会返回True
# # msg传入主题的数据
#     def people_cb(self,userdata,msg):
#         if msg is not None:
#             try:
#                 self.tmp_pos = msg.position
#                 rospy.logwarn(self.tmp_pos)
#                 if self.tmp_pos.point.x==-10 and tmp_pos.point.y == -10 and tmp_pos.point.z == 10:
#                     rospy.logwarn('no people')
#                     pass
#             #如果得到人的坐标信息返回移动的位置
#                 elif self.get_distance(self.tmp_pos) >= 0.5 and self.get_distance(self.tmp_pos)<=2.0 :
#                     rospy.loginfo('i will move')
#                     ps =self.data_deal(self.tmp_pos)
#                     userdata.pos_xm = ps
#                     return False
#                 elif self.get_distance(self.tmp_pos) < 0.5:
#                     rospy.loginfo('i will not move')
#                     ps = self.data_deal_turn(self.tmp_pos)
#                     userdata.pos_xm = ps
#                     return False
#                 else:
#                     rospy.logerr('the person is out of the range')
#                     return True
#             except:
#                 rospy.logerr(e)
#                 return True
            
#         else:
#             raise Exception('MsgNotFind')
#     def get_distance(self,pos_xm):
#         person_x = pos_xm.point.z
#         person_y = pos_xm.point.x

#         return  hypot(person_x,person_y)
#     def data_deal_turn(self,pos_xm):
#         #图像到导航的坐标转换
#         person_x = pos_xm.point.z
#         person_y = pos_xm.point.x
#         #计算人和xm连线与视线正前方夹角
#         angle = atan2(person_y,person_x)
#         #初始化xm现在的位置用于之后得到base_link在全局坐标系中的位置
#         pos_xm.point.x = 0
#         pos_xm.point.y = 0
#         pos_xm.point.z = 0
#         new_header = Header()
#         new_header.frame_id = 'base_link'
#         pos_xm.header = new_header
#         #从角度得到四元数
#         q_angle = quaternion_from_euler(0, 0, angle)
#         self.q = Quaternion(*q_angle)
#         qs = QuaternionStamped()
#         qs.header = pos_xm.header
#         qs.quaternion = self.q
#         rospy.logwarn(qs)
#         #等待tf的信息
#         self.tf_listener.waitForTransform('map', 'base_link',rospy.Time(), rospy.Duration(60.0))
#         rospy.logwarn('get the tf message')
#         #利用tf信息转化坐标
#         pos_xm = self.tf_listener.transformPoint('map',pos_xm)

#         rospy.logwarn('tf point succeeded ')
#         #qs是一个四元数
#         qs =self.tf_listener.transformQuaternion('map',qs)

#         rospy.logwarn('tf quaternion succeeded ')

#         ps = Pose(pos_xm.point,qs.quaternion)
#         return ps
# #返回xm经过处理后的Pose()
#     def data_deal(self,pos_xm):
#         # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
#         # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
#         person_x = pos_xm.point.z
#         person_y = pos_xm.point.x
        
#         #计算方位角
#         angle = atan2(person_y, person_x)
#         #这里是为了到人的面前进行问题回答
# #        person_x = person_x - (hypot(person_x,person_y)-0.2)*cos(angle)
# #        person_y = person_y - (hypot(person_x,person_y)-0.2)*sin(angle)
#         person_x = person_x - 0.65*cos(angle)
#         person_y = person_y - 0.65*sin(angle)
#         pos_xm.point.x = person_x
#         pos_xm.point.y =person_y
#         pos_xm.point.z =0

#         # init the stamped of the Header
#         # 初始化Header
#         new_header =Header()
#         new_header.frame_id = 'base_link'
#         pos_xm.header = new_header

#         #一个四元数描述了旋转轴和旋转角度
#         #绕z轴旋转angle角度
#         #这个函数从欧拉旋转（绕x轴旋转角，绕y轴旋转角，绕z轴旋转角）变换到四元数表示旋转
#         #对给定的旋转轴(a,b,c)和一个角度theta对应四元数
#         #q = (a*sin(theta/2), b*sin(theta/2), c*sin(theta/2), cos(theta))
#         q_angle = quaternion_from_euler(0, 0, angle)
#         self.q = Quaternion(*q_angle)
#         rospy.loginfo(self.q)
#         qs = QuaternionStamped()
#         qs.header = pos_xm.header
#         qs.quaternion = self.q
#         self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
#         rospy.logwarn('wait for tf succeeded ')    

#         #pos_xm是一个Point()
#         pos_xm = self.tf_listener.transformPoint('map',pos_xm)

#         rospy.logwarn('tf point succeeded ')    

#         #qs是一个四元数
#         qs =self.tf_listener.transformQuaternion('map',qs)

#         rospy.logwarn('tf quaternion succeeded ')

#         ps = Pose(pos_xm.point,qs.quaternion)
#         return ps


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



