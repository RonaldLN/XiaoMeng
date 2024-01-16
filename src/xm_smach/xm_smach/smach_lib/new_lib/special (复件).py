#! /usr/bin/env python
#encoding:utf8

import rospy
from smach import State, UserData, StateMachine
from smach_ros import SimpleActionState, ServiceState, MonitorState
from xm_msgs.srv import *
from xm_msgs.msg import *
from leg_tracker.msg import *
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
from xm_smach.target_gpsr import gpsr_target



class LegTracker():
    def __init__(self):
        self.tracker = MonitorState('/people_tracked',
                                        PersonArray,
                                        self.people_cb,
                                        max_checks =5,
                                        input_keys = ['people_id'],
                                        output_keys=['pos_xm','people_id']
                                        )
        self.tf_listener = tf.TransformListener()

# 如果相机找到人,这个状态将会返回False
# 相反,如果在五个循环里相机都没找到人,将会返回True
# msg传入主题的数据
    def people_cb(self,userdata,msg):
        
        rospy.logwarn('people_id:' + str(userdata.people_id))
        if msg is not None:
            rospy.logerr(msg)
            if len(msg.people) == 0:
                rospy.sleep(3)
                return True
            if  userdata.people_id == -1:
                userdata.people_id = msg.people[0].id
                return True

            rospy.logerr('1')
            peoples_con = msg.people
            self.tmp_pos=PointStamped()
            for i in range(len(peoples_con)):
                if(peoples_con[i].id == userdata.people_id):
                    self.tmp_pos = self.transToBase(peoples_con[i].pose.position)
                    break
                if i==len(peoples_con)-1:
                    i = i+1
            
            rospy.logerr("i"+str(i))
            rospy.logerr(str(len(peoples_con)))
            if i>=len(peoples_con):
                userdata.people_id = -1
                rospy.sleep(2)
                return True
            #rospy.logwarn(self.tmp_pos)
            try:
                
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

    def transToBase(self , point_xm):
        new_header = Header()
        new_header.frame_id = 'map'
        xm_point_stamped = PointStamped()
        xm_point_stamped.point = point_xm
        xm_point_stamped.header = new_header
        rospy.logwarn(xm_point_stamped)
        rospy.logwarn("!!!!")
        self.tf_listener.waitForTransform('base_link', 'map',rospy.Time(), rospy.Duration(1))
        base_point = self.tf_listener.transformPoint('base_link' , xm_point_stamped)


        return base_point

    def data_deal_turn(self,pos_xm):
        #图像到导航的坐标转换
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y
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
        self.tf_listener.waitForTransform('map', 'base_link',rospy.Time(), rospy.Duration(1))
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
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y
        
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





class GpsrNextDo(State):
    def __init__(self):
        #add place outcome if necessary
        State.__init__(self , outcomes = ['done' , 'nav' , 'pick' ,'follow' , 'talk' ,'find' , 'aborted' ,'error','put','speak'],
                                input_keys = ['tasksNum','actions','targets'],
                                io_keys = ['indice'],
                                output_keys = ['current_target'])

    def execute(self , userdata):

        userdata.indice = userdata.indice+1
        indice = userdata.indice
        if(userdata.indice>=userdata.tasksNum):
            return 'done'

        action = userdata.actions[indice] 
        current_target = userdata.targets[indice]
        rospy.logwarn(action)
        rospy.logwarn(current_target)
        userdata.current_target = current_target

        if(action == 'go'):
            return 'nav'
        elif (action == 'find'):
            return 'find'
        elif (action == 'follow'):
            return 'follow'
        elif (action == 'grasp'):
            return 'pick'
        elif (action == 'put'):
            return 'put'
        elif (action == 'talk'):
            return 'talk'
        elif action == 'place':
            return 'place'
        elif action == 'tell':
            return 'speak'
        
        else :
            userdata.indice = userdata.indice-1
            return 'aborted'



class Wait(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=["succeeded", 'error'],
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

class  GpsrGetTabPos(State):
    def __init__(self):
        State.__init__(self , outcomes=['succeeded' , 'aborted' ,'error'] ,
                            input_keys=['target','targets' , 'indice'] , 
                            output_keys=['nav_pos','newik_id'] , 
                            io_keys=['table_num'])

    def execute(self , userdata):
        try:
            getattr(userdata,'targets')
            getattr(userdata,'indice')
            getattr(userdata,'target')
        except:
            rospy.logerr('No params specified')
            return 'error'
        
        try:
            
            table_address_str = userdata.targets[userdata.indice-1]+'_table_'+str(userdata.table_num)
            table_num =userdata.table_num
            if(table_address_str in gpsr_target):    
                userdata.nav_pos = gpsr_target[table_address_str]['pos']
                rospy.logwarn(table_address_str)
                userdata.table_num = table_num+1
                return 'succeeded'
            else:
                rospy.logerr('There is no more table')
		        userdata.table_num = 1
                return 'aborted'
        
        except Exception,e:
            rospy.logerr(e)
            return 'error'


class PosJustfy(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['object_pos','distance'],
                       output_keys=['pose'])

        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        try:
            getattr(userdata, 'object_pos')
        except:
            rospy.logerr('No params specified')
            return 'error'
        
        rospy.logwarn('userdata.object pos')
        rospy.logwarn(userdata.object_pos)

        object_pos = self.tf_listener.transformPoint(
            'base_link', userdata.object_pos)
        # data deal
        # object_pos传输从左到右第一物品的位置见FindObject
        pos_xm = object_pos
        # 坐标位置x,y
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y
        distance = userdata.distance
        rospy.logwarn('distance:'+str(distance))
        angle = atan2(person_y, person_x)
        # gpsr length = 0.7  help_me = 0.8
        person_x = person_x - distance*cos(angle)
        person_y = person_y - distance*sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y = person_y
        pos_xm.point.z = 0
        # init the stamped of the Header
        new_header = Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        q_angle = quaternion_from_euler(0, 0, angle)
        q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = q
        # 全局坐标系和base_link间的关系变换发布
        self.tf_listener.waitForTransform(
            'map', 'base_link', rospy.Time(), rospy.Duration(60.0))
        # if error can directly raise the interupt
        # 如果发生错误将直接打断程序
        rospy.logwarn('wait for tf succeeded ')

        pos_xm = self.tf_listener.transformPoint('map', pos_xm)
        obj_pos = self.tf_listener.transformPoint('map', object_pos)
        rospy.logerr('obj_pos:')
        rospy.logerr(obj_pos)
        rospy.logwarn('tf point succeeded ')

        qs = self.tf_listener.transformQuaternion('map', qs)
        rospy.logwarn('tf quaternion succeeded ')
        userdata.pose = Pose(pos_xm.point, qs.quaternion)
        rospy.logerr(pos_xm.point)
        return 'succeeded'



def get_pid(name):
    return map(int,subprocess.check_output(["pidof",name]).split())

class CloseKinect(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    def execute(self,userdata):
        try:
            # subprocess.call("xterm -e touch /home/ye/Recognition/kinect2/dummy_excution_final &" , shell = True)
            pid_str = subprocess.check_output('ps -aux | grep /people_tracking' , shell= True)
            rospy.logwarn(pid_str)
            pid_str1 = pid_str.splitlines()[0].split()[1]
            # pid_str1 = get_pid('people_tracking')
            rospy.logwarn(pid_str1)
            subprocess.call('kill '+pid_str1, shell = True)
            rospy.sleep(1.0)
        except Exception,e :
            rospy.logwarn(e)
            return 'aborted'
        return 'succeeded'

class GpsrPersonOrPosition(State):### switch the goal among person , speaker, thing, position
    def __init__(self):
        State.__init__(self, 
                        outcomes=['error','person','position'],
                        input_keys=['targets','indice'])
    def execute(self,userdata):
        try:
            getattr(userdata, 'indice')
            getattr(userdata, 'targets')
        except:
            rospy.logerr('No param specified')
            return 'error'
        self.target = userdata.targets[userdata.indice]
        if self.target=='person' or self.target=='speaker':
            return 'person'
        else :
            return 'position'

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


class GpsrGetRoomPos(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                            input_keys =['target'],
                            output_keys =['nav_pos'])
    def execute(self,userdata):
        
        
        rospy.logwarn(userdata.target)
        #rospy.logwarn(userdata.current_task)
            #if userdata.current_task == 'speaker':
            #     userdata.current_target = gpsr_target[userdata.target['speaker2']]['pos']
            # else:
        userdata.nav_pos = gpsr_target[userdata.target]['pos']
        
        return 'succeeded'

class ShoppingGetTask(State):
    def __init__(self):
        State.__init__(self,
                        outcomes = ['succeeded','aborted'],
                        input_keys = ['task'],
                        output_keys= ['task'])
    
        self.target_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        self.tf_listener = tf.TransformListener()
    def execute(self,userdata):
        try:
            self.target_client.wait_for_service(timeout = 10)
            self.response = self.target_client.call(command = 6)
            self.object = self.response.object
            
            rospy.logwarn(self.object)
            userdata.task.append(self.object[0])
            userdata.task.append('cashier')
            rospy.logwarn(userdata.task)

        except:
            rospy.logerr('something wrong')
            return 'aborted'

        return 'succeeded'

class ShoppingNextTask(State):
    def __init__(self):
        State.__init__(self,
                        outcomes=['go','back','finish','aborted'],
                        io_keys=['indice'],
                        output_keys=['pose','name'],
                        input_keys=['task','mission'])

    def execute(self ,userdata):
        try:
            getattr(userdata , 'task')
            getattr(userdata , 'mission')

        except:
            rospy.logerr('No param')
            return 'aborted'
        
        try:
            self.indice = userdata.indice
            mission = userdata.mission
            rospy.logwarn(self.indice)
            rospy.logwarn(mission)

            if(self.indice>3):
                return 'finish'
                
            if(self.indice%2 == 0):
                obj = userdata.task[self.indice]
                rospy.logwarn('GO for '+obj)
                userdata.pose = mission[str(obj)]
                userdata.name = obj
                userdata.indice = self.indice+1
                return 'go'

            else:
                userdata.pose = mission['cashier']
                userdata.indice = self.indice+1
                return 'back'
        
        except Exception,e:
            rospy.logwarn(e)
            return 'aborted'

#class GpsrInstruction


        
        
