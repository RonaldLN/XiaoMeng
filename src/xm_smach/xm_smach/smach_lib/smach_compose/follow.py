#!/usr/bin/env python
# encoding:utf8
'''
Author: jacy
Date: 2020-09-06 16:15:37
LastEditTime: 2020-09-20 10:12:01
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: /undefined/home/jacy/gazebo_test_ws/src/xm_smach/xm_smach/tests/jacy/Test/CameraTest/follow.py
'''


from smach import *
from smach_ros import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
import tf
import rospy
import math
from math import *
import subprocess
import actionlib
from actionlib_msgs.msg import GoalStatus
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import *
PI = 3.1415926535
#--------------------------------------------------------------------------------------------------------------------------------#
class CameraFollow(State):
    '''
    摄像头跟随,因为需要时候改变，并不是一个跳转服务，所以不适合使用监视器
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','error'])
        
        rospy.Subscriber('follow', xm_FollowPerson, self.camera_follow_cb)

        self.neck_client = rospy.ServiceProxy('/mobile_base/camera_command',xm_camera)

        self.neck_client.wait_for_service(timeout=10.0)
        
        self.tf_listener = tf.TransformListener()
        
        self.yaw = 0                        #旋转的命令
        self.pitch = 0                      #俯仰    
        self.time_to_change = 3             #3s内，即使需要旋转的相对角度小于5度，也进行一次更新
        self.last_update_time = 0           #上一次更新的时间
    def execute(self,userdata):
        while True:
            a = 1
        
        return 'succeeded'
        
    def camera_follow_cb(self,msg):
        #由于获取的坐标是基于rbg坐标的，我们要使摄像头rbg图像始终对准人
        #也就是算出摄像头要转动的角度后，转换成脖子要转换的角度
        #camera_base_to_rbg = 0.095
        #同时，这里的角度是基于之前的角度，所以要么监听脖子的角度然后转化成绝对角度，要么电子写成相对角度的控制，这里我们选择第1种方式，tf转换
        #同时，这里只有当需要转动的角度大于0.1弧度，也就是5度时，才进行移动，基于这一点，也可能会考虑用监视器控制
        x = msg.position.point.x
        y = -msg.position.point.y

        alpha = abs(math.atan(y/x))
        if x > 0:
            if y > 0:
                angle = alpha
            else:
                angle = -alpha
        else:
            if y > 0:
                angle = PI - alpha
            else:
                angle = alpha - PI
        
        if abs(angle) >0.1 :            #TODO test the time   | ...  or ... |

            #get the base_footprint
            self.tf_listener.waitForTransform('base_footprint','kinect2_rgb_link',rospy.Time(),rospy.Duration(10.0))
            person_base_point = self.tf_listener.transformPoint('base_footprint',msg.position)

            a = person_base_point.point.x
            b = -person_base_point.point.y

            beta = abs(math.atan(b/a))
            if a > 0:
                if b > 0:
                    self.yaw = beta
                else:
                    self.yaw = -beta
            else:
                if b > 0:
                    self.yaw = PI - beta
                else:
                    self.yaw = beta - PI

            req = xm_cameraRequest()
            req.yaw = self.yaw
            req.pitch = 0
            #print(self.yaw)
            self.neck_client.call(req)
            #rospy.sleep(0.01)
        else:
            pass

#--------------------------------------------------------------------------------------------------------------------------------#
class MoveBaseFollow(State):
    '''
    这里没有使用监视器，也可以但开一个话题用于判断和是进行移动、导航,使用监视器或许是更好的选择，主要考虑的是action改变的是否频繁
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','error'])
        
        rospy.Subscriber('follow', xm_FollowPerson, self.move_base_follow_cb)

        self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)

        self.tf_listener = tf.TransformListener()
        
        self.last_goal_point = PointStamped()                   #用来判断
        self.need_to_send_goal_point =PointStamped()            #需要发送的
        self.last_goal_execute_time = rospy.Time.now()          #上一次开始执行的时间
        self.execute_time = rospy.Duration(10.0)                #有效的执行时间
        self.distance_need_to_move = 0.5                        #大于这个距离移动
        self.distance_need_to_change_goal = 0.05                #当新旧goal相差超过此距离，不管是否执行完毕，取消旧目标执行，执行新目标
        self.distance_to_person = 0.7                           #当移动底盘时，靠近人的时候目标点与人的距离
        self.distance_to_turn = 1.0                             #当旋转底盘时，与人保持的距离
        self.if_last_goal_finished = True                       #记录上一个goal是否完成
        self.twist_xm = Twist()

    def execute(self,userdata):
        while True:
            a = 1
        
        return 'succeeded'
        
    def move_base_follow_cb(self,msg):
        
        msg_ = PointStamped()
        msg_.header.frame_id = "kinect2_rgb_link"
        msg_.point = msg.position.point

        if msg_.point.x == -9:
            rospy.logwarn('lost people and I will turn left!')
            #self.twist_xm.angular.z = 0.5
            #self.cmd_vel.publish(self.twist_xm)
            self.cmd_turn(0.5)
            #rospy.sleep(0.3)
            

        elif msg_.point.x == -12:
            #rospy.logwarn('lost people and I will turn right!')
            #self.twist_xm.angular.z = -0.5
            #self.cmd_vel.publish(self.twist_xm)
            #rospy.sleep(0.3)
            self.cmd_turn(-0.5)
        else:
            #TODO get the distance
            x = abs(msg.position.point.x)
            y = abs(msg.position.point.y)
            distance = math.sqrt( x**2 + y**2 )
            
            #TODO send or save the new goal
            #if distance > self.distance_need_to_move:
            #TODO get the position of xm in this time,get the distance of last goal ,then decide if send the new goal,
            #if you do not send ,you need to save the new goal in the self.need_to_send_goal
            #or ,if last goal execute too much time ,we need to cancel it and send the goal or just repeat it again

            #get the map pos
            self.tf_listener.waitForTransform('map','kinect2_rgb_link',rospy.Time(),rospy.Duration(10.0))
            self.need_to_send_goal_point = self.tf_listener.transformPoint('map',msg_)

            #get the distance of old and new goal-point
            new_old_x = self.need_to_send_goal_point.point.x - self.last_goal_point.point.x
            new_old_y = self.need_to_send_goal_point.point.y - self.last_goal_point.point.y
            distance_old_to_new = math.sqrt( new_old_x**2 + new_old_y**2 )
            
            #check if there is goal is running
            if self.nav_client.get_goal_status_text() == 'Goal reached.':
                self.if_last_goal_finished = True

            #distance need to move or time out
            if distance_old_to_new > self.distance_need_to_move or (rospy.Time.now() - self.last_goal_execute_time) > self.execute_time or self.if_last_goal_finished:
                
                rospy.logwarn("**********************")
                rospy.logwarn(self.nav_client.get_goal_status_text())
                #get the base point
                self.tf_listener.waitForTransform('base_link','kinect2_rgb_link',rospy.Time(),rospy.Duration(10.0))
                person_base_point = self.tf_listener.transformPoint('base_link',msg_)

                #change the goal format
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"

                if distance > self.distance_need_to_move:
                    rospy.loginfo("I will move")
                    goal.target_pose.pose = self.data_deal(person_base_point)
                else:
                    rospy.loginfo("I will turn")
                    goal.target_pose.pose = self.data_deal_turn(person_base_point)
                
                #cance the last goal
                self.nav_client.cancel_all_goals()
                #rospy.logwarn(goal.W.pose)
                self.nav_client.send_goal(goal)

                #TODO check if the sent goal was vaild,if not ,the goal was in the black area
                #rospy.logwarn(self.nav_client.get_goal_status_text())

                #update the information of the last goal
                self.last_goal_point = self.need_to_send_goal_point
                self.last_goal_execute_time = rospy.Time.now()
                self.if_last_goal_finished = False


    def data_deal_turn(self,person_base_point):

        #图像到导航的坐标转换
        person_x = person_base_point.point.x
        person_y = person_base_point.point.y
        #计算人和xm连线与视线正前方夹角
        angle = math.atan2(person_y,person_x)
        
        pose_base = PointStamped()
        pose_base.header.frame_id = 'base_link'
        
        #从角度得到四元数
        qs = QuaternionStamped()
        qs.header.frame_id = 'base_link'
        qs.quaternion = Quaternion(*quaternion_from_euler(0,0,angle))

        #等待tf的信息
        self.tf_listener.waitForTransform('map', 'base_link',rospy.Time(), rospy.Duration(1))
        person_base_point = self.tf_listener.transformPoint('map',person_base_point)
        
        qs =self.tf_listener.transformQuaternion('map',qs)
        goal_pose = Pose(person_base_point.point,qs.quaternion)
        
        return goal_pose
    #返回xm经过处理后的Pose()
    def data_deal(self,person_base_point):
        #目前导航点的选择是人的位置，但这是有问题的，人的位置时障碍物，同时还有一种方法是选择机器人与人相连直线上距离0.7米的位置，但这也可能是障碍物
        
        person_x = person_base_point.point.x
        person_y = person_base_point.point.y

        angle = math.atan2(person_y, person_x)
        #person_x = person_x - self.distance_to_person * math.cos(angle)
        #person_y = person_y - self.distance_to_person * math.sin(angle)
        
        person_base_point.point.x = person_x
        person_base_point.point.y =person_y
        person_base_point.point.z =0
         

        # change 
        qs = QuaternionStamped()
        qs.header.frame_id = 'base_link'
        qs.quaternion = Quaternion(*quaternion_from_euler(0,0,angle))
          
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
        person_base_point =self.tf_listener.transformPoint('map',person_base_point)
           
        qs =self.tf_listener.transformQuaternion('map',qs)
        target_pose = Pose(person_base_point.point,qs.quaternion)
        
        return target_pose

    def cmd_turn(self,angle):
        goal_angle = angle
        angular_speed = goal_angle/abs(goal_angle)
        rospy.logwarn(angular_speed)
        self.turn = Twist()
        self.turn.linear.x = 0.0
        self.turn.linear.y = 0.0
        self.turn.linear.z = 0.0
        self.turn.angular.x = 0.0
        self.turn.angular.y = 0.0
        self.turn.angular.z = 2*angular_speed

        angular_duration = goal_angle/angular_speed
        # 发布频率
        rate = 50
        r = rospy.Rate(rate)
        
        # 用1s进行旋转
        rospy.logwarn(angular_duration*rate)
        ticks = abs(int(angular_duration*rate))+5
        for i in range(ticks):
            self.cmd_vel.publish(self.turn)
            rospy.logwarn(i)
            r.sleep()
        rospy.sleep(1.0)

class LegTracker0():
    def __init__(self):
        self.tracker = MonitorState('follow',
                                        xm_FollowPerson,
                                        self.people_cb,
                                        max_checks =5,
                                        output_keys=['pos_xm'])
        self.tf_listener = tf.TransformListener()
    # 如果相机找到人,这个状态将会返回False
    # 相反,如果在五个循环里相机都没找到人,将会返回True
    # msg传入主题的数据
    def people_cb(self, userdata,msg):
        if msg is not None:
            try:
                self.tmp_pos = msg.position
                rospy.logwarn(self.tmp_pos)  
                rospy.logwarn('finding people........')
                if self.tmp_pos.point.x== -9.0 and self.tmp_pos.point.y == -9.0 and self.tmp_pos.point.z == -9.0:
                    rospy.logwarn('lost people and I will turn left!')
                    ps = self.data_deal_lose_left_sign(self.tmp_pos)
                    userdata.pos_xm = ps
                    #pass
                    return False
                if self.tmp_pos.point.x== -12.0 and self.tmp_pos.point.y == -12.0 and self.tmp_pos.point.z == -12.0:
                    rospy.logwarn('lost people and I will turn right!')
                    ps = self.data_deal_lose_right_sign(self.tmp_pos)
                    userdata.pos_xm = ps
                    #pass
                    return False
                #如果得到人的坐标信息返回移动的位置
                #elif self.get_distance(self.tmp_pos)>1 and self.get_distance(self.tmp_pos)<=4.0 :
                elif self.get_distance(self.tmp_pos)<=4.0 :
                    rospy.loginfo('i will move')
                    ps =self.data_deal_new(self.tmp_pos)
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

            
    def get_distance(self,pos_xm):
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y

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
    
    def data_deal(self,pos_xm0):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
        person_x = pos_xm0.point.x
        person_y = pos_xm0.point.y
        angle = atan2(person_y, person_x)
        # 3.28 test  
        # person_x = person_x - 0.7*cos(angle) 
        # person_y = person_y -0.7*sin(angle) 
        pos_xm0.point.x = person_x - 1.0
        pos_xm0.point.y =person_y
        pos_xm0.point.z =0
        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm0.header = new_header
        # change 
        q_angle = quaternion_from_euler(0,0,angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header  =pos_xm0.header
        qs.quaternion = self.q
        
        # self.tf_listener.waitForTransform('base_footprint','camera_link',rospy.Time(),rospy.Duration(60.0))  
        # self.tf_listener.waitForTransform('odom','base_footprint',rospy.Time(),rospy.Duration(60.0))    
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    

        rospy.logwarn('wait for tf succeeded ')    
        
        
        # pos_xm0 =self.tf_listener.transformPoint('base_footprint',pos_xm0)
        # pos_xm0 =self.tf_listener.transformPoint('odom',pos_xm0)
        pos_xm0 =self.tf_listener.transformPoint('map',pos_xm0)
        rospy.logwarn('tf point succeeded ')    
        
        # the angle should also transform to the map frame
        # qs =self.tf_listener.transformQuaternion('base_link',qs)

        # qs =self.tf_listener.transformQuaternion('base_footprint',qs)
        # qs =self.tf_listener.transformQuaternion('odom',qs)
        qs =self.tf_listener.transformQuaternion('map',qs)
        rospy.logwarn('tf quaternion succeeded ')    
        pos_xm = Pose(pos_xm0.point,qs.quaternion)
        return pos_xm

    def data_deal_new(self,pos_xm0):
        
        person_x = pos_xm0.point.x
        person_y = pos_xm0.point.y
        angle = atan2(person_y, person_x)

        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm0.header = new_header
        # change 
        q_angle = quaternion_from_euler(0,0,angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header  =pos_xm0.header
        qs.quaternion = self.q

        pos_xm = Pose(pos_xm0.point,qs.quaternion)
        pos_xm.position.z = 10 

        return pos_xm

    def data_deal_lose_left_sign(self,pos_xm):
        pos_xm = Pose()
        pos_xm.position.x = -9
        return pos_xm

    def data_deal_lose_right_sign(self,pos_xm):
        pos_xm = Pose()
        pos_xm.position.x = -12
        return pos_xm


class LegTracker0_new():
    def __init__(self):
        self.tracker = MonitorState('follow',
                                        xm_FollowPerson,
                                        self.people_cb,
                                        max_checks =5,
                                        output_keys=['pos_xm'])
        self.tf_listener = tf.TransformListener()
        self.last_point = Pose()
    # 如果相机找到人,这个状态将会返回False
    # 相反,如果在五个循环里相机都没找到人,将会返回True
    # msg传入主题的数据
    def people_cb(self, userdata,msg):
        if msg is not None:
            try:
                self.tmp_pos = msg.position
                rospy.logwarn(self.tmp_pos)  
                rospy.logwarn('finding people........')
                if self.tmp_pos.point.x== -9.0 and self.tmp_pos.point.y == -9.0 and self.tmp_pos.point.z == -9.0:
                    rospy.logwarn('lost people and I will turn left!')
                    ps = self.data_deal_lose_left_sign(self.tmp_pos)
                    userdata.pos_xm = ps
                    #pass
                    return False
                if self.tmp_pos.point.x== -12.0 and self.tmp_pos.point.y == -12.0 and self.tmp_pos.point.z == -12.0:
                    rospy.logwarn('lost people and I will turn right!')
                    ps = self.data_deal_lose_right_sign(self.tmp_pos)
                    userdata.pos_xm = ps
                    #pass
                    return False
                #如果得到人的坐标信息返回移动的位置
                elif self.get_distance(self.tmp_pos)>0.5 and self.get_distance(self.tmp_pos)<=4.0 :
                    rospy.loginfo('i will move')
                    ps =self.data_deal(self.tmp_pos)

                    x_ = ps.position.x - self.last_point.position.x 
                    y_ = ps.position.y - self.last_point.position.y
                    if math.sqrt(x_ * x_ + y_ * y_) > 0.05:
                        userdata.pos_xm = ps
                        self.last_point = ps
                    else:       
                        userdata.pos_xm = self.last_point
                    return False
                elif self.get_distance(self.tmp_pos) < 0.5:
                    rospy.loginfo('i will not move')
                    ps = self.data_deal_turn(self.tmp_pos)
                    userdata.pos_xm = ps
                    return False
                else:
                    rospy.logerr('the person is out of the range')
                    return True
            except Exception ,e:
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
    def data_deal(self,pos_xm0):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
        person_x = pos_xm0.point.z
        person_y = pos_xm0.point.x
        angle = atan2(person_y, person_x)
        person_x = person_x - 0.7*cos(angle)
        person_y = person_y -0.7*sin(angle)
        pos_xm0.point.x = person_x
        pos_xm0.point.y =person_y
        pos_xm0.point.z =0
        new_header =Header()
        new_header.frame_id = 'base_link'
        pos_xm0.header = new_header
        # change 
        q_angle = quaternion_from_euler(0,0,angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header  =pos_xm0.header
        qs.quaternion = self.q
        
        # self.tf_listener.waitForTransform('base_footprint','camera_link',rospy.Time(),rospy.Duration(60.0))  
        # self.tf_listener.waitForTransform('odom','base_footprint',rospy.Time(),rospy.Duration(60.0))    
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    

        rospy.logwarn('wait for tf succeeded ')    
        
        
        # pos_xm0 =self.tf_listener.transformPoint('base_footprint',pos_xm0)
        # pos_xm0 =self.tf_listener.transformPoint('odom',pos_xm0)
        pos_xm0 =self.tf_listener.transformPoint('map',pos_xm0)
        rospy.logwarn('tf point succeeded ')    
        
        # the angle should also transform to the map frame
        # qs =self.tf_listener.transformQuaternion('base_link',qs)

        # qs =self.tf_listener.transformQuaternion('base_footprint',qs)
        # qs =self.tf_listener.transformQuaternion('odom',qs)
        qs =self.tf_listener.transformQuaternion('map',qs)
        rospy.logwarn('tf quaternion succeeded ')    
        pos_xm = Pose(pos_xm0.point,qs.quaternion)
        return pos_xm

    def data_deal_lose_left_sign(self,pos_xm):
        pos_xm = Pose()
        pos_xm.position.x = -9
        return pos_xm

    def data_deal_lose_right_sign(self,pos_xm):
        pos_xm = Pose()
        pos_xm.position.x = -12
        return pos_xm

