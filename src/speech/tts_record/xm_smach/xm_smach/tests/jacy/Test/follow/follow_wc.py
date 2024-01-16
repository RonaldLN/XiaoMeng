#!/usr/bin/env python
# encoding:utf8
'''
Author: jacy
Date: 2020-09-06 16:15:37
LastEditTime: 2020-09-08 22:06:51
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

        self.screen_pub = rospy.Publisher('goal_running_pose',PoseStamped, queue_size=10)    

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

    def execute(self,userdata):
        while True:
            a = 1
        
        return 'succeeded'
        
    def move_base_follow_cb(self,msg):
        
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
        self.need_to_send_goal_point = self.tf_listener.transformPoint('map',msg.position)

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
            person_base_point = self.tf_listener.transformPoint('base_link',msg.position)

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
        
        #初始化xm现在的位置用于之后得到base_link在全局坐标系中的位置
        person_x = person_x - self.distance_to_turn * math.cos(angle)
        person_y = person_y - self.distance_to_turn * math.sin(angle)
        
        person_base_point.point.x = person_x
        person_base_point.point.y =person_y
        person_base_point.point.z =0

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

#--------------------------------------------------------------------------------------------------------------------------------#
class CheckStop(State):
    def __init__(self):
        State.__init__(self,outcomes =['stop','error'])
        self.start_time = rospy.Time.now()

    def execute(self,userdata):
        while rospy.Time.now() - self.start_time < rospy.Duration(60.0):
            pass
        return 'stop'


#------------------------------------------------------------------------------#
#1. 自己写一个底盘仿真控制脚本
#2. 设置一个有无正在进行的goal，现在反应太慢，也可以通过减小允许实行时间来达到目的
#3. 未配合在一起使用
#4. 关键在于要到达点的选择与确定

#------------------------------for camera-------------------------------------------#
class C_Follow():
    def __init__(self):
        
        rospy.init_node('follow')
        rospy.on_shutdown(self.shutdown)
        self.smach_bool = False
        

        self.follow = StateMachine(outcomes =['succeeded','aborted','error'])
        
        with self.follow:
            StateMachine.add('CAMERA_FOLLOW',CameraFollow(),
                             transitions={'succeeded': 'succeeded','error': 'error'})


        intro_server = IntrospectionServer('sm_follow' , self.follow , '/SM_ROOT')
        intro_server.start()
        out = self.follow.execute()
        print out
        intro_server.stop()
        self.smach_bool =True

    def shutdown(self):
        if self.smach_bool == True:
            rospy.logwarn("DONE")
        else:
            rospy.logwarn('FUCK THE ERROE')


#------------------------------for move-base-nav---------------------------------------------#
class M_Follow():
    def __init__(self):
        
        rospy.init_node('follow')
        rospy.on_shutdown(self.shutdown)
        self.smach_bool = False
        

        self.follow = StateMachine(outcomes =['succeeded','aborted','error'])
        
        with self.follow:
            StateMachine.add('MOVEBASE_FOLLOW',MoveBaseFollow(),
                             transitions={'succeeded': 'succeeded','error': 'error'})


        intro_server = IntrospectionServer('sm_follow' , self.follow , '/SM_ROOT')
        intro_server.start()
        out = self.follow.execute()
        print out
        intro_server.stop()
        self.smach_bool =True

    def shutdown(self):
        if self.smach_bool == True:
            rospy.logwarn("finish successfully")
        else:
            rospy.logwarn('exit with something wrong')


class C_M_Follow():
    def __init__(self):
        
        rospy.init_node('follow')
        rospy.on_shutdown(self.shutdown)
        self.smach_bool = False
        

        self.follow = Concurrence(outcomes =['succeeded','camera_follow_error','move_follow_error'],
                                    default_outcome = 'succeeded',
                                    outcome_map={
                                                    'succeeded':{'CHECK_STOP':'stop'},
                                                    'camera_follow_error':{'CAMERA_FOLLOW':'error'},      
                                                    'move_follow_error':{'MOVEBASE_FOLLOW':'error'}
                                                },
                                    child_termination_cb=self.nav_child_cb
                                    )
        
        with self.follow:
            Concurrence.add('MOVEBASE_FOLLOW',MoveBaseFollow())
            Concurrence.add('CAMERA_FOLLOW',CameraFollow())
            Concurrence.add('CHECK_STOP',CheckStop())

        intro_server = IntrospectionServer('sm_follow' , self.follow , '/SM_ROOT')
        intro_server.start()
        out = self.follow.execute()
        print out
        intro_server.stop()
        self.smach_bool =True

    def shutdown(self):
        if self.smach_bool == True:
            rospy.logwarn("finish successfully")
        else:
            rospy.logwarn('exit with something wrong')

    def nav_child_cb(self,outcome_map):
        if outcome_map['CHECK_STOP'] == 'stop':
            rospy.logwarn('finish follow')
            return True
        else:
            rospy.logwarn('continue follow')
            return False    

if __name__ == '__main__':
    try:
	    C_M_Follow()
    except Exception , e:
        rospy.logerr(e)