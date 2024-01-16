#!/usr/bin/env python
# encoding:utf8
from turtle import pos
import rospy
from smach import *
from smach_ros import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import *
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from geometry_msgs.msg import *


'''
Author: 
Date: 2022-08-25
LastEditTime: 2022-08-25 13:49:54
LastEditors: yishui
Description: Codes for nav
FilePath: ~/catkin_ws/src/xm_smach/xm_smach/smach_lib/smach_common
'''


class NavStack(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],  # preemeted
                       input_keys=['pos_xm'])
        self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        try:
            getattr(userdata, 'pos_xm')
        except:
            rospy.logerr('No param specified ')
            return 'error'
        else:
            rospy.logwarn(userdata.pos_xm)
            self.nav_client.wait_for_server(rospy.Duration(120))

            clear_client = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            # clear_client = rospy.ServiceProxy('/move_base_simple/goal', Empty)    
            req = EmptyRequest()
            res = clear_client.call()
            print(9999999999999)
            self.nav_thing(userdata.pos_xm)
            return 'succeeded'

    def nav_thing(self, pos_xm):
        # goal = MoveBaseGoal()
        # goal.target_pose.header.frame_id = "map"
        # goal.target_pose.pose = pos_xm
        # goal.target_pose.pose.position.z = 0
        # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        
        # self.nav_client.send_goal(goal)
        self.pubulisher = rospy.Publisher(
            '/move_base_simple/goal', Pose, queue_size=1)

        # self.pose = Point()
        # self.pose.x = goal.target_pose.pose.position.x
        # self.pose.y = goal.target_pose.pose.position.y
        # self.pose.z = goal.target_pose.pose.position.z
        # print(self.pose)
        
        # self.pubulisher.publish(self.pose)
        self.pose = PoseStamped()
        self.pose.header.stamp = rospy.Time.now()
        self.pose.header.frame_id = "map"

        self.pose.pose = pos_xm
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!")
        print(self.pose)
        self.pubulisher.publish(self.pose)
        # self.pubulisher.publish({
        #     'header':self.pose.header,
        #     'point':self.pose.pose
        # })


        nav_counter = 0
        while self.nav_client.get_state() != GoalStatus.SUCCEEDED and nav_counter < 150:
            nav_counter += 1
            if self.preempt_requested():
                rospy.logerr('preempted')
                return 'succeeded'
            else:
                pass
            rospy.sleep(0.5)

        (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time())
        deta = sqrt(pow(trans[0] - pos_xm.position.x, 2) + pow(trans[1] - pos_xm.position.y, 2))
        rospy.logerr("deta:" + str(deta))

        if self.nav_client.get_goal_status_text() == 'Goal reached.':
            rospy.loginfo("nav task executes successfully ^_^")
            return 'succeeded'
        else:
            rospy.logerr("xm cannot find the way  T_T")
            return 'aborted'

class NavStack0(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['pos_xm'])
        self.nav_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

        self.tf_listener = tf.TransformListener()
        self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.twist_xm = Twist()

    def execute(self, userdata):
        # 当mode ==1 时xm导航到人, 需要xm面对这人
        try:
            getattr(userdata, 'pos_xm')
        except:
            rospy.logerr('No param specified ')
            return 'error'
        else:
            # 等待60s到action服务可以使用
            rospy.logwarn(userdata.pos_xm)
            if userdata.pos_xm.position.x == -9:
                rospy.logwarn('!!!!!!lost people,and I will turn left!')
                self.twist_xm.angular.z = 0.7
                self.cmd_vel.publish(self.twist_xm)
                rospy.logwarn('turn left!!!!!!!!')
                rospy.sleep(0.3)
                return 'succeeded'

            elif userdata.pos_xm.position.x == -12:
                rospy.logwarn('########lost people,and I will turn right!')
                self.twist_xm.angular.z = -0.7
                self.cmd_vel.publish(self.twist_xm)
                rospy.logwarn('turn right!!!!!!!!')
                rospy.sleep(0.3)
                return 'succeeded'

            # if userdata.pos_xm.position.z == 10:
            rospy.logwarn('!!!!! follow !!!!!!!!')
            person_x = userdata.pos_xm.position.x
            person_y = userdata.pos_xm.position.y
            angle = atan(person_y / person_x)
            self.twist_xm.angular.z = 0.0
            self.twist_xm.linear.x = 0.0
            if 0.2 < angle < 1.0:
                #self.twist_xm.angular.z = 0.2
                self.twist_xm.angular.z = angle / 3
                # if person_x < 0.6:
                # self.twist_xm.linear.x = -0.25
                if person_x > 1.1:
                    self.twist_xm.linear.x = 0.35
                elif 0.85 < person_x < 1.1:
                    #self.twist_xm.linear.x = 0.2
                    self.twist_xm.linear.x = person_x / 4
            elif -0.2 > angle > -1:
                #self.twist_xm.angular.z = -0.2
                self.twist_xm.angular.z = angle / 3
                # if person_x < 0.6:
                # self.twist_xm.linear.x = -0.25
                if person_x > 1.1:
                    self.twist_xm.linear.x = 0.35
                elif 0.85 < person_x < 1.1:
                    #self.twist_xm.linear.x = 0.2
                    self.twist_xm.linear.x = person_x / 4
            elif angle > 1.0:
                self.twist_xm.angular.z = 0.4
                if person_x > 1.1:
                    self.twist_xm.linear.x = 0.35
                elif 0.85 < person_x < 1.1:
                    #self.twist_xm.linear.x = 0.2
                    self.twist_xm.linear.x = person_x / 4
            elif angle < -1.0:
                self.twist_xm.angular.z = -0.4
                if person_x > 1.1:
                    self.twist_xm.linear.x = 0.35
                elif 0.85 < person_x < 1.1:
                    #self.twist_xm.linear.x = 0.2
                    self.twist_xm.linear.x = person_x / 4
            else:
                # if person_x < 0.6:
                # self.twist_xm.linear.x = -0.25
                if person_x > 1.1:
                    self.twist_xm.linear.x = 0.35
                elif 0.85 < person_x < 1.1:
                    #self.twist_xm.linear.x = 0.2
                    self.twist_xm.linear.x = person_x / 4

            # if self.preempt_requested():
            #    rospy.logerr('preemted')
            #    self.nav_client.send_goal(goal)
            # self.nav_client.cancel_goal()
            #    return 'aborted'
            rospy.logwarn('!!!cmd_vel published!')
            self.cmd_vel.publish(self.twist_xm)
            rospy.logwarn('!!!! nav task executes successfully ^_^')
            return 'succeeded'

            # rospy.logwarn('follow^^^^^^^^^^')
            # self.nav_client.wait_for_server(rospy.Duration(60))
            # return self.nav_thing(userdata.pos_xm)

    def nav_thing(self, pos_xm):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = pos_xm
        self.nav_client.send_goal(goal)
        nav_counter = 0
        # if nav stack is failed, we can plan again
        while self.nav_client.get_state() != GoalStatus.SUCCEEDED and nav_counter < 50:
            nav_counter += 1
            # 如果navstack状态被占用，我们将停止nav-task
            # 所以我们需要有一个多出来的preempted输出来完成这个状态
            # 向高层状态机跳出
            # 不要忘记刷新preempted标志来使容器的execute函数更安全
            # TODO:add the outcome 'preempted' and modify all the python-snippet
            # 下面代码：如果被占用取消目标并返回succeeded
            if self.preempt_requested():
                rospy.logerr('preemted')
                self.nav_client.send_goal(goal)
                # self.nav_client.cancel_goal()
                return 'aborted'
            else:
                pass
            rospy.sleep(0.5)
        # 如果到达目标地点返回succeeded,否则返回aborted
        if self.nav_client.get_goal_status_text() == 'Goal reached.':
            rospy.loginfo("nav task executes successfully ^_^")
            return 'succeeded'
        else:
            rospy.logerr("xm cannot find the way  T_T")
            return 'aborted'


class TurnDegree(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['degree'])
        self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
        try:
            getattr(userdata, 'degree')
        except:
            rospy.logerr('No param specified!')
            return 'error'

        else:
            if userdata.degree == 0:
                return 'succeeded'
            rospy.loginfo("the degree received is:" + str(userdata.degree))
            goal_angle = userdata.degree
            # 1 radis/s
            angular_speed = goal_angle / abs(goal_angle)

            self.move_cmd = Twist()
            self.move_cmd.linear.x = 0.0
            self.move_cmd.linear.y = 0.0
            self.move_cmd.linear.z = 0.0
            self.move_cmd.angular.x = 0.0
            self.move_cmd.angular.y = 0.0
            self.move_cmd.angular.z = angular_speed

            # 需要的时间
            angular_duration = abs(goal_angle / angular_speed)
            # 发布频率
            rate = 50
            r = rospy.Rate(rate)

            try:
                print((2 * 3.1415926535 - goal_angle) * 0.1)
                ticks = int(angular_duration * rate + (2 * 3.1415926535 - goal_angle) * 0.1)
                for i in range(ticks):
                    self.cmd_vel.publish(self.move_cmd)
                    r.sleep()
                rospy.sleep(0.3)
                # 发送一个空的消息，让机器人停下来
                self.move_cmd = Twist()
                self.cmd_vel.publish(self.move_cmd)
                rospy.sleep(0.3)
                return 'succeeded'
            except:
                rospy.logerr('meet question when publish Twist')
                return 'aborted'


class LinearDisplacement(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'preempted', 'error'],
                       input_keys=['displacementVec'])
        self.LinearDClient = rospy.ServiceProxy('/directMove/move', xm_Move)

    def execute(self, userdata):

        try:
            getattr(userdata, 'displacementVec')

        except Exception, e:
            rosy.logerr(e)
            rospy.logerr("lacking necessary displacementVec ")
            return 'error'

        point = userdata.displacementVec

        try:
            self.LinearDClient.wait_for_service(30)
        except Exception as e:
            rospy.logerr(e)
            rospy.logerr('Service Timeout')
            return 'error'

        try:
            rospy.logwarn('The vector is:')
            rospy.logwarn(point)
            self.LinearDClient.call(xm_MoveRequest(point))

            if self.preempt_requested():
                rospy.logwarn('preemted')
                self.recall_preempt()
                self.LinearDClient.call(xm_MoveRequest(point))

                if self.preempted_request():
                    rospy.logerr('preemptd2')
                    return 'preempted'

        except Exception, e:
            rospy.logerr(e)
            rospy.logerr('Call Failed')
            return 'error'

        return 'succeeded'


# class DoorDetect():
#     def __init__(self):
#         self.door_detect_ = MonitorState('DoorState', Bool, self.door_cb,max_checks =1)

#     def door_cb(self,userdata,msg):
#         if msg.data == True:
#             clear_client = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
#             req = EmptyRequest()
#             res = clear_client.call(req)
#             return False
#         else:
#             return True

class GoAhead(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['move_len'])
        self.cmd_vel = rospy.Publisher(
            '/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
        try:
            self.turn = Twist()
            self.turn.linear.x = 0.2 * int(abs(userdata.move_len) / userdata.move_len)
            self.turn.linear.y = 0.0
            self.turn.linear.z = 0.0
            self.turn.angular.x = 0.0
            self.turn.angular.y = 0.0
            self.turn.angular.z = 0.0

            angular_duration = abs(userdata.move_len / 0.2)
            # 发布频率
            rate = 50
            r = rospy.Rate(rate)
            try:
                # 用1s进行旋转
                rospy.logwarn(angular_duration * rate)
                ticks = abs(int(angular_duration * rate))
                for i in range(ticks):
                    self.cmd_vel.publish(self.turn)
                    rospy.logwarn(i)
                    r.sleep()
                rospy.sleep(1.0)
                return 'succeeded'
            except:
                rospy.logerr('meet question when publish Twist')
                return 'aborted'

        except Exception, e:
            return 'error'
