#! /usr/bin/env python
# encoding:utf8

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

#A simple move 
class LinearDisplacement(State):
    def __init__(self):
        State.__init__(self , outcomes = ['succeeded' , 'preempted' , 'error'] , 
                        input_keys = ['displacementVec'])
        self.LinearDClient = rospy.ServiceProxy('/directMove/move', xm_Move)

    def execute(self , userdata):

        try:
            getattr(userdata , 'displacementVec')

        except Exception , e:
            rosy.logerr(e)
            rospy.logerr("lacking necessary displacementVec ")
            return 'error'

        point = userdata.displacementVec

        try:
            self.LinearDClient.wait_for_service(30)
        except Exception,e:
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
            
        except Exception,e:
            rospy.logerr(e)
            rospy.logerr('Call Failed')
            return 'error'
        
        return 'succeeded'


class TurnDegree(State):
    def __init__(self):
        State.__init__(self, outcomes=[
                       'succeeded', 'aborted' , 'preempted'],
                       input_keys = ['degree'])
        # self.tf_Listener = tf.TransformListener()
        self.cmd_vel = rospy.Publisher(
            '/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):

        try:
            # now = rospy.Time(0)
            # # self.tf_Listener.waitForTransform('base_link' , 'map' , now , rospy.Duration(1.0))
            # # turn_pose= self.tf_Listener.lookupTransform('base_link', 'map', now)            rospy.logwarn('position-------')
            # rospy.logerr(turn_pose[1])
            # rospy.logwarn(turn_pose)
            # quaternion1 = turn_pose[1][0]
            # quaternion2 = turn_pose[1][1]
            # quaternion3 = turn_pose[1][2]
            # quaternion4 = turn_pose[1][3]
            # angular = euler_from_quaternion([quaternion1, quaternion2, quaternion3, quaternion4])
            # rospy.logwarn(angular)
            # angle = angular[2] + pi/2
            # rospy.logwarn(angle)
            # quaternion = quaternion_from_euler(0, 0, angle)
            # rospy.logwarn(quaternion)
            # userdata.turn_pose.orientation = Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
            # userdata.turn_pose.position = Point(turn_pose[0][0],turn_pose[0][1],turn_pose[0][2])
            # rospy.logwarn(userdata.turn_pose)
            self.turn = Twist()
            self.turn.linear.x = 0.0
            self.turn.linear.y = 0.0
            self.turn.linear.z = 0.0
            self.turn.angular.x = 0.0
            self.turn.angular.y = 0.0
            self.turn.angular.z = 0.8*abs(userdata.degree)/userdata.degree

            rospy.logwarn(self.turn)
            
            speed = abs(self.turn.angular.z)
            rate = 50
            goal_angle = userdata.degree
            angual_duration = abs(goal_angle)/speed
            r = rospy.Rate(rate)
            rospy.logerr(goal_angle)
            
                
            ticks = int(abs(goal_angle)*rate+55)
            for i in range(ticks):
                self.cmd_vel.publish(self.turn)

                if self.preempt_requested():
                    rospy.logerr('preemptd')
                    self.recall_preempt()
                    self.cmd_vel.publish(self.turn)

                    if self.preempt_requested():
                        rospy.logerr('preemptd2')
                        return 'preempted'

                r.sleep()
            # move_cmd = Twist()
            # self.cmd_vel.publish(move_cmd)
          
            rospy.sleep(6.0)
            return 'succeeded'
        except Exception, e:
            rospy.logerr(e)
            return 'aborted'

        
class NavStack(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error' ,'preempted'],
                       input_keys=['pos_xm'])
        self.nav_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction)

        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        try:
            getattr(userdata, 'pos_xm')
        except:
            rospy.logerr('No param specified ')
            return 'error'
        else:
            rospy.logwarn(userdata.pos_xm)
            self.nav_client.wait_for_server(rospy.Duration(60))
            return self.nav_thing(userdata.pos_xm)

    def nav_thing(self, pos_xm):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose = pos_xm
        self.nav_client.send_goal(goal)
        nav_counter = 0
        while self.nav_client.get_state() != GoalStatus.SUCCEEDED and nav_counter < 50:
            nav_counter += 1
            if self.preempt_requested():
                rospy.logerr('preempted')
                # self.recall_preempt()
                # self.nav_client.send_goal(goal)
                # if self.preempt_requested():
                #     rospy.logerr('preemtped2')
                return 'succeeded'
            else:
                pass
            rospy.sleep(0.5)

        
        (trans, rot) = self.tf_listener.lookupTransform('map' , 'base_link' , rospy.Time())
        deta =sqrt(pow(trans[0]-pos_xm.position.x ,2)+pow(trans[1] - pos_xm.position.y ,2))
        rospy.logerr("deta:" + str(deta))
        if self.nav_client.get_goal_status_text() == 'Goal reached.':
            rospy.loginfo("nav task executes successfully ^_^")
            return 'succeeded'
        else:
            rospy.logerr("xm cannot find the way  T_T")
            return 'aborted'



class FollowTest(State):
    def __init__(self):
        State.__init__(self,outcomes = ['succeeded' , 'aborted' , 'error'])

        self.moveClient = rospy.ServiceProxy('/directMove/move', xm_Move)
        

    def execute(self, userdata):
        try:
            self.moveClient.wait_for_service(10.0)
        
        except Exception,e:
            rospy.logerr('no sevice')
            return 'aborted'

        try:
            self.circle()
        except Exception,e:
            rospy.logerr('circle error')
            return 'error'

        return 'succeeded'


    def circle(self):
        
        rospy.Subscriber('follow' , xm_FollowPerson , 
                                    self.posSubCb , queue_size = 5 )
        rospy.sleep(5.0)
        rospy.logwarn('sleep over')
           
        rospy.spin()


    def posSubCb(self, msg):
        
        #publishe  Twist()????
        try:    
            moveRequest = Point()
            moveRequest.point.x = msg.position.point.x
            moveRequest.point.y = msg.position.y
            moveRequest.point.z = 0.0

            rospy.logwarn(moveRequest)

            if(self.preempt_requested()):
                rospy.logerr('preempted')

            moveResponse = self.moveClient.call(moveRequest)
            rospy.logwarn(moveResponse)

            rospy.sleep(2.0)
        
        except Exception,e:
            rospy.logerr(e)



class DoorDetect():
    def __init__(self):
        self.door_detect_ = MonitorState('DoorState', Bool, self.door_cb,max_checks =1)

    def door_cb(self,userdata,msg):
        if msg.data == True:
            clear_client = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
            req = EmptyRequest()
            res = clear_client.call(req)
            return False
        else:
            return True
        


class GoAhead(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'])
        self.cmd_vel = rospy.Publisher(
            '/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):

            # if self.cmd_vel.get_num_connections() == 0:
            #     rospy.logerr('This state did nothing!')
          #     return 'succeeded'
        try:
            self.turn = Twist()
            self.turn.linear.x = 0.2
            self.turn.linear.y = 0.0
            self.turn.linear.z = 0.0
            self.turn.angular.x = 0.0
            self.turn.angular.y = 0.0
            self.turn.angular.z = 0.0

            angular_duration = userdata.move_len/0.2
            # 发布频率
            rate = 50
            r = rospy.Rate(rate)
            try:
                    # 用1s进行旋转
                rospy.logwarn(angular_duration*rate)
                ticks = abs(int(angular_duration*rate))+5
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

        

    
