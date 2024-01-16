#!/usr/bin/env python
# encoding:utf8

import rospy
from smach import *
from smach_ros import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import *
from math import pi, atan
import tf
#from turn.srv import *


class check_turn(State):
    def __init__(self):
        State.__init__(self, outcomes=['yes', 'no','stop'])
        self.check_client = rospy.ServiceProxy('/turning', turnings)

    def execute(self, userdata):
        try:
            self.req = turningsRequest()
            self.req.check = True
            self.timesss = 0
            rospy.logwarn('runing-----')
            rospy.sleep(0.5)

            while not rospy.is_shutdown():
                self.check_client.wait_for_service(1.0)
                self.ans = self.check_client.call(self.req)
                rospy.logwarn(self.ans)
                self.timesss=self.timesss+1
                rospy.sleep(0.5)
                if(self.ans.turning == True):
                    return 'yes'
                # elif(self.timesss>=3):
                #     return 'no'
                elif(self.preempt_requested()):
                    rospy.logwarn('stop')    
                    return 'no'
        except Exception,e:
            rospy.logerr(e)
            return 'no'


class go_turn(State):
    def __init__(self):
        State.__init__(self, outcomes=[
                       'succeeded', 'aborted'])
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
            self.turn.angular.z = 0.6
            speed = self.turn.angular.z
            rate = 50
            goal_angle = 3.1415926/2
            angual_duration = goal_angle/speed
            r = rospy.Rate(rate)

            ticks = int(goal_angle*rate)
            for i in range(ticks):
                self.cmd_vel.publish(self.turn)
                r.sleep()
            move_cmd = Twist()
            self.cmd_vel.publish(move_cmd)
          
            rospy.sleep(6.0)
            return 'succeeded'
        except Exception, e:
            rospy.logerr(e)
            return 'aborted'
