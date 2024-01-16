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



class ArmCmd(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['mode', 'arm_ps', 'objmode'],
                       input_keys=['object_state'])
        self.xm_arm_client = rospy.ServiceProxy('arm_stack', xm_PickOrPlace)
        # 确保用于机械臂pick_or_place的PointStamped(抓取点的位置)在底盘base_footprint的坐标上
        # make sure that the PointStamped used for arm pick_or_place is in the frame base_footprint
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        rospy.loginfo('Moving arm')
        try:
            getattr(userdata, 'mode')
            getattr(userdata, 'arm_ps')
            getattr(userdata, 'objmode')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            try:
                req = xm_PickOrPlaceRequest()
                req.action = userdata.mode
                self.arm_ps_ = PointStamped()
            
                if userdata.arm_ps.header.frame_id is not '':
                    rospy.loginfo('frame checked')
                    self.arm_ps_ = self.tf_listener.transformPoint(
                        'base_link', userdata.arm_ps)
                else:
                    pass
                if (userdata.object_state == 0):
                    rospy.logerr('fuck the if')
                    self.arm_ps_.point.x += 0.18 # -0.14 -0.10 store -0.05
                    self.arm_ps_.point.y +=0.06
                    self.arm_ps_.point.z += -0.18#0.23  0.42
                else:
                    self.arm_ps_.point.x += 0.14 # -0.14 -0.10 store -0.05
                    self.arm_ps_.point.y +=0.06
                    self.arm_ps_.point.z += -0.14#0.23  0.42
                req.goal_position = self.arm_ps_
                req.objmode = userdata.objmode
                rospy.logwarn(req)
                res = self.xm_arm_client.call(req)
                if res.result == True:
                    return 'succeeded'
                else:
                    return 'aborted'
            except Exception, e:
                rospy.logerr(e)
                return 'aborted'


class ArmCmdForTf(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['mode', 'arm_ps', 'objmode'])
        self.xm_arm_client = rospy.ServiceProxy('arm_stack', xm_PickOrPlace)
        # 确保用于机械臂pick_or_place的PointStamped(抓取点的位置)在底盘base_footprint的坐标上
        # make sure that the PointStamped used for arm pick_or_place is in the frame base_footprint
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        rospy.loginfo('Moving arm')
        try:
            getattr(userdata, 'mode')
            getattr(userdata, 'arm_ps')
            getattr(userdata, 'objmode')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            try:
                req = xm_PickOrPlaceRequest()
                req.action = userdata.mode
                self.arm_ps_ = PointStamped()
                if userdata.arm_ps.header.frame_id is not '':
                    rospy.loginfo('frame checked')
                    self.arm_ps_ = self.tf_listener.transformPoint(
                        'base_link', userdata.arm_ps)
                else:
                    pass
                self.arm_ps_.point.x += 0.03 # -0.14 -0.10 store -0.05
                self.arm_ps_.point.y +=0.08
                self.arm_ps_.point.z += -0.09#0.23  0.42
                req.goal_position = self.arm_ps_
                req.objmode = userdata.objmode
                rospy.logwarn(req)
                res = self.xm_arm_client.call(req)
                if res.result == True:
                    return 'succeeded'
                else:
                    return 'aborted'
            except Exception, e:
                rospy.logerr(e)
                return 'aborted'
