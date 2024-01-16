#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import *
from smach_ros import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
'''
Author: your name
Date: 2020-11-08 21:09:04
LastEditTime: 2020-11-10 10:59:35
LastEditors: Please set LastEditors
Description: In User Settings Edit
FilePath: /undefined/home/jacy/gazebo_test_ws/src/xm_smach/xm_smach/smach_lib/smach_common/HW.py
'''
class GetAngle(State):
    '''
    用于获取听音辨位获取的角度
    zuo +
    you -
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted'],
                        output_keys =['angle'])
        self.client = rospy.ServiceProxy('/mobile_base/SoundSource',SoundSource)
        
        self.sounce_point_pub = rospy.Publisher('sounce_point', PointStamped,queue_size=10)

        self.tf_listener = tf.TransformListener()
        self.PI = 3.1415926535
        self.angle =0.0
        self.SoundSourcePoint = PointStamped()
        self.sounce_distance = 1.0
    def execute(self,userdata):
        try:
            self.client.wait_for_service(timeout=10.0)
            res = self.client.call("s")
            self.angle = res.angle * self.PI/180
            rospy.logwarn("***********************")
            rospy.logwarn(res.angle)
            rospy.logwarn("***********************")

            #display in rviz
            self.SoundSourcePoint.header.frame_id = 'listen_link'
            self.SoundSourcePoint.point.x = self.sounce_distance * math.cos(self.angle)
            self.SoundSourcePoint.point.y = - self.sounce_distance * math.sin(self.angle)
            for i in range(10):
                self.sounce_point_pub.publish(self.SoundSourcePoint)
                rospy.sleep(0.01)

            #get pos angle of the turn
            self.tf_listener.waitForTransform('base_link','listen_link',rospy.Time(),rospy.Duration(10.0))
            SoundSourceBasePoint = self.tf_listener.transformPoint('base_link',self.SoundSourcePoint)
            
            x = SoundSourceBasePoint.point.x
            y = - SoundSourceBasePoint.point.y
            #rospy.logwarn("***************")
            #rospy.logwarn(x)
            #rospy.logwarn(y)
            self.angle = abs( math.atan(y/x) )
            #print(self.angle * 180/self.PI)
             
            #向右前转
            if y > 0 and x > 0:
                self.angle =  self.angle 
            #向右后转
            elif y > 0 and x < 0:
                self.angle = self.PI - self.angle
            #向左前转
            elif y < 0 and x > 0:
                self.angle = - self.angle 
            #向左后转
            elif y < 0 and x < 0:
                self.angle = self.angle - self.PI
            userdata.angle = - self.angle
            rospy.logwarn("***********")
            rospy.logwarn(self.angle* 180/self.PI)
            return 'succeeded'
        except Exception, e:
            rospy.logerr(e)
            return 'aborted'

class CameraTurn(State):
    '''
    生成介绍的语句
    '''
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted','error'],
                        input_keys = ['angle'])
    def execute(self, userdata):
        try:
            
            return 'succeeded'
        except Exception, e:
            rospy.logerr(e)
            return 'aborted'