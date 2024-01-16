#!/usr/bin/env python
#coding=utf-8
'''
Author: jacy
Date: 2020-09-04 21:49:25
LastEditTime: 2020-09-11 12:02:49
LastEditors: Please set LastEditors
Description: get position
FilePath: /undefined/home/jacy/gazebo_test_ws/src/xm_smach/sim_interface/src/xm_arm_sim_hw.py
'''

import rospy, sys
import roslib
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
from xm_msgs.srv import *
from xm_msgs.msg import *
#for the scene
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene
#tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
'''
string object_name
int32 people_id
---
xm_msgs/xm_Object[] object
  int32 name
  geometry_msgs/PointStamped pos
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Point point
      float64 x
      float64 y
      float64 z
  int32 state

'''    

class GetPosition:
    def __init__(self):
        
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('get_position', anonymous = True)
        
        self.tf_listener = tf.TransformListener()

        rospy.Service('/get_position', xm_ObjectDetect, self.callback)

        # Use the planning scene object to add or remove objects
        self.scene = PlanningSceneInterface()

        rospy.loginfo("get_position has work")
        rospy.spin()

    def callback(self,req):

        object_name = req.object_name
        people_id = req.people_id

        res = xm_ObjectDetectResponse()
        res.object = list()

        if object_name == "person":
            #多人辨识项目
            
            if people_id == -1:
                #进行所有人的识别  
                for i in range(3):
                    #编号为i的person
                    person_i = xm_Object()
                    person_name = 'person_'+str(i)
                    
                    person_pose_pose = self.scene.get_object_poses([person_name])[person_name]
                    
                    person_pose = PoseStamped()
                    person_pose.header.frame_id = 'odom'
                    person_pose.pose = person_pose_pose

                    #get the kinect2_rgb_link
                    self.tf_listener.waitForTransform('kinect2_rgb_link','odom',rospy.Time(),rospy.Duration(10.0))
                    person_camera_pose = self.tf_listener.transformPose('kinect2_rgb_link',person_pose)

                    #match the xm_msgs/xm_Object type
                    person_i.name = i
                    person_i.pos.header.frame_id = 'kinect2_rgb_link'
                    person_i.pos.point.x = person_camera_pose.pose.position.x
                    person_i.pos.point.y = person_camera_pose.pose.position.y
                    person_i.pos.point.z = person_camera_pose.pose.position.z

                    res.object.append(person_i)
               
            else:
                #识别编号为people_id的人
                person = xm_Object()
                person_name = 'person_'+str(people_id)
                
                person_pose_pose = self.scene.get_object_poses([person_name])[person_name]
                
                person_pose = PoseStamped()
                person_pose.header.frame_id = 'odom'
                person_pose.pose = person_pose_pose

                #get the kinect2_rgb_link
                self.tf_listener.waitForTransform('kinect2_rgb_link','odom',rospy.Time(),rospy.Duration(10.0))
                person_camera_pose = self.tf_listener.transformPose('kinect2_rgb_link',person_pose)

                #match the xm_msgs/xm_Object type
                person.name = people_id
                person.pos.header.frame_id = 'kinect2_rgb_link'
                person.pos.point.x = person_camera_pose.pose.position.x
                person.pos.point.y = person_camera_pose.pose.position.y
                person.pos.point.z = person_camera_pose.pose.position.z

                res.object.append(person)
        
        else:
            if people_id == 0:
                #进行特定物体的识别
                target = xm_Object()
                target_name = object_name
                
                target_pose_pose = self.scene.get_object_poses([target_name])[target_name]
                
                target_pose = PoseStamped()
                target_pose.header.frame_id = 'odom'
                target_pose.pose = target_pose_pose

                #get the kinect2_rgb_link
                self.tf_listener.waitForTransform('kinect2_rgb_link','odom',rospy.Time(),rospy.Duration(10.0))
                target_camera_pose = self.tf_listener.transformPose('kinect2_rgb_link',target_pose)

                #match the xm_msgs/xm_Object type
                target.name = 0
                target.pos.header.frame_id = 'kinect2_rgb_link'
                target.pos.point.x = target_camera_pose.pose.position.x
                target.pos.point.y = target_camera_pose.pose.position.y
                target.pos.point.z = target_camera_pose.pose.position.z

                res.object.append(target)

            elif people_id == 1:
                #单个人的识别
                pass
            elif people_id == 2:
                #同时进行两个人的识别
                pass
            elif people_id == 3:
                #进行打电话的人的识别
                pass
            else:
                pass

        return res

if __name__ == '__main__':
    get_position = GetPosition()
    
    
