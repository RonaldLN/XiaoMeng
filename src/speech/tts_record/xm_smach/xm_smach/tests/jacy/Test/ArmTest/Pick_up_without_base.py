#!/usr/bin/env python
# encoding:utf8
'''
Author: jacy
Date: 2020-08-10 20:15:44
LastEditTime: 2020-11-09 17:32:06
LastEditors: Please set LastEditors
Description: 底盘不移动，联合摄像头进行机械臂抓取,假设已经移动到适合抓取的位置
FilePath: /undefined/home/jacy/catkin_ws/src/xm_smach/tests/jacy/ArmTest/Pick_up_without_base.py
'''

import rospy
from smach import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import subprocess
from smach_ros import *
import actionlib
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
from smach_common.common import *
"""
class GetObjectPosition(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target_name'],
                       output_keys=['target_camera_point' ,'object_state','table_depth'])
        self.xm_findobject = rospy.ServiceProxy('/get_position', xm_ObjectDetect)


    def execute(self, userdata):
        goal = Point()
        try:
            name =str( userdata.target_name)
            name.replace(' ' , '' , name.count(' '))    
        except Exception ,e:
            rospy.logerr(e)
            rospy.logerr('No param specified')
            return 'error'

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)

            req = xm_ObjectDetectRequest()
            req.object_name = name
            req.people_id = 0
            rospy.logwarn(name)
            rospy.logwarn(req)
            
            res = self.xm_findobject.call(req)
               
            if len(res.object) != 0:
                rospy.loginfo("find object!")
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            return 'aborted'

        if res.object[0].pos.point.x == -10.000 or res.object[0].pos.point.x == 10.000 or res.object[0].pos.point.x == 5:
            rospy.logerr('find nothing')
            return 'aborted'


        rospy.logwarn(res.object[0])
        userdata.target_name = name
        userdata.target_camera_point = res.object[0].pos
        
        return 'succeeded'

class PickJusufy(State):
    '''
    -修正图像传过来的物体坐标等信息
    -in:target_camera_point
    -out:target_camera_point
    -outcome:
        -succeeded
        -error
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','error'],
                        input_keys= ['target_name'],
                        io_keys =['target_camera_point','target_size','table_depth'])
    def execute(self,userdata):
        try:
            getattr(userdata, 'target_camera_point')
        except:
            rospy.logerr('no param')
            return 'error'
        else :
            userdata.target_size = [0.04, 0.065, 0.105]
            userdata.table_depth = 0.04
            name = userdata.target_name
            target_camera_point = userdata.target_camera_point
            if name =='water':
                userdata.target_camera_point.point.y -= 0.08 
                userdata.target_camera_point.point.x -= 0.0 
            else:
                userdata.target_camera_point = target_camera_point
            rospy.logwarn(userdata.target_camera_point)
            return 'succeeded'

class ArmStack(State):
    '''
    ArmStack
    '''
    def __init__(self):
        State.__init__(self,
                        outcomes=['succeeded','aborted' ,'error'],
                       input_keys=['target_camera_point','target_size','table_depth'])
        self.arm_stack_client = actionlib.SimpleActionClient("/xm_arm/arm_stack", xm_ArmStackAction)
    def execute(self,userdata):
        try:
            
            goal = xm_ArmStackGoal()
            goal.target_camera_point  = userdata.target_camera_point
            goal.target_size.l = userdata.target_size[0]
            goal.target_size.w = userdata.target_size[1]
            goal.target_size.h = userdata.target_size[2]
            goal.table_depth = userdata.table_depth
    
            
            self.arm_stack_client.wait_for_server(rospy.Duration(60.0))
            
            self.arm_stack_client.cancel_all_goals()
            rospy.logwarn("send the goal")
            self.arm_stack_client.send_goal(goal)

            self.arm_stack_client.wait_for_result(rospy.Duration.from_sec(60.0))

            if self.arm_stack_client.get_state() == 3:
                return 'succeeded'
            else:
                rospy.logerr("arm stack ik failed!")
                return 'aborted'
        except Exception ,e:
            rospy.logerr('Arm Stack have not work!')
            rospy.logerr(e)
            return 'error'
        else:
            return 'succeeded'
"""  
class ArmStack(State):
    '''
    ArmStack
    '''
    def __init__(self):
        State.__init__(self,
                        outcomes=['succeeded','aborted' ,'error'],
                       input_keys=['target_camera_point','target_size','table_depth'])
        self.arm_stack_client = actionlib.SimpleActionClient("/xm_arm/arm_stack", xm_ArmStackAction)
    def execute(self,userdata):
        try:
            
            goal = xm_ArmStackGoal()
            goal.target_camera_point  = userdata.target_camera_point
            goal.target_size.l = userdata.target_size[0]
            goal.target_size.w = userdata.target_size[1]
            goal.target_size.h = userdata.target_size[2]
            goal.table_depth = userdata.table_depth
    
            
            self.arm_stack_client.wait_for_server(rospy.Duration(60.0))
            
            self.arm_stack_client.cancel_all_goals()
            rospy.logwarn("send the goal")
            self.arm_stack_client.send_goal(goal)

            self.arm_stack_client.wait_for_result(rospy.Duration.from_sec(60.0))

            if self.arm_stack_client.get_state() == 3:
                return 'succeeded'
            else:
                rospy.logerr("arm stack ik failed!")
                return 'aborted'
        except Exception ,e:
            rospy.logerr('Arm Stack have not work!')
            rospy.logerr(e)
            return 'error'
        else:
            return 'succeeded'           

class Pick_up():
    def __init__(self):
        rospy.init_node('Pick_up_without_move')
        rospy.on_shutdown(self.shutdown)
        self.smach_bool = False
        

        self.Pick = StateMachine(outcomes =['succeeded','aborted','error'])
        
        with self.Pick:
            self.Pick.userdata.target_name ='cola'  
            self.Pick.userdata.table_depth = 0.04
            self.Pick.userdata.traget_size = [0.04, 0.065, 0.105]          
            StateMachine.add('FIND',GetObjectPosition(),
                             transitions={'succeeded': 'TARGET_POINT_JUSFY','aborted': 'aborted', 'error': 'error'})
            
            StateMachine.add('TARGET_POINT_JUSFY',TargerPosReload(),
                             transitions={'succeeded': 'PICK', 'error': 'error'})

            StateMachine.add('PICK', ArmStack(),
                             transitions={'succeeded': 'succeeded','error': 'error'})
            

        intro_server = IntrospectionServer('sm_Pick' , self.Pick , '/SM_ROOT')
        intro_server.start()
        out = self.Pick.execute()
        print out
        intro_server.stop()
        self.smach_bool =True

    def shutdown(self):
        if self.smach_bool == True:
            rospy.logwarn("DONE")
        else:
            rospy.logwarn('FUCK THE ERROE')

if __name__ == '__main__':
    try:
	    Pick_up()
    except Exception , e:
        rospy.logerr(e)
