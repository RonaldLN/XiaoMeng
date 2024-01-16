#! /usr/bin/env python
# encoding:utf8

import rospy
from smach import *
from smach_ros import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import *
from math import pi,atan
import tf

# FGM: This State is expected to solve the problem that xm backs up outwards the door


class PickTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=[
                       'succeeded', 'aborted'],input_keys=['turn_pose'], output_keys=['turn_pose'])
        self.tf_Listener = tf.TransformListener()
    def execute(self, userdata):
        
        try:
            now = rospy.Time(0)
            self.tf_Listener.waitForTransform('map' , 'base_link' , now , rospy.Duration(1.0))
            turn_pose= self.tf_Listener.lookupTransform('map', 'base_link', now)
            rospy.logwarn('position-------')
            rospy.logerr(turn_pose[1])
            rospy.logwarn(turn_pose)
            quaternion1 = turn_pose[1][0]
            quaternion2 = turn_pose[1][1]
            quaternion3 = turn_pose[1][2]
            quaternion4 = turn_pose[1][3]
            angular = euler_from_quaternion([quaternion1, quaternion2, quaternion3, quaternion4])
            rospy.logwarn(angular)
            angle = angular[2] + pi*2/3
            rospy.logwarn(angle)
            quaternion = quaternion_from_euler(0, 0, angle)
            rospy.logwarn(quaternion)
            userdata.turn_pose.orientation = Quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3])
            userdata.turn_pose.position = Point(turn_pose[0][0],turn_pose[0][1],turn_pose[0][2])
            rospy.logwarn(userdata.turn_pose)
            return 'succeeded'
        except Exception , e:
            rospy.logerr(e)
            return 'aborted'


# FGM : This is a State to judge if we need turn
class IsTurn(State):
    def __init__(self):
        State.__init__(self , outcomes = ['yes' , 'no' , 'error'])
        self.tf_Listener = tf.TransformListener()

    def execute(self , userdata):
        try:
            now = rospy.Time(0)
            self.tf_Listener.waitForTransform('map' , 'base_link' , now , rospy.Duration(2.0))
            (point,orientation) = self.tf_Listener.lookupTransform('base_link' , 'map' , now)
            rospy.logerr(point)
            while not is_shutdown():
                now = rospy.Time(0)
                self.tf_Listener.waitForTransform('map' , 'base_link' , now , rospy.Duration(1.0))
                now_velocity = self.tf_Listener.lookupTwist('base_link' , 'map' , now,rospy.Duration(2.0))

                rospy.logwarn(now_velocity)

                if abs(now_velocity[0][1])+abs(now_velocity[0][0]) <= 0 :
                    continue

                quaternion1 = orientation[0]
                quaternion2 = orientation[1]
                quaternion3 = orientation[2]
                quaternion4 = orientation[3]        
                angular_xm = euler_from_quaternion([quaternion1,quaternion2,quaternion3,quaternion4])
                rospy.logwarn(angular_xm)

                velocity_an = atan(now_velocity[0][1]/now_velocity[0][0])
               
                rospy.logwarn(velocity_an)
                
                if(now_velocity[0][1]>0 and now_velocity[0][0]<0):
                    velocity_an += pi
                elif(now_velocity[0][1]<0 and now_velocity[0][0]<0):
                    velocity_an -= pi
                rospy.logwarn(velocity_an)
                deta = angular_xm[2] - velocity_an
		rospy.logwarn(deta)

                if( deta > pi/2 or deta < -pi/2):
                    return  'yes'
                else:
                    return 'no'
        
        except Exception , e:
            rospy.logerr(e)
            return 'error'


class NewNav():
    def __init__(self):
        self.new_nav = Concurrence(outcomes = ['succeeded','aboretd','error'],
                                    input_keys=['pos_xm'],
                                    default_outcome = 'succeeded',
                                    outcome_cb = self.nav_outcome_cb,
                                    child_termination_cb = self.nav_child_termination_cb)

        with self.new_nav:
            self.turn_back = StateMachine(outcomes = ['succeeded','aborted','error'])
            
            with self.turn_back:
                self.turn_back.userdata.nav_pos = Pose()
                StateMachine.add('ISTURN',
                IsTurn(),
                transitions={'yes':'PICKTURN' , 'no':'ISTURN','error':'error'},
                remapping={'pos_xm':'pos_xm'})
                StateMachine.add('PICKTURN',
                                PickTurn(),
                                transitions={'succeeded':'TURNGO','aborted':'ISTURN'},
                                remapping={'xm_pos':'pos_xm',
                                            'turn_pos':'nav_pos'} )

                StateMachine.add('TURNGO',NavStack(),
                                transitions={'succeeded':'ISTURN','aborted':'TURNGO','error':'error'},
                                remapping={'pos_xm':'turn_pos'})

            Concurrence.add('TURNBACK',self.turn_back,
                            remapping={'pos_xm':'pos_xm'})

            Concurrence.add('NAV',NavStack(),
                            remapping={'pos_xm':'pos_xm'})

            
    def nav_outcome_cb(self,outcome_map):
        if(outcome_map['NAV']=='succeeded'):
            return 'succeeded'
        elif(outcome_map['NAV'] == 'aborted'):
            return 'aborted'
        elif(outcome_map['TURNBACK']=='error'):
            return 'error'
        

    def nav_child_termination_cb(self,outcome_map):
        if(outcome_map['NAV'] == 'succeeded'):
            return True
        

        
        



