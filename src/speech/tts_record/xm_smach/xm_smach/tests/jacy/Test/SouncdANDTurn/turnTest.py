#!/usr/bin/env python
# encoding:utf8

import rospy
from xm_msgs.msg import *
from xm_msgs.srv import *
from smach import *
from smach_ros import *
from subprocess import *
from geometry_msgs.msg import *
#from xm_smach.gpsr_lib import *
#from xm_smach.common_lib import *
#from xm_smach.speech_reco_lib import *

from smach_common.speech import *
from smach_common.nav import *
from xm_smach.target_gpsr import gpsr_target
import math
import linecache
import datetime
import tf   

PI = 3.1415926535

class CheckTurn(State):
    '''
    
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','continue','error'],
                        io_keys =['angle_list'],
                        output_keys=['angle'])

    def execute(self,userdata):
        try:
            getattr(userdata, 'angle_list')
        except:
            rospy.logerr('no param')
            return 'error'
        else :  
            try:
                angle_  =input()
                angle_ = angle_ * 3.1415926535/180
                userdata.angle = float(angle_)
                return 'continue'
            except:
                return 'error'
            else:
                return 'error'

class CheckTurn1(State):
    '''
    
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','continue','error'],
                        output_keys=['pos'])
        self.tf_listener = tf.TransformListener()
    def execute(self,userdata):
        try:
            angle_  =input()
            angle_ = float(angle_)* 3.1415926535/180
            
            #从角度得到四元数
            qs = QuaternionStamped()
            qs.header.frame_id = 'base_link'
            qs.quaternion = Quaternion(*quaternion_from_euler(0,0,angle_))
            
            #等待tf的信息
            self.tf_listener.waitForTransform('map', 'base_link',rospy.Time(), rospy.Duration(1))
        
            qs =self.tf_listener.transformQuaternion('map',qs)
            userdata.pos = Pose(Point(),qs.quaternion)

            return 'continue'
        except Exception , e:
            rospy.logerr(e)
            return 'error'
        else:
            return 'error'

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
            rospy.loginfo("the degree received is:"+str(userdata.degree))
            goal_angle = userdata.degree
            # 1 radis/s
            angular_speed = goal_angle/abs(goal_angle)

            self.move_cmd = Twist()
            self.move_cmd.linear.x = 0.0
            self.move_cmd.linear.y = 0.0
            self.move_cmd.linear.z = 0.0
            self.move_cmd.angular.x = 0.0
            self.move_cmd.angular.y = 0.0
            self.move_cmd.angular.z = angular_speed

            #需要的时间
            angular_duration = abs(goal_angle/angular_speed)
            # 发布频率
            rate = 50
            r = rospy.Rate(rate)
            
            try:
                print((2 * 3.1415926535 - goal_angle) * 0.1)
                ticks = int( angular_duration*rate+ (60 * 3.1415926535 - goal_angle) * 0.1 ) 
                for i in range(ticks):
                    self.cmd_vel.publish(self.move_cmd)
                    r.sleep()
                rospy.sleep(0.3)
                #发送一个空的消息，让机器人停下来
                self.move_cmd = Twist()
                self.cmd_vel.publish(self.move_cmd)
                rospy.sleep(0.3)
                return 'succeeded'
            except:
                rospy.logerr('meet question when publish Twist')
                return 'aborted'


'''
为了测试，将GetQuestion()改为调用7服务
'''
class SpeechRecognition():
    def __init__(self):
        rospy.init_node('speech_recognition')
        rospy.on_shutdown(self.shutdown)
        rospy.logwarn('Welcome to SpeechRecognition!!!')
        self.smach_bool = False


        #顶层状态机
        self.SPEECHRECOGNITION = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.SPEECHRECOGNITION:
            self.SPEECHRECOGNITION.userdata.sentences = 'start turn now!'
            self.SPEECHRECOGNITION.userdata.num = 1
            self.SPEECHRECOGNITION.userdata.angle_list = [PI/6]
            self.SPEECHRECOGNITION.userdata.question = ""
            StateMachine.add('START',
                                    Speak(),
                                    transitions={'succeeded':'CHECKTURN','aborted':'aborted','error':'error'},
                                    remapping = {'sentences':'sentences'}
                                    )   
            """
            StateMachine.add('CHECKTURN',
                                        CheckTurn(),
                                        transitions={'succeeded':'succeeded','continue':'TURN','error':'error'},
                                        remapping = {'angle':'angle'}
                                        )
            StateMachine.add('TURN',
                                        TurnDegree(),
                                        transitions={'succeeded':'START','aborted':'aborted','error':'error'},
                                        remapping = {'degree':'angle'}
                                        )                           
            """
            StateMachine.add('CHECKTURN',
                                        CheckTurn1(),
                                        transitions={'succeeded':'succeeded','continue':'TURN','error':'error'},
                                        remapping = {'pos':'pos'}
                                        )
            StateMachine.add('TURN',
                                        NavStack(),
                                        transitions={'succeeded':'START','aborted':'aborted','error':'error'},
                                        remapping = {'pos_xm':'pos'}
                                        )             
        intro_server = IntrospectionServer('SPEECHRECOGNITION',self.SPEECHRECOGNITION,'/SPEECHRECOGNITION')
        intro_server.start()
        out = self.SPEECHRECOGNITION.execute()
        intro_server.stop()

    def shutdown(self):  
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

if __name__ == "__main__":
    try:
        SpeechRecognition()
    except:
        rospy.logerr('test wrong!')



