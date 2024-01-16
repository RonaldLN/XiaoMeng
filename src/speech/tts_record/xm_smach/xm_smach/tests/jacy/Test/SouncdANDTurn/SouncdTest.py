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



class CheckTurn(State):
    '''
    用于判断循环的一个状态
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','continue','error'],
                        io_keys =['num'])

    def execute(self,userdata):
        try:
            getattr(userdata, 'num')
        except:
            rospy.logerr('no param')
            return 'error'
        else :
            if userdata.num <=5:
                userdata.num +=1
                return 'continue'
            elif userdata.num ==6:
                return 'succeeded'
            else:
                return 'error'

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


class TurnDegree0(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['degree'])
        self.cmd_vel = rospy.Publisher(
            '/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
        try:
            getattr(userdata, 'degree')
        except:
            rospy.logerr('No param specified!')
            return 'error'

        else:
            # if self.cmd_vel.get_num_connections() == 0:
            #     rospy.logerr('This state did nothing!')
            #     return 'succeeded'
            if userdata.degree == 0:
                return 'succeeded'
            goal_angle = userdata.degree
            angular_speed = goal_angle/abs(goal_angle)
            rospy.logwarn(angular_speed)
            self.turn = Twist()
            self.turn.linear.x = 0.0
            self.turn.linear.y = 0.0
            self.turn.linear.z = 0.0
            self.turn.angular.x = 0.0
            self.turn.angular.y = 0.0
            self.turn.angular.z = 2*angular_speed

            angular_duration = goal_angle/angular_speed
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


class TurnDegree1(State):
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
                ticks = int( angular_duration*rate+ (2 * 3.1415926535 - goal_angle) * 0.1 ) 
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
            self.SPEECHRECOGNITION.userdata.sentences = 'We can start now!'
            self.SPEECHRECOGNITION.userdata.num = 1
            self.SPEECHRECOGNITION.userdata.question_list = list()
            self.SPEECHRECOGNITION.userdata.question = ""
            StateMachine.add('START',
                                    Speak(),
                                    transitions={'succeeded':'GETANGLE','aborted':'aborted','error':'error'},
                                    remapping = {'sentences':'sentences'}
                                    )   
            StateMachine.add('GETANGLE',
                                    GetAngle(),
                                    transitions={'succeeded':'TURN','aborted':'aborted'},
                                    remapping = {'angle':'angle'}
                                    )  
            StateMachine.add('TURN',
                                        TurnDegree1(),
                                        transitions={'succeeded':'CHECKTURN','aborted':'aborted','error':'error'},
                                        remapping = {'degree':'angle'}
                                        )                           
            StateMachine.add('CHECKTURN',
                                        CheckTurn(),
                                        transitions={'succeeded':'succeeded','continue':'START','error':'error'},
                                        remapping = {'num':'num'}
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



