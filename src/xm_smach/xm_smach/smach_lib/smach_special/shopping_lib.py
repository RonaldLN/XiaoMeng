#!/usr/bin/env python
# encoding:utf8
# this module only contants the simple states usually used, the smach_ros we will directly use in the scripts
# userdata of smach-container includes the whole userdata interface ,you can achieve the different interface by try-except function
# no param specified error should be raise by state itself so it is easy for us to debug
from matplotlib import use
import rospy
from smach import State,UserData,StateMachine
from smach_ros import SimpleActionState, ServiceState, MonitorState
from xm_msgs.srv import *
from pydub import AudioSegment#音频预处理
from pydub.playback import play#用于播放音频
#import xm_speech.srv
from xm_msgs.msg import *
from geometry_msgs.msg import *
from time import sleep
from math import *
import tf 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import subprocess
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from std_msgs.msg import Bool,Header
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from xm_smach.target_gpsr import gpsr_target
from math import *
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalStatus 
from time import sleep
from std_msgs.msg import String,Int32,Bool,Header
from std_srvs.srv import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import *
from control_msgs.msg import *
import subprocess
import math  
from speech.srv import *
from copy import deepcopy

def get_pid(name):
    pid = ''
    with open("/home/xm/vision_pid/{}.txt".format(name)) as f:
        pid = f.read()
    return pid

class CheckStop2(State):
    def __init__(self):
        State.__init__(self,
                        outcomes = ['remeber','stop','aborted'],
                        input_keys = ['PT_LIST','mission','task'],
                        output_keys= ['PT_LIST','mission','task'])
    
        # self.target_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        self.tf_listener = tf.TransformListener()
        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)

    def execute(self,userdata):
        # 任务数量
        num=3
        try:
            while not rospy.is_shutdown():
                self.speak_client.wait_for_service(timeout = 10)
                self.response = self.speak_client.call(command = 5)
                self.action = self.response.action
                self.object = self.response.object
                # left还是right
                self.target = self.response.gesture
                rospy.logwarn(self.action)
                rospy.logerr(self.object)
                rospy.logwarn("len"+str(len(userdata.PT_LIST)))

                rospy.logerr("!!!!!!! Object Length ======== ")
                rospy.logerr(len(self.object))

                if len(self.object) == 1 :
                    # subprocess.call("touch /home/ye/Recognition/kinect2/dummy_excution_final &" , shell = True)
                    rospy.logerr("******* ENTER **********")

                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')

                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))

                    # if len(self.target)!= 0:
                    #     rospy.logerr("target")
                    #     rospy.logerr(self.target)
                    #     eulerVec = euler_from_quaternion(rot)
                    #     rot = quaternion_from_euler(eulerVec[0] , eulerVec[1] , eulerVec[2])
                            
                    
                    userdata.PT_LIST[str(self.object[0])] = Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    userdata.task.append(str(self.object[0]))
                    rospy.logwarn(userdata.PT_LIST)
                    self.speak_sentence = 'Here is the ' + self.object[0]
            

                    self.speak_client.wait_for_service(timeout=10.0)
                    self.speak_client.call(2, self.speak_sentence)
                    return 'remeber'

                # 引导阶段结束，下一状态即打开摄像头寻找物品
                elif len(self.action)>0 and self.action[0] == 'stop' and len(userdata.PT_LIST)>=num:
                    rospy.logwarn('I will stop!!')
                    rospy.logwarn('go shopping!!!!!!!!!!')
                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')
                    self.mission = {}
                    for obj in self.object:
                        self.mission[obj] = userdata.PT_LIST[obj]
                    userdata.mission = self.mission
                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
                    # 收银台位置
                    userdata.PT_LIST['cashier'] =  Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    rospy.logerr(userdata.PT_LIST)

                    pid = get_pid("people_tracking")
                    subprocess.Popen(['kill','-9',pid],shell=False)
                    with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                        f.write('')
                    print("sleepiiiiiiiiing!!!!!!!!!!!")

                    return 'stop'
                elif len(self.object) == 3:
                    rospy.logwarn('go shopping!!!!!!!!!!')
                    self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))
                    rospy.logwarn('wait for tf succeeded')
                    self.mission = {}
                    for obj in self.object:
                        self.mission[obj] = userdata.PT_LIST[obj]
                    userdata.mission = self.mission
                    (trans,rot) = self.tf_listener.lookupTransform('/map','/base_link',rospy.Time(0))
                    
                    userdata.PT_LIST['cashier'] =  Pose(Point(trans[0],trans[1],trans[2]),Quaternion(rot[0],rot[1],rot[2],rot[3]))
                    rospy.logerr(userdata.PT_LIST)
                    return 'stop'
                # else:
                #     rospy.logwarn("FUCK :")
                #     return 'remeber'
        except Exception,e:
            rospy.logerr('xm meet wrong when get the target')
            rospy.logerr(e)
            return 'aborted'
        
# 运行跟随人的节点
class RunNode(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_vison_publisher.py','-d','true'] ,shell =False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w+') as f:
                if f.read() == '':
                    print('-------------------')
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('-------------------')
                    f.write(str(a.pid))
            rospy.sleep(2.0)
            # a.wait()
            print a.poll()
            if a.returncode != None:
                a.wait()
                return 'aborted'
        except Exception as e:
            print('exception:',e)
            rospy.logerr('people_tracking node error')
            return 'aborted'
        return 'succeeded'


class GetSignal(State):
    def __init__(self):
        State.__init__(self,outcomes = ['succeeded','aborted'])

        #创建speech_core服务的客户端对象
        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)

    def execute(self,userdata):
        try:
            #等待speech_core服务的可用性，超时时间为10s
            self.speak_client.wait_for_service(timeout = 10)
            #发送一个命令，等待响应
            self.response = self.speak_client.call(command = 1)
            #输出响应中的action数组
            rospy.logwarn(self.response.action)
            # 返回的action数组全名是什么？
            if self.response.action[0] == 'follow':
                return 'succeeded'
            else:
                return 'aborted'
        except Exception,e:
            rospy.logerr('xm meet wrong when get follow signal')
            rospy.logerr(e)
            return 'aborted'

# 延续(苟)一段时间的简单状态,rec为延续(苟)的时间
class Wait(State):
    def __init__(self):
        State.__init__(self, 
        outcomes=["succeeded",'error'],
        input_keys=['rec'])

    def execute(self, userdata):
        try:
            self.rec = userdata.rec
        except:
            rospy.logerr('no param specified')
            return 'error'
        else:  
            rospy.sleep(userdata.rec)
            return "succeeded"

class Wait_trace(State):
    def __init__(self):
        State.__init__(self, 
        outcomes=["succeeded",'error','preempted'])

    def execute(self, userdata):
        for i in range(0,20):
            if self.preempt_requested():
                return 'preempted'
            else:
                rospy.sleep(0.1)
        return "succeeded"


class ArmTrajectory_Point_Before(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded', 'error'],input_keys=['arm_waypoints'])
        self.arm_client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    def execute(self,userdata):
        
        trajectory = JointTrajectory()
        joint_names = ["arm_joint_0", "arm_joint_1", "arm_joint_2","arm_joint_3", "arm_joint_4", "arm_joint_5"]	
        trajectory.joint_names = joint_names

        pos_demo = JointTrajectoryPoint()
        pos_demo.positions = [0,-1.4,1.67,0,0.24,0]######
        pos_demo.velocities = [0.0 for i in joint_names]
        pos_demo.accelerations = [0.0 for i in joint_names]
        pos_demo.time_from_start = rospy.Duration(5.0)

        arm_waypoints = userdata.arm_waypoints
        for i in range( len(arm_waypoints) ):
            pos = deepcopy(pos_demo)
            pos.positions = arm_waypoints[i]
            trajectory.points.append(pos)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result(rospy.Duration(60.0))

        return 'succeeded'


class ArmTrajectory_Point_After(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded', 'error'],input_keys=['arm_waypoints'])
        self.arm_client = actionlib.SimpleActionClient("/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
    def execute(self,userdata):
        
        trajectory = JointTrajectory()
        joint_names = ["arm_joint_0", "arm_joint_1", "arm_joint_2","arm_joint_3", "arm_joint_4", "arm_joint_5"]	
        trajectory.joint_names = joint_names

        pos_demo = JointTrajectoryPoint()
        pos_demo.positions = [0,-1.47,3.14,0,0,0]######
        pos_demo.velocities = [0.0 for i in joint_names]
        pos_demo.accelerations = [0.0 for i in joint_names]
        pos_demo.time_from_start = rospy.Duration(5.0)

        arm_waypoints = userdata.arm_waypoints
        for i in range( len(arm_waypoints) ):
            pos = deepcopy(pos_demo)
            pos.positions = arm_waypoints[i]
            trajectory.points.append(pos)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)

        self.arm_client.send_goal(goal)
        self.arm_client.wait_for_result(rospy.Duration(60.0))

        return 'succeeded'



class Speak_Shopping(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['sentences','name'])
        #self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
        #rospy.wait_for_service('speech_core')
        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)
        

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            self.string_ = str(userdata.sentences) + str(userdata.name)#返回对象的字符串形式
            rospy.logwarn(self.string_)
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                # subprocess - Subprocesses with accessible I/O streams
                # 创建一个子进程,父进程等待子进程完成,返回退出信息
                # 让xm在子进程中说出userdata.sentences的语句，不影响父进程
                if(userdata.sentences == 'I have arrived here'):
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/gpsr_arrive.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'please ask me'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/please_ask_me.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'which luggage you want'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/which_luggage.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'Please hang the bag up to the claw'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/hang_up_luggage.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'I find you'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/find_you.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'Now I will follow you and help you carry the luggage.We can go to the car.'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/start_follow.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'Please take your luggage out of my claw'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/finish_carry.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'i will release the gripper,please grap it'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/release_object.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'i will open the door ,please be attention to the door.'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/open_the_door.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'sorry, can you say it again'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/say_it_again.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'please follow me.'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/follow_me.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'please seat in the left sofa.'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/seat_left.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'please seat in the center left sofa.'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/seat_center_left.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'please seat in the center right sofa.'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/seat_center_right.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == 'please seat in the rigt sofa.'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/seat_right.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
                elif(userdata.sentences == ' I find you,and I will introduce that guy'):
                    #rospy.logwarn("!!!!!!!!!!!!!")
                    record=AudioSegment.from_wav("/home/xm/catkin_ws/src/speech/smach_tts_record/introduce.wav")
                    play(record)
                    rospy.sleep(3.0)
                    return 'succeeded'
      
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, self.string_)

                #rospy.sleep(2.0)
                return 'succeeded'
                # speech_bool = self.speak_client.call(self.string_)
                # if speech_bool.flag == 1:
                #     subprocess.call(["play", "tts_sample.wav"])
                # elif speech_bool.flag == 0:
                #     subprocess.call(["espeak","-v","f3+en_us","-s","130",str(self.string_)])
                # #sim
                # elif speech_bool.flag ==2:
                    
                #     return 'succeeded'
                # else:
                #     rospy.logerr('the response error')
                #     return 'error'
            except:
                return 'aborted'
            else:
                return 'succeeded'



class GetPutPosition(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       output_keys=['target_camera_point'])
        self.xm_findobject = rospy.ServiceProxy('get_position', xm_ObjectDetect)


    def execute(self, userdata):
        try:
        #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        #     rospy.sleep(2.0)
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/basket_find.py','-d','true'] ,shell =False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w+') as f:
                if f.read() == '':
                    print('-------------------')
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('-------------------')
                    f.write(str(a.pid))
            rospy.sleep(2.0)
            print a.poll()
            if a.returncode != None:
                a.wait()
                return 'aborted'
        except:
             rospy.logerr('No param specified')
             return 'error'
        
        try:
            self.xm_findobject.wait_for_service(timeout=30.0)

            req = xm_ObjectDetectRequest()
            req.object_name = 'basket'
            req.people_id = 0
            rospy.logwarn(req)
            
            res = self.xm_findobject.call(req)
               
            if res.object.pos.point.x != 0 or res.object.pos.point.y != 0 or res.object.pos.point.z != 0:
                rospy.loginfo("find object!")
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        if res.object.pos.point.x == -10.000 or res.object.pos.point.x == 10.000 or res.object.pos.point.x == 5:
            rospy.logerr('find nothing')
            self.killPro()
            return 'aborted'


        rospy.logwarn(res.object)
        userdata.target_camera_point = res.object.pos
        self.killPro()
        return 'succeeded'

    def killPro(self):
        try:
            # pid_str = subprocess.check_output('ps -aux | grep mrsupw_detect_object.py' , shell= True)
            # pid_str1 = pid_str.splitlines()[0].split()[1]
            # rospy.logwarn(pid_str1)
            # subprocess.call('kill -9 '+pid_str1 , shell = True)

            pid = get_pid("people_tracking")
            subprocess.Popen(['kill','-9',pid],shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                f.write('')
                
        except Exception,e:
            rospy.logerr('No such process ')


class CheckTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','continue','error'],
                        input_keys=['task','turn','PT_LIST'],
                        io_keys=['cashier_pos', 'current_turn'])
    def execute(self,userdata):
        try:
            self.task = userdata.task
            self.turn = userdata.turn
            userdata.current_turn+=1
            rospy.logwarn(self.task)
            rospy.logwarn(self.turn)
            rospy.logwarn(userdata.current_turn)

        except Exception,e:
            rospy.logerr(e)
            return 'error'
        if userdata.current_turn >= self.turn:
            rospy.logwarn('xm finish the turn')
            return 'succeeded'
        else:
            rospy.logwarn('finish one turn')
            rospy.logwarn(userdata.PT_LIST)
            userdata.cashier_pos = userdata.PT_LIST['cashier']
            return 'continue'
       
class Pos_Find(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','error'],
                        input_keys=['PT_LIST','task','current_turn'],
                        io_keys=['present_pos','pick_sentences'])
    def execute(self,userdata):
        try:
            userdata.present_pos=userdata.PT_LIST[userdata.task[userdata.current_turn]]
            rospy.logwarn(userdata.present_pos)
            
        except Exception,e:
            rospy.logerr(e)
            return 'error'
        return 'succeeded'

class PresentTask(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','error'],
                        input_keys=['task','current_turn','find_sentences'],
                        io_keys=['present_task','pick_sentences'])
    def execute(self,userdata):
        try:
            userdata.present_task= userdata.task[userdata.current_turn]
            print("present_task:  ",userdata.present_task)
            userdata.pick_sentences=userdata.find_sentences+userdata.present_task
        except Exception,e:
            rospy.logerr(e)
            return 'error'
        return 'succeeded'


class GetPickPosBack_shopping(State):
    '''
    no use
    根据图像传过来的数据,确定抓取时xm的位置
    up_down变量为1,则使用上面的摄像头;up_down变量为0,则使用下面的摄像头。
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','error'],
                        input_keys= ['distance','up_down'],
                        output_keys =['pick_pos'],
                        io_keys=['target_camera_point'])
        self.tf_listener = tf.TransformListener()
    def execute(self,userdata):
        try:
            getattr(userdata, 'distance')
            getattr(userdata, 'target_camera_point')
        except:
            rospy.logerr('no param')
            return 'error'
        else :
            if userdata.up_down == 0:
                camera_link = "camera_link_down"
            else :
                camera_link = "camera_link_up"
            target_camera_point = userdata.target_camera_point
            target_camera_point.header.frame_id = camera_link
            print("target_camera_point:",target_camera_point)
            self.tf_listener.waitForTransform('base_link',camera_link,rospy.Time.now(),rospy.Duration(4.0))
            # print(55435)
            target_base_point = self.tf_listener.transformPoint('base_link',target_camera_point)
            #                           y
            #               ————————————————————————————
            #               |                       |
            #               |                   |
            #               |               |
            #          x    |  alpha    |
            #               | ~~~   |
            #               |   |
            #               |
            print("target_base_point:",target_base_point)
            x = target_base_point.point.x
            y = target_base_point.point.y

            distance = math.sqrt(x**2+y**2)
            alpha =  math.atan(y/x)
            #  状态机的userdata.distance 需要确定
            move_in_line = distance - 0.5
            
            print("distance is:"+str(distance))
            print("move_in_line is:"+str(move_in_line))

            pick_base_point = PointStamped()
            pick_base_point.header.frame_id = 'base_link'
            pick_base_point.point.x = move_in_line*math.cos(alpha)
            pick_base_point.point.y = move_in_line*math.sin(alpha) 

            target_camera_point_new = PointStamped()
            target_camera_point_new.header.frame_id = 'base_link'
            target_camera_point_new.point.x = (distance-move_in_line)*math.cos(alpha)
            target_camera_point_new.point.y = (distance-move_in_line)*math.sin(alpha) 
            target_camera_point_new.point.z = target_base_point.point.z


            self.tf_listener.waitForTransform(camera_link,'base_link',rospy.Time(),rospy.Duration(10.0))
            userdata.target_camera_point = self.tf_listener.transformPoint(camera_link,target_camera_point_new)
            print('target_camera_point new',userdata.target_camera_point)
            
            qs = QuaternionStamped()
            qs.header.frame_id = 'base_link'
            qs.quaternion = Quaternion(*quaternion_from_euler(0,0,alpha))
            
            self.tf_listener.waitForTransform('map','base_link',rospy.Time.now(),rospy.Duration(60.0))    
            pick_base_point =self.tf_listener.transformPoint('map',pick_base_point)
            
            qs =self.tf_listener.transformQuaternion('map',qs)
            userdata.pick_pos = Pose(pick_base_point.point,qs.quaternion)

            return 'succeeded'


class GetPickPosBack_shopping(State):
    '''
    no use
    根据图像传过来的数据，确定抓取时xm的位置
    up_down变量为1，则使用上面的摄像头；up_down变量为0，则使用下面的摄像头。
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','error'],
                        input_keys= ['distance','up_down'],
                        output_keys =['pick_pos'],
                        io_keys=['target_camera_point'])
        self.tf_listener = tf.TransformListener()
    def execute(self,userdata):
        try:
            getattr(userdata, 'distance')
            getattr(userdata, 'target_camera_point')
        except:
            rospy.logerr('no param')
            return 'error'
            return 'succeeded'

class GetPickPosBack_shopping_putdown(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['target_camera_point'])
        self.cmd_vel = rospy.Publisher(
            '/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
        try:
            move_len=userdata.target_camera_point.point.x-0.15
            self.turn = Twist()
            self.turn.linear.x = 0.2 * int(abs(move_len) / move_len)
            self.turn.linear.y = 0.0
            self.turn.linear.z = 0.0
            self.turn.angular.x = 0.0
            self.turn.angular.y = 0.0
            self.turn.angular.z = 0.0

            angular_duration = abs(move_len / 0.2)
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


class TurnDegree_Shop(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['order','current_turn'])
        self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    def execute(self, userdata):
            degree=0
            if userdata.order[userdata.current_turn] == 'left':
                degree=0.5
            elif userdata.order[userdata.current_turn] =='right' :
                degree=-0.5
                
            rospy.loginfo("the degree received is:" + str(degree*57))
            if degree == 0 :
                return 'succeeded'
            goal_angle = degree
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



class FindtheSpeak(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['find_sentences', 'current_object_name'])

        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            self.string = str(userdata.find_sentences)+str(userdata.current_object_name)
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, self.string)
                rospy.sleep(1.0)
                return 'succeeded'

            except:
                return 'aborted'


class TryFindObject1(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted','giveup','next', 'error'],
                       input_keys=['task'],
                       output_keys=['target_camera_point'],
                       io_keys=['current_object_name'])
        self.xm_findobject = rospy.ServiceProxy('get_3position', xm_3ObjectDetect)

    def execute(self, userdata):
        try:
            print(userdata.task[0])
            # a = subprocess.Popen(['python3.9','/home/xm/catkin_ws/src/xm_vision/src/scripts/new/test.py','-d','True','-t','Chip'] ,shell =False)

            a = subprocess.Popen(['python3.9','/home/xm/catkin_ws/src/xm_vision/src/scripts/new/know_and_findplace_new.py','-d','True','-t','%s' %userdata.task[0]] ,shell =False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w+') as f:
                if f.read() == '':
                    print('-------------------')
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('-------------------')
                    f.write(str(a.pid))
            rospy.sleep(2.0)
            print a.poll()
            if a.returncode != None:
                a.wait()
                self.killPro()
                return 'aborted'
        except:
             rospy.logerr('No param specified')
             self.killPro()
             return 'error'

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)
            req = xm_3ObjectDetectRequest()
            req.object_name1 = userdata.task[0]
            req.object_name2 = userdata.task[1]
            req.object_name3 = userdata.task[2]
            # rospy.logwarn(req)
            
            res = self.xm_findobject.call(req)
            rospy.logwarn(res)
            if res.pos.point.x != 0 or res.pos.point.y != 0 or res.pos.point.z != 0:
                rospy.loginfo("find object!")
            else:
                rospy.loginfo("can't find object!")
                self.killPro()
                return 'next'
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        # rospy.logwarn('1111111111111111111')
        userdata.current_object_name = res.object_name
        # userdata.target_camera_point = res.pos
        self.killPro()
        rospy.logwarn(userdata.current_object_name)
        return 'succeeded'

    def killPro(self):
        try:
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill','-9',pid],shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                f.write('')
                
        except Exception,e:
            rospy.logerr('No such process ')


class TryFindObject2(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted','giveup','next', 'error'],
                       input_keys=['task'],
                       output_keys=['target_camera_point'],
                       io_keys=['current_object_name'])
        self.xm_findobject = rospy.ServiceProxy('get_3position', xm_3ObjectDetect)

    def execute(self, userdata):
        try:
            print(userdata.task[1])
            a = subprocess.Popen(['python3.9','/home/xm/catkin_ws/src/xm_vision/src/scripts/new/know_and_findplace_new.py','-d','True','-t','%s' %userdata.task[1]] ,shell =False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w+') as f:
                if f.read() == '':
                    print('-------------------')
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('-------------------')
                    f.write(str(a.pid))
            rospy.sleep(2.0)
            print a.poll()
            if a.returncode != None:
                a.wait()
                self.killPro()
                return 'aborted'
        except:
             rospy.logerr('No param specified')
             self.killPro()
             return 'error'

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)
            req = xm_3ObjectDetectRequest()
            req.object_name1 = userdata.task[0]
            req.object_name2 = userdata.task[1]
            req.object_name3 = userdata.task[2]
            rospy.logwarn(req)
            
            res = self.xm_findobject.call(req)
            rospy.logwarn(res)
            if res.pos.point.x != 0 or res.pos.point.y != 0 or res.pos.point.z != 0:
                rospy.loginfo("find object!")
            else:
                rospy.loginfo("can't find object!")
                self.killPro()
                return 'next'
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        userdata.current_object_name = res.object_name
        userdata.target_camera_point = res.pos
        self.killPro()
        return 'succeeded'

    def killPro(self):
        try:
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill','-9',pid],shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                f.write('')
                
        except Exception,e:
            rospy.logerr('No such process ')


class TryFindObject3(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted','giveup','next', 'error'],
                       input_keys=['task'],
                       output_keys=['target_camera_point'],
                       io_keys=['current_object_name'])
        self.xm_findobject = rospy.ServiceProxy('get_3position', xm_3ObjectDetect)

    def execute(self, userdata):
        try:
            print(userdata.task[2])
            a = subprocess.Popen(['python3.9','/home/xm/catkin_ws/src/xm_vision/src/scripts/new/know_and_findplace_new.py','-d','True','-t','%s' %userdata.task[2]] ,shell =False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w+') as f:
                if f.read() == '':
                    print('-------------------')
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('pid',a.pid)
                    print('-------------------')
                    f.write(str(a.pid))
            rospy.sleep(2.0)
            print a.poll()
            if a.returncode != None:
                a.wait()
                self.killPro()
                return 'aborted'
        except:
             rospy.logerr('No param specified')
             self.killPro()
             return 'error'

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)
            req = xm_3ObjectDetectRequest()
            req.object_name1 = userdata.task[0]
            req.object_name2 = userdata.task[1]
            req.object_name3 = userdata.task[2]
            rospy.logwarn(req)
            
            res = self.xm_findobject.call(req)
            rospy.logwarn(res)
            if res.pos.point.x != 0 or res.pos.point.y != 0 or res.pos.point.z != 0:
                rospy.loginfo("find object!")
            else:
                rospy.loginfo("can't find object!")
                self.killPro()
                return 'next'
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        userdata.current_object_name = res.object_name
        userdata.target_camera_point = res.pos
        self.killPro()
        return 'succeeded'

    def killPro(self):
        try:
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill','-9',pid],shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                f.write('')
                
        except Exception,e:
            rospy.logerr('No such process ')