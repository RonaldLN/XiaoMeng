#!/usr/bin/env python
# encoding:utf8
from pyparsing import CaselessKeyword
from xm_smach.target_gpsr import gpsr_target
import rospy
import tf
import actionlib
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
import subprocess
import math  
from speech.srv import *

'''
Author: yishui
Date: 2022-06-29
LastEditTime: 2022-06-29 10:10:24
LastEditors: yishui
Description: Codes for GPSR
FilePath: smach_lib\smach_special
'''

def get_pid(name):
    pid = ''
    with open('/home/xm/vision_pid/{}.txt'.format(name)) as f:
        pid = f.read()
    return  pid
    # return map(int,subprocess.check_output(["pidof",name]).split())

class NextDo(State):###跳转action
    def __init__(self):
        State.__init__(self, 
                        #nav_grasp is something interesting
                    outcomes=['succeeded','aborted','go','find','answer','follow','get','release','speak','guide','introduce','count','attend','error'],
                    input_keys =['action','task_num'],
                    io_keys =['current_task'])
        
    def execute(self, userdata):
        try:
            action = userdata.action
            current_task = userdata.current_task
            task_num = userdata.task_num
        except:
            rospy.logerr('No param specified')
            return 'error'
        userdata.current_task+=1

        # 测试运行成功
        # current_task目前测试的次数
        # task_num 测试总次数
        if userdata.current_task ==  task_num:
            return 'succeeded'
            
        current_action =  action[userdata.current_task]

        #-------test for find---------#
        # if current_action != 'find':
        #     return 'aborted'
        # else: return 'find'



        if current_action == 'go':
            return 'go'
        elif current_action == 'find':
            return 'find'
        elif current_action == 'follow':
            return 'follow'
        elif current_action == 'pick' or current_action == 'get' or current_action == 'take':
            return 'get'
        elif current_action == 'answer':
            return 'answer'
        elif current_action == 'place':
            return 'place'
        elif current_action == 'tell':
            return 'speak'
        elif current_action == 'deliver':
            return 'release'
        elif current_action == 'guide':
            return 'guide'
        elif current_action == 'introduce':
            return 'introduce'
        elif current_action == 'count':
            return 'count'
        elif current_action == 'attend':
            return 'attend'
        # somethin interest here
        # elif current_action == 'nav_grasp':
        #     return 'nav_grasp'
        # elif current_action == 'nav_grasp':
        #     return 'nav_grasp'
        # elif current_action == 'guide':
        #     return 'guide'
        else:
            # no avaiable action find
            # userdata.current_task_out -1
            userdata.current_task -=1
            return 'aborted'
  
#   function as the name of state -_-
#   在 人\说话者\物品\位置 中进行切换
class PersonOrPosition(State):### switch the goal among person , speaker, thing, position
    def __init__(self):
        State.__init__(self, 
                        outcomes=['calling','person','position'],
                        input_keys=['target','current_task'])
        
    def execute(self,userdata):
        self.target = userdata.target[userdata.current_task]
        self.target = self.target.lower()
        person_list = ['person','mary','alex','angel','edward','homer','jamie','john','kevin','kurt','tracy','robin','eve','jane','liza','melissa','sophia','james']
        if self.target in person_list:
            print("it is the person")
            return 'person'
        elif self.target == "calling_person":
            print("it is the calling")
            return 'calling'
        else :
            print("it is the position")
            return 'position'
            
# simple state used for get meaning from the speech node
# command mean invoke the speech node to return the meaning from the order
# 用于从对话中得到含义的简单状态
# 命令行是启动speech node来返回命令的含义
class GetTask(State):
    def __init__(self):
        State.__init__(self, 
                        outcomes=['succeeded','aborted','error'],
                        io_keys=['target','action','task_num','answer','gesture'])
                        
        # self.speech_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        #rospy.wait_for_service('speech_core')
        self.speech_client = rospy.ServiceProxy('speech_core', speech_to_smach)
                
    def execute(self,userdata):
        try:
            getattr(userdata, 'target')
            getattr(userdata, 'action')            
            getattr(userdata, 'task_num')
            getattr(userdata, 'answer')
            getattr(userdata, 'gesture')
        except:
            rospy.logerr('No param specified')
            return 'error'
        try:
            self.speech_client.wait_for_service(timeout=10)
        #command = 1用于gpsr
            response =self.speech_client.call(command = 1,text = 'www')
            print(response.num)
            print(response.action)
            print(response.object)
            print(response.gesture)
        except:
            rospy.logerr('wrong in call the service')
            return 'error'
        # if response.action[0] == 'stop':
        #     rospy.logerr('response wrong!')
        #     return 'aborted'

        # num任务个数
        # action动作序列go find get
        # target目标对象 drinks
        # answer回复的语音文本 一句话
        # gesture （机械臂）姿态 rasie their right arm 等
        if response.num > 0:
            userdata.task_num = response.num
            userdata.action = response.action
            userdata.target = response.object
            userdata.answer = response.answer
            userdata.gesture = response.gesture
            rospy.logwarn(userdata.task_num)
            rospy.logwarn(userdata.action)
            rospy.logwarn(userdata.target)
            rospy.logwarn(userdata.answer)
            rospy.logwarn(userdata.gesture)
            # if userdata.action[1] == 'grasp':
                # something interest here
                # userdata.action[0] = 'nav_grasp'
                # userdata.action.append('place')
                # userdata.task_num += 1
            return 'succeeded'
        else:
            return 'aborted'


# 用于得到特定目标信息的状态
# 当 mode == 0 的时候返回目标的名字
# 当 mode == 1 是返回目标的位置
class GetTarget(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                            input_keys =['target','current_task','mode'],
                            output_keys =['current_target'])
    def execute(self,userdata):
        try:#判断这些属性是否存在
            getattr(userdata,'target')
            getattr(userdata,'current_task')
            getattr(userdata,'mode')
        except:
            rospy.logerr('No params specified ')
            return 'error'
        if userdata.mode ==0:
            # due to the cv may need a different name-style from the speech_node
            # this should be modify
            # 由于cv可能需要一个来自speech_node的不同的name-style
            # 这个需要被修改
            userdata.current_target = userdata.target[userdata.current_task]#存目标名字
        elif userdata.mode ==1:
            rospy.logwarn(userdata.target)

            #if userdata.current_task == 'speaker':
            #     userdata.current_target = gpsr_target[userdata.target['speaker2']]['pos']
            # else:
            target0 =userdata.target[userdata.current_task]
            if target0 =='show' or target0=='shop':
                target0 = 'shelf'
            if target0 == 'living_room':
                target0 = 'livingroom'
            if target0 == 'dining_room':
                target0 = 'diningroom'
            print(target0)
            userdata.current_target = gpsr_target[target0]['pos']#存目标位置
            rospy.logwarn(userdata.current_task)
        else:
            return 'aborted'
        return 'succeeded'

class Judge(State):
    def __init__(self):
        State.__init__(self , outcomes = ['TURN' , 'NO'] , input_keys =['pos_xm'])
    
    def execute(self , userdata):
        if userdata.pos_xm == gpsr_target['speaker2']['pos']:
            return 'TURN'
        
        return 'NO'
	

# close the Kinect use a simple executable file
# 用一个可执行文件关闭关闭Kinect的状态
class CloseCamera(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    def execute(self,userdata):
        try:
            # subprocess.call("xterm -e touch /home/ye/Recognition/kinect2/dummy_excution_final &" , shell = True)
            pid = get_pid("people_tracking")
            subprocess.Popen(['kill','-9',pid],shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                        f.write('')
            rospy.sleep(1.0)
        except:
            return 'aborted'
        return 'succeeded'

class CloseKinect_img(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    def execute(self,userdata):
        try:
            
            subprocess.call("touch /home/ye/Recognition/kinect2/close_image_test &" , shell = True)

            # subprocess.call("xterm -e rosnode kill /object_detection &" , shell = True)
        except:
            return 'aborted'
        return 'succeeded'

# the state is used for invoking the speech service to get the stop-signal
# 这个状态用于请求speech service来得到停止信号
class StopFollow(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
        self.speech_client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        
    
    def execute(self,userdata):
        try:
            subprocess.call("xterm -e rosrun xm_speech xm_speech_client_demo.py &", shell = True)
        except:
            return 'aborted'
        
        self.speech_client.wait_for_service(timeout=10)
        res= self.speech_client.call(command=4)
        if res.num>0:
            return 'succeeded'

# 运行跟随人的节点
class RunNode(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_vison_publisher.py','-d','true'] ,shell =False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
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

class RunNode_img(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            a = subprocess.Popen('xterm -e rosrun xm_vision image_test &',shell =True)
        except:
            rospy.logerr('people_tracking node error')
            return 'aborted'
        return 'succeeded'

# get the detect Pose where can make the find-object task execute best
# 得到检测的位置能够使find-object测试运行得更好 
class GetPos(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted','error'],
                        input_keys =['target','current_task','mode'],
                        output_keys =['pose'])

    def execute(self,userdata):
        try:
            getattr(userdata,'target')
            getattr(userdata,'current_task')
            getattr(userdata,'mode')
        except:
            rospy.logerr('No params specified')
            return 'error'
        else:
            last_task = userdata.current_task -1
            last_target =str(userdata.target[last_task])
            #last_target = str(userdata.target[last_task]) +'_table' +'_' +str(userdata.mode)
            print last_target
            if last_target == 'living_room':
                last_target = 'livingroom'
            userdata.pose = gpsr_target[last_target]['pos']
            return 'succeeded'
            #这根本不会返回aborted吧
            if False:
                return 'aborted'

class StopByCount(State):
    def __init__(self):
        State.__init__(self,outcomes=['continue','stop','error'],
                       io_keys=['counter'],
                       input_keys=['count_num'])

    def execute(self, userdata):
        try:
            getattr(userdata,'counter')
        except:
            rospy.logerr('No param specified')
            return 'error'
        else:
            self_counter = userdata.counter
            self_count_num = userdata.count_num
            if self_counter >= self_count_num:
                userdata.counter = 0
                return 'stop'
            else:
                userdata.counter +=1
                return 'continue'

class CheckTurn(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','continue','error'],
                        io_keys=['current_turn','task_num','current_task'],
                        input_keys=['turn'])
    def execute(self,userdata):
        try:
            self.current_turn = userdata.current_turn
            self.turn = userdata.turn
            rospy.logwarn(self.current_turn)
            rospy.logwarn(self.turn)
        except Exception,e:
            rospy.logerr(e)
            return 'error'
        if self.current_turn >= self.turn:
            rospy.logwarn('xm finish the turn')
            return 'succeeded'
        else:
            rospy.logwarn('finish one turn')
            userdata.current_turn += 1
            userdata.task_num = 0
            userdata.current_task = -1
            return 'continue'

class PosCounter(State):
    def __init__(self):
        State.__init__(self , outcomes = ['succeeded' , 'continue' , 'aborted'] ,
                        io_keys = ['go_counter'])

    def execute(self , userdata):
        try:
            userdata.go_counter = userdata.go_counter -1

            if userdata.go_counter < 1:
                return 'succeeded'
            
            else :
                return 'continue'
        except Exception,e:
            return 'aborted'

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

class GetObjectNum(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['target','current_task'],
                       output_keys=['count_num'])
        #self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
        #rospy.wait_for_service('handGesture')
        # self.nav_client = actionlib.SimpleActionClient(
        #     "move_base", MoveBaseAction)

        
        # self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel',Twist,queue_size=1)
        # self.twist_xm = Twist()
        
        self.countObject_client = rospy.ServiceProxy('countObject', xm_count_object)
        

    def execute(self, userdata):
        try:
        #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        #     rospy.sleep(2.0)
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_count_object.py','-d','true'] ,shell =False)
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
        
        rospy.loginfo('.................Count Object ^_^..........\n')
        try:
            req = xm_count_objectRequest()
            # obj=str(userdata.target)
            # L=obj.split("_")
            # if "three_" in obj:
            #     req.name = "_".join(L[2:])
            #     req.adj = "three_"+L[1]
            # elif L[0] in ["biggest","largest","thinnest","smallest","heaviest","lightest"]:
            #     req.name = "_".join(L[1:])
            #     req.adj = L[0]
            req.des = userdata.target[userdata.current_task]
            rospy.logwarn(req)
                
            self.countObject_client.wait_for_service(timeout=10.0)
            res = self.countObject_client.call(req)
            rospy.logwarn(res)
            userdata.count_num = res.num

            pid = get_pid("people_tracking")
            subprocess.Popen(['kill','-9',pid],shell=False)
            with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                 f.write('')

            return 'succeeded'
            
        except Exception:
            rospy.logerr('e')
            return 'aborted'
        

class GetPeopleNum(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['gesture','count_num'],
                       output_keys=['count_num','gesture'])
        #self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
        #rospy.wait_for_service('handGesture')
        # self.nav_client = actionlib.SimpleActionClient(
        #     "move_base", MoveBaseAction)

        
        # self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel',Twist,queue_size=1)
        # self.twist_xm = Twist()
        
        self.countPeople_client = rospy.ServiceProxy('countPeople', xm_count_people)
        

    def execute(self, userdata):
        try:
        #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        #     rospy.sleep(2.0)
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_count_person_by_gesture.py','-d','true'] ,shell =False)
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
                pid = get_pid("people_tracking")
                subprocess.Popen(['kill','-9',pid],shell=False)
                with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                    f.write('')
                a.wait()
                return 'aborted'
        except:
             rospy.logerr('No param specified')
             return 'error'
        
        rospy.loginfo('.................Count People ^_^..........\n')
        try:
            self.string_ = str(userdata.gesture)#返回对象的字符串形式
            self.index = -1
            userdata.gesture = ''
            rospy.logwarn(self.string_)

            if(self.string_ == 'waving'):
                self.index = 0
            elif(self.string_ == 'left_point'):
                self.index = 1
            elif(self.string_ == 'right_point'):
                self.index = 2
            elif(self.string_ == 'left_raise'):
                self.index = 3
            elif(self.string_ == 'right_raise'):
                self.index = 4
            elif(self.string_ == 'standing'):
                self.index = 5
            elif(self.string_ == 'sitting'):
                self.index = 6
            elif(self.string_ == 'lying'):
                self.index = 7
            elif(self.string_ == 'male_person'):
                self.index = 8
            elif(self.string_ == 'female_person'):
                self.index = 9
            elif(self.string_ == 'boy'):
                self.index = 10
            elif(self.string_ == 'girl'):
                self.index = 11
            elif(self.string_ == 'man'):
                self.index = 12
            elif(self.string_ == 'woman'):
                self.index = 13
            elif(self.string_ == 'name'):
                self.index = 14
            else:
                self.index = -1
            
            self.string_ = ''
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                req = xm_count_peopleRequest()
                req.index = self.index
                rospy.logwarn(req)
                
                self.countPeople_client.wait_for_service(timeout=10.0)
                res = self.countPeople_client.call(req.index)
                rospy.logwarn(res)
                userdata.count_num = res.num
                rospy.logwarn(userdata.count_num)
                pid = get_pid("people_tracking")
                subprocess.Popen(['kill','-9',pid],shell=False)
                with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                    f.write('')

                return 'succeeded'
            except Exception:
                pid = get_pid("people_tracking")
                subprocess.Popen(['kill','-9',pid],shell=False)
                with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                    f.write('')
                rospy.logerr(e)
                return 'aborted'

class FindPeople_new(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['gesture'],
                       output_keys=['character','gesture','pos_xm'])
        #self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
        #rospy.wait_for_service('handGesture')
        # self.nav_client = actionlib.SimpleActionClient(
        #     "move_base", MoveBaseAction)

        
        # self.cmd_vel = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel',Twist,queue_size=1)
        # self.twist_xm = Twist()
        self.tf_listener = tf.TransformListener()
        self.findPeople_client = rospy.ServiceProxy('findPeople', xm_find_people)
        

    def execute(self, userdata):
        try:
        #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        #     rospy.sleep(2.0)
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_find_and_desc_person_by_gesture.py','-d','true'] ,shell =False)
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
        
        rospy.loginfo('.................Find People ^_^..........\n')
        try:
            self.string_ = str(userdata.gesture)#返回对象的字符串形式
            self.index = -1
            userdata.gesture = ''
            rospy.logwarn(self.string_)

            if(self.string_ == 'waving'):
                self.index = 0
            elif(self.string_ == 'pointing_to_the_left'):
                self.index = 1
            elif(self.string_ == 'pointing_to_the_right'):
                self.index = 2
            elif(self.string_ == 'raising_their_left_arm'):
                self.index = 3
            elif(self.string_ == 'raising_their_right_arm'):
                self.index = 4
            elif(self.string_ == 'standing'):
                self.index = 5
            elif(self.string_ == 'sitting'):
                self.index = 6
            elif(self.string_ == 'lying_down'):
                self.index = 7
            elif(self.string_ == 'male_person'):
                self.index = 8
            elif(self.string_ == 'female_person'):
                self.index = 9
            elif(self.string_ == 'boy'):
                self.index = 10
            elif(self.string_ == 'girl'):
                self.index = 11
            elif(self.string_ == 'man'):
                self.index = 12
            elif(self.string_ == 'woman'):
                self.index = 13
            elif(self.string_ == 'name'):
                self.index = 14
            else:
                self.index = -1
            
            self.string_ = ''
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                req = xm_find_peopleRequest()
                req.index = self.index
                rospy.logwarn(req)
                
                self.findPeople_client.wait_for_service(timeout=10.0)
                res = self.findPeople_client.call(req.index)
                rospy.logwarn(res)
                userdata.character = res.character
                self.tmp_pos = res.position
                
                pid = get_pid("people_tracking")
                subprocess.Popen(['kill','-9',pid],shell=False)
                with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                    f.write('')
                    
                if self.tmp_pos.point.x== -12.0 and self.tmp_pos.point.y == -12.0 and self.tmp_pos.point.z == -12.0:
                    rospy.logwarn('lost people and I will turn right!')
                    self.twist_xm.angular.z = 0.7
                    self.cmd_vel.publish(self.twist_xm)
                    rospy.sleep(1.0)
                    #ps = self.data_deal_lose_right_sign(self.tmp_pos)
                    #userdata.pos_xm = ps
                    #pass
                    return 'aborted'

                if self.get_distance(self.tmp_pos) >= 0.5 and self.get_distance(self.tmp_pos)<=4.0 :
                    rospy.loginfo('i will move')
                    ps =self.data_deal(self.tmp_pos)
                    userdata.pos_xm = ps
                    return 'succeeded'
                elif self.get_distance(self.tmp_pos) < 0.5:
                    rospy.loginfo('i will not move')
                    ps = self.data_deal_turn(self.tmp_pos)
                    userdata.pos_xm = ps
                    return 'succeeded'
                else:
                    rospy.logerr('the person is out of the range')
                    return 'aborted'
            except Exception as e:
                rospy.logerr(e)
                return 'error'
            
        
    def get_distance(self,pos_xm):
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y

        return  hypot(person_x,person_y)
    def data_deal_turn(self,pos_xm):
        #图像到导航的坐标转换
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y
        #计算人和xm连线与视线正前方夹角
        angle = atan2(person_y,person_x)
        #初始化xm现在的位置用于之后得到base_link在全局坐标系中的位置
        pos_xm.point.x = 0
        pos_xm.point.y = 0
        pos_xm.point.z = 0
        new_header = Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        #从角度得到四元数
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        rospy.logwarn(qs)
        #等待tf的信息
        self.tf_listener.waitForTransform('map', 'base_link',rospy.Time(), rospy.Duration(60.0))
        rospy.logwarn('get the tf message')
        #利用tf信息转化坐标
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')
        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps

    #返回xm经过处理后的Pose()
    def data_deal(self,pos_xm):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y
        
        #计算方位角
        angle = atan2(person_y, person_x)
        #这里是为了到人的面前进行问题回答
        #person_x = person_x - (hypot(person_x,person_y)-0.2)*cos(angle)
        #person_y = person_y - (hypot(person_x,person_y)-0.2)*sin(angle)
        #person_x = person_x - 0.6*cos(angle)
        #person_y = person_y - 0.6*sin(angle)
        # pos_xm.point.x = person_x
        # pos_xm.point.y =person_y
        # pos_xm.point.z =0

        pos_xm.point.x = person_x - 0.8
        pos_xm.point.y =person_y
        pos_xm.point.z =0  
        # init the stamped of the Header
        # 初始化Header
        new_header =Header()
        #new_header.frame_id = 'camera_base'
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header

        #一个四元数描述了旋转轴和旋转角度
        #绕z轴旋转angle角度
        #这个函数从欧拉旋转（绕x轴旋转角，绕y轴旋转角，绕z轴旋转角）变换到四元数表示旋转
        #对给定的旋转轴(a,b,c)和一个角度theta对应四元数
        #q = (a*sin(theta/2), b*sin(theta/2), c*sin(theta/2), cos(theta))
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        rospy.loginfo(self.q)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        #self.tf_listener.waitForTransform('map','camera_base',rospy.Time(),rospy.Duration(60.0))    
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    

        #pos_xm是一个Point()
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')    

        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps
    
    def data_deal_lose_right_sign(self,pos_xm):
        pos_xm = Pose()
        pos_xm.position.x = -12
        return pos_xm

class FindPeople():
    def __init__(self):
        self.find_people_ = MonitorState('follow',
                                        xm_FollowPerson,
                                        self.people_cb,
                                        max_checks =5,
                                        output_keys=['pos_xm'])

        self.tf_listener = tf.TransformListener()

    # 如果相机找到人,这个状态将会返回False
    # 相反,如果在五个循环里相机都没找到人,将会返回True
    # msg传入主题的数据
    def people_cb(self,userdata,msg):
        if msg is not None:
            try:
                self.tmp_pos = msg.position
                rospy.logwarn(self.tmp_pos)
            #如果得到人的坐标信息返回移动的位置
                if self.get_distance(self.tmp_pos) >= 0.5 and self.get_distance(self.tmp_pos)<=4.0 :
                    rospy.loginfo('i will move')
                    ps =self.data_deal(self.tmp_pos)
                    userdata.pos_xm = ps
                    return False
                elif self.get_distance(self.tmp_pos) < 0.5:
                    rospy.loginfo('i will not move')
                    ps = self.data_deal_turn(self.tmp_pos)
                    userdata.pos_xm = ps
                    return False
                else:
                    rospy.logerr('the person is out of the range')
                    return True
            except:
                rospy.logerr(e)
                return False
            
        else:
            raise Exception('MsgNotFind')
    def get_distance(self,pos_xm):
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y

        return  hypot(person_x,person_y)
    def data_deal_turn(self,pos_xm):
        #图像到导航的坐标转换
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y
        #计算人和xm连线与视线正前方夹角
        angle = atan2(person_y,person_x)
        #初始化xm现在的位置用于之后得到base_link在全局坐标系中的位置
        pos_xm.point.x = 0
        pos_xm.point.y = 0
        pos_xm.point.z = 0
        new_header = Header()
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header
        #从角度得到四元数
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        rospy.logwarn(qs)
        #等待tf的信息
        self.tf_listener.waitForTransform('map', 'base_link',rospy.Time(), rospy.Duration(60.0))
        rospy.logwarn('get the tf message')
        #利用tf信息转化坐标
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')
        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps
    #返回xm经过处理后的Pose()
    def data_deal(self,pos_xm):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        # 由于我们改变了camera_link 的坐标,所以数据处理可能没有跟着改变
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y
        
        #计算方位角
        angle = atan2(person_y, person_x)
        #这里是为了到人的面前进行问题回答
        #person_x = person_x - (hypot(person_x,person_y)-0.2)*cos(angle)
        #person_y = person_y - (hypot(person_x,person_y)-0.2)*sin(angle)
        person_x = person_x - 0.6*cos(angle)
        person_y = person_y - 0.6*sin(angle)
        # pos_xm.point.x = person_x
        # pos_xm.point.y =person_y
        # pos_xm.point.z =0

        pos_xm.point.x = person_x - 0.2
        pos_xm.point.y =person_y
        pos_xm.point.z =0  
        # init the stamped of the Header
        # 初始化Header
        new_header =Header()
        #new_header.frame_id = 'camera_base'
        new_header.frame_id = 'base_link'
        pos_xm.header = new_header

        #一个四元数描述了旋转轴和旋转角度
        #绕z轴旋转angle角度
        #这个函数从欧拉旋转（绕x轴旋转角，绕y轴旋转角，绕z轴旋转角）变换到四元数表示旋转
        #对给定的旋转轴(a,b,c)和一个角度theta对应四元数
        #q = (a*sin(theta/2), b*sin(theta/2), c*sin(theta/2), cos(theta))
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        rospy.loginfo(self.q)
        qs = QuaternionStamped()
        qs.header = pos_xm.header
        qs.quaternion = self.q
        #self.tf_listener.waitForTransform('map','camera_base',rospy.Time(),rospy.Duration(60.0))    
        self.tf_listener.waitForTransform('map','base_link',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    

        #pos_xm是一个Point()
        pos_xm = self.tf_listener.transformPoint('map',pos_xm)

        rospy.logwarn('tf point succeeded ')    

        #qs是一个四元数
        qs =self.tf_listener.transformQuaternion('map',qs)

        rospy.logwarn('tf quaternion succeeded ')

        ps = Pose(pos_xm.point,qs.quaternion)
        return ps

class FindObject(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['name'],
                       output_keys=['object_pos'],
                       io_keys=['objmode'])
        self.xm_findobject = rospy.ServiceProxy(
            'get_position', xm_ObjectDetect)
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        goal = Point()
        try:
            name = userdata.name
            rospy.logwarn(name)
            if name == "Icetea":
                name = 'Ice tea'
            if name == 'Milktea':
                name = 'Milk tea'
            if name == 'Grapejuice':
                name = 'Grape juice'
            if name == 'orange_juice':
                name = 'juice'
            p = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_detect_object.py',
            '-d','true'], shell=True)
            with open('/home/xm/vision_pid/object_detect.txt','w') as f:
                f.write(str(p.pid))
        except:
            rospy.logerr('No param specified')
            return 'error'
        # i从1到5
        if name == 'tooth brush':
            userdata.objmode = 0
            rospy.logerr('hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh')
        else:
            userdata.objmode = 1
            rospy.logerr('ssssssssssssssssssssssssssssssssssss' +
                         str(userdata.objmode))

        for i in range(5):
            try:
                self.xm_findobject.wait_for_service(timeout=10.0)
                req = xm_ObjectDetectRequest()
                req.object_name = name
                rospy.logwarn(name)
                rospy.logwarn(req)
                res = self.xm_findobject.call(req)
                rospy.logerr("oooooooo")
                if len(res.object) != 0:
                    break
            except Exception, e:
                rospy.logerr(e)
                return 'aborted'
        rospy.logwarn(res.object[0])
        if i == 2:
            rospy.logerr('result wrong')
            return 'aborted'
        if res.object[0].pos.point.x == -10.000 and res.object[0].pos.point.y == -10.000 and res.object[0].pos.point.z == -10.000:
            rospy.logerr('find nothing')
            return 'aborted'
        #   object_pos 是 PointStamped 类型
        object_pos = PointStamped()
        object_pos.point.x = res.object[0].pos.point.z - 0.11    #-0.11
        object_pos.point.y = res.object[0].pos.point.x - 0.125   #-0.125
        object_pos.point.z = 0.917-res.object[0].pos.point.y
        object_pos.header.frame_id = 'camera_base'

       # 将物品的位置信息传输到userdata
        userdata.object_pos = object_pos
        try:
            # pid_str = subprocess.check_output('ps -aux | grep mrsupw_detect_object.py' , shell= True)
            # pid_str1 = pid_str.splitlines()[0].split()[0]
            # rospy.logwarn(pid_str1)
            # subprocess.call('kill '+pid_str1 , shell = True)
            pid = get_pid("object_detect")
            subprocess.Popen(['kill','-9',pid],shell=True)
            print("sleepiiiiiiiiing!!!!!!!!!!!")
            return 'stop'


        except Exception,e:
            rospy.logerr('No such process ')
            return 'succeeded'

        # userdata.object_pos = res.object[0].pos
        # output_keys cannot be read
        # print userdata.object_pos
        return 'succeeded'
