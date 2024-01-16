#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import *
from smach_ros import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
import subprocess
from speech.srv import *
from pydub import AudioSegment#音频预处理
from pydub.playback import play#用于播放音频

'''
Author: yyb
Date: 2021-01-30 16:11:15
LastEditTime: 2021-01-30 16:11:21
LastEditors: yyb
Description: In User Settings Edit
FilePath: /jacy_lib/speech.py
'''
def get_pid(name):
    pid = ''
    with open("/home/xm/vision_pid/{}.txt".format(name)) as f:
        pid = f.read()
    return pid
    # return map(int,subprocess.check_output(["pidof",name]).split())

class SpeakFirstGuestDes(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['sentences','des'],
                       output_keys=['sentences','des'])
        #self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
        #rospy.wait_for_service('speech_core')
        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)
        

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            userdata.sentences = userdata.sentences + str(userdata.des)
            self.string_ = str(userdata.sentences)#返回对象的字符串形式
            rospy.logwarn(self.string_)
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, self.string_)

                rospy.sleep(2.0)
                return 'succeeded'
              
            except:
                return 'aborted'

class SpeakObjectName(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['sentences','objectName'],
                       output_keys=['sentences','objectName'])
        #self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
        #rospy.wait_for_service('speech_core')
        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)
        

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            userdata.sentences = userdata.sentences + str(userdata.objectName)
            self.string_ = str(userdata.sentences)#返回对象的字符串形式
            rospy.logwarn(self.string_)
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, self.string_)
                userdata.sentences = 'please give me '
                rospy.sleep(2.0)
                return 'succeeded'
              
            except:
                return 'aborted'
           
class SpeakAttend(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['sentences','target','current_task'],
                       output_keys=['target','sentences','current_task'])
        #self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
        #rospy.wait_for_service('speech_core')
        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)
        

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            userdata.sentences = str(userdata.target[userdata.current_task]) + userdata.sentences 
            self.string_ = str(userdata.sentences)#返回对象的字符串形式
            
            rospy.logwarn(self.string_)
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, self.string_)

                rospy.sleep(2.0)
                return 'succeeded'
                
            except:
                return 'aborted'
            else:
                return 'succeeded'

class SpeakName(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['sentences','objectName'],
                       output_keys=['objectName','sentences'])
        #self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
        #rospy.wait_for_service('speech_core')
        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)
        

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            userdata.sentences = userdata.sentences + str(userdata.objectName)
            self.string_ = str(userdata.sentences)#返回对象的字符串形式
            userdata.objectName = ''
            rospy.logwarn(self.string_)
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, self.string_)
                userdata.sentences = "the object you asked is " 
                rospy.sleep(2.0)
                return 'succeeded'
                
            except:
                return 'aborted'
            else:
                return 'succeeded'

class SpeakNum(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['sentences','count_num'],
                       output_keys=['count_num','sentences'])
        #self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
        #rospy.wait_for_service('speech_core')
        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)
        

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            userdata.sentences = userdata.sentences + str(userdata.count_num)
            self.string_ = str(userdata.sentences)#返回对象的字符串形式
            userdata.count_num = 0
            rospy.logwarn(self.string_)
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, self.string_)
                userdata.sentences = "the number is " 
                rospy.sleep(2.0)
                return 'succeeded'
                
            except:
                return 'aborted'
            else:
                return 'succeeded'

class Speak_Guide(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['sentences','target','current_task'])

        rospy.wait_for_service('speech_core')
        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)
        

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            self.string_ = str(userdata.sentences)+str(userdata.target[userdata.current_task])#返回对象的字符串形式
            rospy.logwarn(self.string_)
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                # subprocess - Subprocesses with accessible I/O streams
                # 创建一个子进程,父进程等待子进程完成,返回退出信息
                # 让xm在子进程中说出userdata.sentences的语句，不影响父进程
                
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(2, self.string_)

                rospy.sleep(1.0)
                return 'succeeded'
            except:
                return 'aborted'
            

class Speak(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['sentences'])
        #self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
        #rospy.wait_for_service('speech_core')
        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)
        

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            self.string_ = str(userdata.sentences)#返回对象的字符串形式
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

class SpeakAnswer(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['sentences'
                       ])
        #self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
        rospy.wait_for_service('speech_core')
        self.speak_client = rospy.ServiceProxy('speech_core', speech_to_smach)
        

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            self.string_ = str(userdata.sentences)#返回对象的字符串形式
            rospy.logwarn(self.string_)
        except:
            rospy.logerr('No sentences provided')
            return 'error'
        else:
            try:
                # subprocess - Subprocesses with accessible I/O streams
                # 创建一个子进程,父进程等待子进程完成,返回退出信息
                # 让xm在子进程中说出userdata.sentences的语句，不影响父进程
                
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(3, self.string_)

                rospy.sleep(2.0)
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

class SpeakWAV(State):
    '''
    播放音频文件
    '''
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['filename'])
        self.file = ''

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        try:
            self.file = str(userdata.filename)
        except:
            rospy.logerr('No file provided')
            return 'error'
        else:
            try:
                subprocess.call(["play", self.file])
            except:
                return 'aborted'
            else:
                return 'succeeded'

class CheckStop(State):
    '''
    使用语音检测停止信号
    '''
    def __init__(self):
        State.__init__(self,
                        outcomes = ['stop','aborted'])
    
        rospy.wait_for_service('speech_core')
        self.speech_client = rospy.ServiceProxy('speech_core', speech_to_smach)
    def execute(self,userdata):
        try:
            while not rospy.is_shutdown():
                #self.target_client.wait_for_service(timeout = 10)
               # self.response = self.target_client.call(command = 2)
                self.speech_client.wait_for_service(timeout=10)
        #command = 1用于gpsr
                response =self.speech_client.call(command = 1,text = 'www')
                #self.action = self.response.action
                if response.action[0] == 'stop':
                    # subprocess.call("touch /home/ye/Recognition/kinect2/dummy_excution_final &" , shell = True)
                    pid = get_pid("people_tracking")
                    subprocess.Popen(['kill','-9',pid],shell=False)
                    with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
                        f.write('')
                    print("sleepiiiiiiiiing!!!!!!!!!!!")
                    return 'stop'
                else:
                    continue
        except Exception, e:
            rospy.logerr('xm meet wrong when get the target')
            rospy.logerr(e)
            return 'aborted'

class Talk(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['sentences'])
        self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)

    def execute(self, userdata):
        rospy.loginfo('.................Speak ^_^..........\n')
        rospy.logwarn(userdata.sentences)
        try:
            if "1" in userdata.sentences:
                self.string_ =str(userdata.sentences[2:])
            else:
                self.string_ =str(userdata.sentences)
            rospy.logwarn(self.string_)
        except:
            rospy.logerr('get wrong when transfrom the sentences') 
            return 'error'
        else:
            try:
                # subprocess - Subprocesses with accessible I/O streams
                # 创建一个子进程,父进程等待子进程完成,返回退出信息
                # 让xm在子进程中说出userdata.sentences的语句，不影响父进程
                self.speak_client.wait_for_service(timeout=10.0)
                self.speak_client.call(self.string_)

                rospy.sleep(2.0)

                speech_bool = self.speak_client.call(self.string_)
                if speech_bool.flag == 1:
                    subprocess.call(["play", "tts_sample.wav"])
                elif speech_bool.flag == 0:
                    subprocess.call("espeak -vf5 -s 75 '%(a)s'" %
                                    {'a': str(self.string_)}, shell=True)
                else:
                    rospy.logerr('the response error')
                    return 'error'
            except:
                return 'aborted'
            else:
                return 'succeeded'

class CheckIfTalk(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['talk_answer', 'ask'],
                       input_keys = ['current_target'])
    def execute(self, userdata):
        if userdata.current_target == "question" or userdata.current_target == "":
            return 'ask'
        else:
            return 'talk_answer'

class CheckNumOrANS(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['answer', 'num','name'],
                       input_keys = ['current_target'])
    def execute(self, userdata):
        if userdata.current_target == "num" :
            return 'num'
        elif userdata.current_target == "ans":
            return 'answer'
        else:
            return 'name'

class CheckPeopleOrObject(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['people', 'object'],
                       input_keys = ['current_target'])
    def execute(self, userdata):
        if userdata.current_target == "person" :
            return 'people'
        else:
            return 'object'