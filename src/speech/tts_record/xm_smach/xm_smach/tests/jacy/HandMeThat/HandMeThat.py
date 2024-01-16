#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import StateMachine
from smach_ros import IntrospectionServer
from xm_smach.common_lib import *
from xm_smach.gpsr_lib import * 
from xm_smach.help_me_carry_lib import *
from xm_smach.target_gpsr import gpsr_target
from geometry_msgs.msg import *
import math
import subprocess
from xm_smach.pick_turn import PickTurn , IsTurn
from xm_smach.store_lib import *
'''
Hand Me That
规则理解:
        操作员依次指示个放置物体的地方，机器人到达后询问问题，触碰物体示意
        每完成一次，机器人回到固定位置，操作员位置不变，图像识别人的指向
状态机:
        导航到观察操作员的位置->观察操作员指向->与预存的环境信息匹配，找到具体组->
        图像识别该组全部物体，并返回物体名称->提取物体名称，根据名称，在文件中找该组物体的不同属性，生成问题->触碰该物体
        （定义一个物体类，该类有各种属性，全为布尔类型，对应着操作员的yes no，
        具体的物体，比如橘子是可以吃的，需要剥皮，等等属性都已经事先编程记录）

图像：
        主要问题在于识别操作员的指向，从而根据指向和场地实际情况，分配到各个组面前
        还需要一个服务，图像识别摄像头中全部物体，并将物体名称按位置顺序写到一个文件中（我看到这个功能之前就有了）

问题:
        需要在比赛之前确定object的具体属性以及与之对应的问题
        xxx状态机编程中还没有写如何通过图像识别到的数据导航到具体位置的代码xxx
'''
#每问一次，如果出现一个yes no，则更新object——list
class Object(object):
    '''
    用来存储一个物体属性的类
    '''
    def __init__(self,overhead,hollow,edible,skinning,plastic_bag):
        self.overhead = overhead #高的
        self.hollow = hollow #空心
        self.edible = edible #可食用的
        self.skinning = skinning #需要剥皮的
        self.plastic_bag = plastic_bag #含有塑料袋

    @staticmethod 
    def compare(object_list):
        #用于比较不同并且生成语句的方法
        #句子前面的编号是为了便于辨别是在讨论哪个属性，从而根据回答更新物体列表
        #问题越靠前，越容易问出来
        question = list()
        length = len(object_list)

        for i in range(1,length):
            if object_list[0].overhead !=object_list[i].overhead:
                question.append("1_Is it higher")
                break
        for i in range(1,length):
            if object_list[0].hollow !=object_list[i].hollow:
                question.append("2_Is it hollow")
                break

        for i in range(1,length):
            if object_list[0].edible !=object_list[i].edible:
                question.append("3_Is it edible")
                break

        for i in range(1,length):
            if object_list[0].skinning !=object_list[i].skinning:
                question.append("4_Is it skinning")
                break
        for i in range(1,length):
            if object_list[0].plastic_bag !=object_list[i].plastic_bag:
                question.append("5_Is it plastic_bag")
                break
        
        return question

object_translation = {
    'orange':Object(0,0,1,1,0),
    'milk':Object(0,0,1,0,0)
}

class Resure(State):
    '''
    语音交互确认
    reply  1-yes   0-no
    '''
    def __init__(self):
        State.__init__(self,
                       outcomes=['clear', 'unclear','error'],
                       input_keys = ['sentences'],
                       output_keys= ['reply'])
        self.client = rospy.ServiceProxy('xm_speech_meaning',xm_Speech_meaning)
        self.speak_client = rospy.ServiceProxy("tts", xm_Speech_tts)
    def execute(self, userdata):
        try:
            getattr(userdata, 'sentences')
        except:
            rospy.logerr('No param')
            return 'error'
        else:
            try:
                self.string_ = userdata.sentences
                rospy.logwarn(self.string_)
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
                return 'error'
            else:
                try:
                    self.client.wait_for_service(timeout=10)
                except:
                    rospy.logerr('xm_speech_meaning service is error')
                    return 'error'
                else:
                    res = self.client.call(command=3)
                    if res == 'yes':
                        userdata.reply =1
                        return 'clear'

                    elif res == 'no':
                        userdata.reply =0
                        return 'clear'

                    else:
                        return 'unclear'
class RunNode(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'])
    
    def execute(self,userdata):
        try:
            subprocess.Popen('xterm -e rosrun xm_vision object_all &',shell =True)
        except:
            rospy.logerr('all_object node error')
            return 'aborted'
        return 'succeeded'

class GetObject_list(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'],
                        output_keys = ['object_list'])
    def execute(self,userdata):
        try:
            test_open = os.access('/home/jacy/Vision/data/result/out.txt',os.R_OK)
            if test_open == True:
                self.obj_file = open('/home/jacy/Vision/data/result/out.txt','r')
                self.obj_list = list()
                for line in self.obj_file:
                    self.obj_list.append(str(line.replace('\n','')))
                #转换成对象，具有属性
                for i in range(len(self.obj_list)):
                    thing = object_translation[self.obj_list[i]]
                    userdata.object_list.append(thing)
                
                rospy.logwarn(self.obj_list)
                subprocess.call('python xm_pdf.py',shell = True)
                return 'succeeded'

            else:
                rospy.logerr('cannot open the out.txt')
                return 'aborted'

        except Exception,e:
            rospy.logerr(e)
            return 'aborted'
                
class GetQuestion(State):
    '''
    用于提取问题
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted'],
                        input_keys =['object_list'],
                        output_keys= ['question_list'])

    def execute(self,userdata):
        try:
            getattr(userdata, 'object_list')
        except:
            rospy.logerr('no param')
            return 'aborted'
        else :
            userdata.question_list = Object.compare(userdata.object_list)
            rospy.logwarn(question_list)
            return 'succeeded'

class GetSentence(State):
    '''
    用于提取一个问题，并且删除这个问题在列表中
    '''
    def __init__(self):
        State.__init__(self,outcomes =['continue','fail'],
                        io_keys =['question_list'],
                        output_keys= ['sentence','case'])

    def execute(self,userdata):
        try:
            getattr(userdata, 'question_list')
        except:
            rospy.logerr('all the question is done')
            return 'fail'
        else :
            sentence_ = userdata.question_list.pop(0)
            userdata.case = int(sentence_.split('_')[0])
            userdata.sentence = sentence_.split('_')[1]
            rospy.logwarn(userdata.case)
            rospy.logwarn(userdata.sentence)
            return 'continue'
class UpdateObject(State):
    '''
    用于更新物体列表,并当物体确定后返回物体名称
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','finish'],
                        io_keys =['object_list'],
                        input_keys= ['reply','case'],
                        output_keys= ['object'])

    def execute(self,userdata):
        try:
            getattr(userdata, 'object_list')
            getattr(userdata, 'reply')
            getattr(userdata, 'case')
        except:
            rospy.logerr('no param')
            return 'aborted'
        else :
            length = len(userdata.object_list)
            
            if userdata.case == 1:
                for i in range(length):
                    if userdata.object_list[i].overhead !=userdata.reply:
                        userdata.object_list.pop(i)
            elif userdata.case == 2:
                for i in range(length):
                    if userdata.object_list[i].hollow !=userdata.reply:
                        userdata.object_list.pop(i)
            elif userdata.case == 3:
                for i in range(length):
                    if userdata.object_list[i].edible !=userdata.reply:
                        userdata.object_list.pop(i)
            elif userdata.case == 4:
                for i in range(length):
                    if userdata.object_list[i].skinning !=userdata.reply:
                        userdata.object_list.pop(i)
            elif userdata.case == 5:
                for i in range(length):
                    if userdata.object_list[i].plastic_bag !=userdata.reply:
                        userdata.object_list.pop(i)
            if len(userdata.object_list) ==1:
                userdata.object = str(userdata.object_list[0])
                return 'finish'
            else:
                userdata.object= ""
                return 'succeeded'


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
            if useradta.num <=4:
                userdata.num +=1
                return 'continue'
            elif userdata.num ==5:
                return 'succeeded'
            else:
                return 'error'

class GetPos(State):
    '''
    
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','continue','error'],
                        input_keys =['pos1','pos2'],
                        output_keys = ['pos'])

    def execute(self,userdata):
        try:
            getattr(userdata, 'pos1')
            getattr(userdata, 'pos2')
        except:
            rospy.logerr('no param')
            return 'error'
        else :
            




class HandMeThat():
    def __init__(self):
        rospy.init_node('HandMeThat_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to HandMeThat!!!')
        self.smach_bool = False

        self.sm_Guess = StateMachine(outcomes = ['succeeded','aborted','fail'],
                                        input_keys = ['object_list'],
                                        output_keys = ['object'])
        with self.sm_Guess:
            self.sm_Guess.userdata.sentences = ""
            StateMachine.add('GETQUESTION',
                                GetQuestion(),
                                transitions = {'succeeded':'GETSENTENCE','aborted':'GETQUESTION'},
                                remapping = {'object_list':'object_list','question_list':'question_list'}
                                )   
            StateMachine.add('GETSENTENCE',
                                GetSentence(),
                                transitions = {'continue':'ASK','fail':'fail'},
                                remapping = {'sentence':'sentences','case':'case','question_list':'question_list'}
                                )
            StateMachine.add('ASK',
                                Resure(),
                                transitions={'clear':'UPDATEOBJECT','unclear':'GETSENTENCE','error':'fail'},
                                remapping = {'sentences':'sentences','reply':'reply'}
                                )
            StateMachine.add('UPDATEOBJECT',
                                UpdateObject(),
                                transitions={'succeeded':'GETQUESTION','aborted':'aborted','finish':'succeeded'},
                                remapping = {'object_list':'object_list','reply':'reply','case':'case'}
                                )


        self.sm_Touch = StateMachine(outcomes =['succeeded','aborted','error'],
                                    input_keys =['target'])    
        with self.sm_Touch: 
            self.sm_Touch.userdata.name =''
            self.sm_Touch.userdata.target_mode =0
            self.sm_Touch.userdata.objmode = -1

            StateMachine.add('RUNNODE_IMG',
                                RunNode_img(),
                                transitions = {'succeeded':'FIND_OBJECT','aborted':'RUNNODE_IMG'})
            self.sm_Touch.userdata.mode_1 =1
            self.sm_Touch.userdata.object_pos = PointStamped()
            StateMachine.add('FIND_OBJECT',
                                FindObject(),
                                transitions ={'succeeded':'POS_JUSTFY','aborted':'succeeded','error':'SPEAK'},
                                remapping ={'name':'name','object_pos':'object_pos','objmode':'objmode'})
            self.sm_Touch.userdata.pose = Pose()
            StateMachine.add('POS_JUSTFY',
                                PosJustfy(),
                                remapping={'object_pos':'object_pos','pose':'pose'},
                                transitions={'succeeded':'NAV_TO','aborted':'aborted','error':'error'})
            StateMachine.add('NAV_TO',
                                NavStack(),
                                transitions ={'succeeded':'RUNNODE_IMG2','aborted':'NAV_TO','error':'error'},
                                remapping ={"pos_xm":'pose'})
            StateMachine.add('RUNNODE_IMG2',
                                RunNode_img(),
                                transitions = {'succeeded':'FIND_AGAIN','aborted':'RUNNODE_IMG2'})                    
            StateMachine.add('FIND_AGAIN',
                                FindObject(),
                                transitions ={'succeeded':'PICK_JUS','aborted':'succeeded','error':'error'},
                                remapping ={'name':'name','object_pos':'object_pos','objmode':'objmode'})
            StateMachine.add('PICK_JUS' , 
                                PickJustfy(),
                                transitions = {'succeeded':'PICK','error':'error'},
                                remapping = {'name':'name',
                                            'object_pos':'object_pos'})
            self.sm_Touch.userdata.arm_mode_1 =1
            StateMachine.add('PICK',
                                ArmCmd(),
                                transitions ={'succeeded':'succeeded','aborted':'aborted','error':'error'},
                                remapping ={'arm_ps':'object_pos','mode':'arm_mode_1'})
            self.sm_Touch.userdata.sentences = 'xiao meng can not find the thing'
            StateMachine.add('SPEAK',
                                Speak(),
                                transitions={'succeeded':'succeeded','aborted':'aborted','error':'error'})


        
        
        
        #顶层状态机
        self.HANDMETHAT = StateMachine(outcomes = ['succeeded','aborted','error'])
        with self.HANDMETHAT:
            self.HANDMETHAT.userdata.num = 1 #循环次数
            self.HANDMETHAT.userdata.pos_init = gpsr_target['speaker']['pos']  #观察操作员指向的位置            
            StateMachine.add('NAV_INIT',
                    NavStack(),
                    transitions = {'succeeded':'FINDOBJECT','aborted':'NAV_INIT','error':'error'},
                    remapping = {'pos_xm':'pos_init'}
                )
            StateMachine.add('FINDOBJECT',
                    GetObject_list(),
                    transitions = {'succeeded':'GETOBJECTLIST','aborted':'FINDOBJECT'}
                )
            StateMachine.add('GETOBJECTLIST',
                    NavStack(),
                    transitions = {'succeeded':'GUESS','aborted':'GETOBJECTLIST'},
                    remapping = {'object_list':'object_list'}
                )         
            StateMachine.add('GUESS',
                    self.sm_Guess,
                    transitions = {'succeeded':'TOUCH','aborted':'GUESS','fail':'CHECKTURN'},
                    remapping = {'object_list':'object_list','object':'object'}
                )  
            StateMachine.add('TOUCH',
                    self.sm_Touch,
                    transitions = {'succeeded':'CHECKTURN','aborted':'TOUCH','error':'error'},
                    remapping = {'object_list':'object_list','object':'object'}
                )          
            StateMachine.add('CHECKTURN',
                    CheckTurn(),
                    transitions = {'succeeded':'succeeded','continue':'NAV_INIT','error':'error'},
                    remapping = {'num':'num'}
                )                           

                                                    

        intro_server = IntrospectionServer('HANDMETHAT',self.HANDMETHAT,'/HANDMETHAT')
        intro_server.start()
        out = self.HANDMETHAT.execute()
        intro_server.stop()

  
    def shutdown(self):
        if self.smach_bool ==True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

if __name__ == "__main__":
    try:
        HandMeThat()
    except Exception,e:
        rospy.logerr(e)   
