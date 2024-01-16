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
from xm_smach.pick_turn import PickTurn, IsTurn
from xm_msgs.msg import *
'''
Find My Mates
报告三个客人的描述和位置信息
房间里有4个人
图像节点：
        在拍摄到的6个图片中，
        输出3个客人的位置信息列表，可参考多人辨识的脚本，用于判断位置和身高；
        输出客人的姿态，用于辅助身高判断以及姿态描述(坐着还是站着，0为站着，1为坐着)
        输出客人的性别，年龄，衣服...
        服务名称可以沿用多人辨识里的get_position,但是数据类型需要更改，
        服务有两种情况，command为0时，单纯拍摄照片并存到指定文件
        command为1时，对指定文件中的6张图片进行分析并返回数据
        
        
状态机：
        对客人位置信息与预定的点进行比较，确定位置信息并转换为文本；
        对图像识别的人的空间高数数据处理，得出人的身高
语音：
        经过图像节点和状态机，客人列表信息已经处理完成；
        （信息包括：位置，姿态，身高（数值），性别，年龄，衣服...）
        语音将传过去的身高、年龄、衣服等信息进行唯一性判断，转换为一个文本描述
        并说出，处理过程写成一个函数即可，输入为信息列表（长度为3），输出为一段文本

流程：
    导航到操作员地点->语音交互操作员->导航到指定房间便于观察的位置->开启图像节点(旋转拍摄6张照片之后处理)
    ->处理数据（身高高和位置）->导航返回操作员位置->语音处理数据->说出->退场

规则疑问：
        是否需要分清客人的姓名，如果要分清，分清的手段是什么，如果是举手的话，会不会扣分，还是用声音
未确定问题：
        指定房间物体位置信息
        沙发的高度    
        拍摄照片的间隔时间（视摄像头稳定）
        拍摄照片的张数
        图像传来的数据的具体格式
        导航之前清空地图的频率问题（不宜太过频繁）

'''

class TakePhoto(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'],
                        input_keys = ['name'])
    
    def execute(self,userdata):
        try:
            a = subprocess.Popen('xterm -e ./Vision/Camera_mates/build/take_photos_find_my_mates '+userdata.name,shell =True)
            rospy.sleep(4.0)
            a.wait()
            if a.poll() !=0:
                return 'aborted'
        except:
            rospy.logerr('Vision/Camera_mates/build/take_photos_find_my_mates process error')
            return 'aborted'
        return 'succeeded'

class Analyse(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'],
                            input_keys = ['name'])
    
    def execute(self,userdata):
        try:
            a = subprocess.Popen('xterm -e python Vision/Function/find_my_mates.py '+userdata.name,shell =True)
            rospy.sleep(4.0)
            a.wait()
            if a.poll() != 0:
                return 'aborted'
        except:
            rospy.logerr('Vision/Function/find_my_mates.py process error')
            return 'aborted'
        return 'succeeded'


class Scan(State):
    '''
    旋转拍1张照片，之后再调用图像服务得到客人信息列表
    '''

    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'],
                       output_keys=['information',''])
        self.cmd_vel = rospy.Publisher(
            '/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        # 暂定
        #self.scan_client = rospy.ServiceProxy('get_position', xm_ObjectDetect)
        self.scan_client = rospy.ServiceProxy('get_information',xm_Person_Information)
        self.information = list()

    def execute(self, userdata):
        try:
            for i in range(6):
                self.cmd_vel.publish(self.turn)
                # 等待摄像头稳定
                rospy.sleep(3)
                # 拍照片
                self.scan_client.call(command=0)
        except:
            rospy.logerr('meet wrong when turn and take photos')
            return 'error'
        else:
            self.information = self.scan_client.call(command=1)
            rospy.logwarn(len(self.information))
            if len(self.information) != 3:
                return 'aborted'
            else:
                rospy.logwarn(self.information)
                userdata.information = self.information
                return 'succeeded'


class GetLocationAndHeight(State):
    '''
    对传过来的信息进行处理
    人的姿态：坐着为1，站着为0
    记住调整沙发的高度
    '''

    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['information'],
                       input_keys=['sofa_height'])
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        try:
            getattr(userdata, 'sofa_height')
            getattr(userdata, 'information')

            for i in range(3):
                # 身高信息修正
                userdata.information[i].height = float(
                    userdata.information[i].pos.point.y)+int(userdata.information[i].state)*userdata.sofa_height
                userdata.information[i].height = str(
                    userdata.information[i].height)
                # 位置信息修正
                userdata.information[i].location = self.get_location(
                    userdata.information[i].pos)
        except:
            rospy.logerr('meet wrong when fix the information')
            return 'error'
        else:
            return 'succeeded'

    def data_deal(self, pos_xm0):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        person_x = pos_xm0.point.z
        person_y = pos_xm0.point.x
        angle = atan2(person_y, person_x)
        person_x = person_x - 0.7*cos(angle)
        person_y = person_y - 0.7*sin(angle)
        pos_xm0.point.x = person_x
        pos_xm0.point.y = person_y
        pos_xm0.point.z = 0
        new_header = Header()
        new_header.frame_id = 'base_link'
        pos_xm0.header = new_header
        # change
        q_angle = quaternion_from_euler(0, 0, angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header = pos_xm0.header
        qs.quaternion = self.q

        self.tf_listener.waitForTransform(
            'map', 'base_link', rospy.Time(), rospy.Duration(60.0))

        rospy.logwarn('wait for tf succeeded ')

        pos_xm0 = self.tf_listener.transformPoint('map', pos_xm0)
        rospy.logwarn('tf point succeeded ')

        qs = self.tf_listener.transformQuaternion('map', qs)
        rospy.logwarn('tf quaternion succeeded ')
        pos_xm = Pose(pos_xm0.point, qs.quaternion)
        return pos_xm

    def get_location(self, pos):
        position = self.data_deal(pos)
        match_degree = 1000
        location = ""
        # 在特定房间里寻找最匹配的
        for key in room:
            degree = pow(room[key]['pos'].position.x-position.x, 2) + \
                pow(room[key]['pos'].position.y-position.y, 2)
            if degree < match_degree:
                location = key
                match_degree = degree
        return location

'''
class GetInformation(State):
    
    将信息转换为句子
    

    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['information'],
                       output_keys=['sentences_information']
                       )

    def execute(self, userdata):
        try:
            pass
            userdata.sentences_information = self.get_information(
                userdata.information)
        except:
            rospy.logerr(
                'meet wrong when turn the information to the sentnece')
            return 'error'
        else:
            return 'succeeded'

    def get_information(self, information):
        sentences = ""
        return sentences
'''
class CheckTurn(State):
    '''
    用于判断循环的一个状态
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','continue','error'],
                        io_keys =['index'])

    def execute(self,userdata):
        try:
            getattr(userdata, 'index')
        except:
            rospy.logerr('no param')
            return 'error'
        else :
            if useradta.num <3:
                return 'continue'
            else:
                return 'succeeded'


class GetAngle(State):
    '''
    用于获取听音辨位获取的角度
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','preempt','aborted'],
                        output_keys =['angle'])
        self.angle =0.0
    def execute(self,userdata):
        if self.preempt_requested():
            return 'preempt'
        try:
            self.getAndle_cmd = rospy.Subscriber('mobile_base/sound_source' , sound_source , 
                                    callback = self.angle_cb , queue_size = 1 )
            rospy.sleep(1)
            while not rospy.is_shutdown():
                if self.preempt_requested():
                    return 'preempt'
                else:
                    if self.angle:
                        userdata.angle = self.angle/360.0*2*3.1415926535
                        rospy.logwarn(self.angle)
                        return 'succeeded'
                print(self.angle)
        except Exception,e:
            rospy.logerr(e)
            return 'aborted'


    def angle_cb(self, msg):
        if msg is not None:
            if msg.sound_source is not 0.0:
                if msg.sound_source <180:
                    self.angle = -msg.sound_source
                else:
                    self.angle = 360-msg.sound_source
                rospy("The message from the topic is :"+msg.sound_source)
                  
        else:
            raise Exception('MsgNotFind')


class GetSentence(State):
    '''
    获取听音辨位语句
    '''
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','aborted','error'],
                        input_keys =['name_list'],
                        output_keys = ['sentences','name'],
                        io_keys = ['index'])

    def execute(self,userdata):
        try:
            if len(userdata.name_list) == 0:
                rospy.logwarn("All the mates have benn found")
                return 'aborted'
            else:
                userdata.sentences = 'Who is '+userdata.name_list[userdata.index]+'?'
                userdata.name = userdata.name_list[userdata.index]
                userdata.index +=1
                return 'succeeded'
        except:
            return 'error'


class FindMyMates():
    def __init__(self):
        rospy.init_node('FindMyMates_Smach')
        rospy.on_shutdown(self.shutdown)
        rospy.logerr('Welcome to FindMyMates!!!')
        self.smach_bool = False

        self.FindMyMates = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'])
        with self.FindMyMates:
            self.FindMyMates.userdata.information = list()
            # 操作员位置
            self.FindMyMates.userdata.start_pos = gpsr_target['speaker']['pos']
            # 观察位置
            self.FindMyMates.userdata.scan_pos = gpsr_target['table']['pos']
            # 退场位置
            self.FindMyMates.userdata.exit_pos = gpsr_target['speaker']['pos']
            self.FindMyMates.userdata.sentences_start = 'i will check your mates,please stay here'
            self.FindMyMates.userdata.sentences_information = ""
            self.FindMyMates.userdata.index = 0
            self.FindMyMates.userdata.name = 'jacy'
            self.FindMyMates.userdata.name_list = ['Coulson','Mike','Jack']

            StateMachine.add('TakePhoto',
                                    TakePhoto(),
                                    transitions={'succeeded':'ANALYSE','aborted':'ANALYSE'},
                                    remapping = {'name':'name'}
                                    )        
            StateMachine.add('ANALYSE',
                                    Analyse(),
                                    transitions={'succeeded':'succeeded','aborted':'aborted'}
                                    )     
        intro_server = IntrospectionServer('FindMyMates', self.FindMyMates, '/SM_ROOT')
        intro_server.start()
        out = self.FindMyMates.execute()
        intro_server.stop()
    def shutdown(self):
        if self.smach_bool == True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')
'''
            self.sm_FindOne = StateMachine(outcomes = ['succeeded','aborted','error'],
                                    input_keys = ['name_list','index'],
                                    output_keys = ['index'])
            with self.sm_FindOne:
                StateMachine.add('GETSENTECE',
                                    GetSentence(),
                                    transitions={'succeeded':'ASK','aborted':'aborted','error':'error'},
                                    remapping = {'sentences':'sentences','index':'index','name_list':'name_list','name':'name'}
                                    )
                StateMachine.add('ASK',
                                    Speak(),
                                    transitions={'succeeded':'GETANGLE','aborted':'aborted','error':'error'},
                                    remapping = {'sentences':'sentences'}
                                    )
                StateMachine.add('GETANGLE',
                                    GetAngle(),
                                    transitions={'succeeded':'TURN','aborted':'aborted','preempt':'error'},
                                    remapping = {'angle':'degree'}
                                    )
                StateMachine.add('TURN',
                                    TurnDegree(),
                                    transitions={'succeeded':'TakePhoto','aborted':'aborted','error':'error'},
                                    remapping = {'degree':'degree'}
                                    )  
                StateMachine.add('TakePhoto',
                                    TakePhoto(),
                                    transitions={'succeeded':'ANALYSE','aborted':'ANALYSE'},
                                    remapping = {'name':'name'}
                                    )        
                StateMachine.add('ANALYSE',
                                    Analyse(),
                                    transitions={'succeeded':'succeeded','aborted':'aborted'}
                                    )                       
            StateMachine.add('START',
                             NavStack(),
                             transitions={'succeeded': 'SPEAK','aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'start_pos'}
                             )

            StateMachine.add('SPEAK',
                             Speak(),
                             transitions={'succeeded': 'GOTOSCAN', 'aborted': 'aborted', 'error': 'error'},
                             remapping={'sentences': 'sentences_start'}
                             )
            StateMachine.add('GOTOSCAN',
                             NavStack(),
                             transitions={'succeeded': 'FINDONE','aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'scan_pos'}
                             )
            StateMachine.add('FINDONE',
                                self.sm_FindOne,
                                transitions={'succeeded':'CHECKTURN','aborted':'aborted','error':'error'},
                                remapping = {'name_list':'name_list','index':'index'}
                                )  
            StateMachine.add('CHECKTURN',
                                CheckTurn(),
                                transitions={'succeeded':'succeeded','continue':'FINDONE','error':'error'},
                                remapping = {'index':'index'}
                                )                                 


   
            
            StateMachine.add('BACK',
                             NavStack(),
                             transitions={'succeeded': 'GETINFORMATION','aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'start_pos'}
                             )
            
            StateMachine.add('GETINFORMATION',
                             GetInformation(),
                             transitions={'succeeded': 'TALK','aborted': 'aborted', 'error': 'error'},
                             remapping={'sentences_information': 'sentences_information'}
                             )
            StateMachine.add('TALK',
                             Speak(),
                             transitions={'succeeded': 'EXIT','aborted': 'aborted', 'error': 'error'},
                             remapping={'sentences': 'sentences_information'}
                             )
            StateMachine.add('EXIT',
                             NavStack(),
                             transitions={'succeeded': 'succeeded','aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'exit_pos'}
                             )
'''





if __name__ == "__main__":
    try:
        FindMyMates()
    except Exception, e:
        rospy.logerr(e)
