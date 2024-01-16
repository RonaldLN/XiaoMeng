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

room={
    #center is the left lower corner of one object,
    #the center type is depended on the map type 
    #to make sure the size.a and size.b is bigger than zero
    'sofa':{'center':[-0.5,0.7],'size':[1.0,2.0]},
    'table':{'center':[0.9,-1.2],'size':[1.0,1.0]},
    'door':{'center':[1.7,-1.7],'size':[0.6,0.6]},
    #'bed':{'center':[0.0,0.0],'size':[0.0,0.0]},
    #'table':{'center':[0.0,0.0],'size':[0.0,0.0]},
    #'table':{'center':[0.0,0.0],'size':[0.0,0.0]},
    #'table':{'center':[0.0,0.0],'size':[0.0,0.0]},
    #'table':{'center':[0.0,0.0],'size':[0.0,0.0]},
    #'table':{'center':[0.0,0.0],'size':[0.0,0.0]},
    #'table':{'center':[0.0,0.0],'size':[0.0,0.0]},

}



class Analyse(State):
    def __init__(self):
        State.__init__(self,outcomes=['succeeded','aborted'],
                            input_keys = ['name'])
    
    def execute(self,userdata):
        try:
            a = subprocess.Popen('xterm -e python3 Vision/Function/find_my_mates.py '+userdata.name,shell =True)
            rospy.sleep(4.0)
            a.wait()
            if a.poll() != 0:
                return 'aborted'
        except:
            rospy.logerr('Vision/Function/find_my_mates.py process error')
            return 'aborted'
        return 'succeeded'




class FixInformation(State):
 
    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'close','far','error'],
                       input_keys = ['name'],
                       io_keys=['information_list'])
        self.tf_listener = tf.TransformListener()
        self.location = ""

    def execute(self, userdata):
        #1.open the file
        #2.get the msg.position
        #3. tmp_pos = msg.postion
        #4. pos_xm = self.data_deal(tmp_pos)
        #5.location = self.get_location(pos_xm)
        try:
            getattr(userdata, 'information_list')

            filename = '/home/domistic/Vision/TXT/'+userdata.name+'.txt'
            information = ""
            with open(filename) as f:
                information = f.readlines()
                for i in range(0,len(information)):
                    information[i] = information[i].rstrip('\n')

                #判断相片的信息是否太近或太远
                if information[8] == 1:
                    rospy.logwarn("too close") 
                    return 'close'
                if information[8] == 2:
                    rospy.logwarn("too far")    
                    return 'far'      
                if information[8] ==3:
                    rospy.logwarn('no people')
                    return 'error'

                #提取信息到dict中        
                dict_ = {}
                dict_['Name'] = information[0]
                dict_['Age'] = information[1].replace("Age: ","")
                dict_['Gender'] = information[2].replace("Gender: ","")
                dict_['Glass'] = information[3].replace("Glass: ","")
                dict_['Upper_wear'] = information[4].replace("Upper_wear: ","")
                dict_['Upper_wear_color'] = information[5].replace("Upper_wear_color: ","")
                dict_['Posture'] = information[6].replace("Posture: ","")
                dict_['Location'] = information[7]
                
                #修正位置信息
                pos = information[7].split(" ")
                pos_xm = PointStamped()
                pos_xm.point.x = float(pos[5])#1
                pos_xm.point.y = float(pos[1])#3
                pos_xm.point.z = -float(pos[3])#
                dict_['Location'] = self.get_location(pos_xm)

                userdata.information_list.append(dict_)
                rospy.logwarn(dict_)
        except Exception, e:
            rospy.logerr('meet wrong when fix the information')
            rospy.logerr(e)
            return 'error'
        else:
            return 'succeeded'

    def data_deal(self, pos_xm0):
        # 这个方法简单地处理来自cv的数据,将数据从PointStmp()类型转换到Pose()类型
        person_x = pos_xm0.point.z
        person_y = pos_xm0.point.x
        angle = atan2(person_y, person_x)

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
        '''
        
        '''     
        position = self.data_deal(pos)
        location = ""
        min_percent = 1000
        print(position)
        # 在特定房间里寻找最匹配的
        for key in room:
            print(key)
            size_a = abs(position.position.x - room[key]['center'][0])/room[key]['size'][0]
            size_b = abs(position.position.y - room[key]['center'][1])/room[key]['size'][1]
            print(size_a)
            print(size_b)
            if size_a+size_b < min_percent:
                min_percent = size_a+size_b
                location = key
        return location


class GetInformation(State):
    '''
    将信息转换为句子
    '''

    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'error'],
                       input_keys=['information_list'],
                       output_keys=['sentences_information']
                       )

    def execute(self, userdata):
        try:
            sentence_list =  self.get_information(userdata.information_list)
            userdata.sentences_information = "First of all,"+sentence_list[0]+"Then,"+sentence_list[1]+"In the last,"+sentence_list[2]
        except:
            rospy.logerr('meet wrong when turn the information to the sentnece')
            return 'error'
        else:
            return 'succeeded'

    def get_information(self, information_list):
        person = information_list
        sentences = ["","",""]
        age = []
        gender = []
        glass = []
        cloth = []
        for i in range(3):
            age.append(person[i]["Age"])
            gender.append(person[i]["Gender"])
            glass.append(person[i]["Glass"])
            cloth.append(person[i]["Upper_wear_color"]+" "+person[i]["Upper_wear"])
            if person[i]["Posture"][1] == 't':
                person[i]["Posture"] = "standing near"
            else:
                person[i]["Posture"] = "sitting on"

        sentences = ["","",""]
        for i in range(3):
            sentences[i] += "I find "+person[i]["Name"]+" is "+person[i]["Posture"]+" the "+person[i]["Location"]+". "
            pron = ["she","her"] if gender[i][0]=='f' else ["he","his"]
            if gender.count(gender[i]) == 1:
                sentences[i]+=pron[0] + " looks like a "+gender[i]+" to me. "
            if age.count(age[i]) == 1:
                sentences[i]+=pron[0] + " seems "+age[i]+" years old. "
            if glass.count(glass[i]) == 1:
                sentences[i]+=pron[0]
                sentences[i]+= " wears glasses. " if glass[i][0]=='y' else " does not wear glasses. "
            if cloth.count(cloth[i]) == 1:
                sentences[i]+="and "+pron[0]+" wears "+cloth[i]+"."
        return sentences

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
            if userdata.index < 3:
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
        self.client = rospy.ServiceProxy('/mobile_base/SoundSource',SoundSource)

        self.angle =0.0
    def execute(self,userdata):
        try:
            self.client.wait_for_service(timeout=10.0)
            res = self.client.call("s")
            self.angle = res.angle
            if self.angle < 180:
                self.angle = -self.angle
            else:
                self.angle = 360-self.angle

            rospy.logwarn(self.angle)
            userdata.angle = self.angle/360.0*2*3.1415926535
            return 'succeeded'
        except Exception, e:
            rospy.logerr(e)
            return 'aborted'



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
        rospy.logwarn('Welcome to FindMyMates!!!')
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
            self.FindMyMates.userdata.information_list = list()

            self.FindMyMates.userdata.name_list = ['Coulson','Mike','Jack']

            self.sm_FindOne = StateMachine(outcomes = ['succeeded','aborted','error'],
                                    input_keys = ['name_list','index'],
                                    output_keys = ['index','name'])
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
                                    transitions={'succeeded':'ANALYSE','aborted':'aborted','error':'error'},
                                    remapping = {'degree':'degree'}
                                    )       
                StateMachine.add('ANALYSE',
                                    Analyse(),
                                    transitions={'succeeded':'succeeded','aborted':'aborted'},
                                    remapping = {'name':'name'}
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
                                transitions={'succeeded':'FIXINFORMATION','aborted':'aborted','error':'error'},
                                remapping = {'name_list':'name_list','index':'index'}
                                )  
            StateMachine.add('FIXINFORMATION',
                                    FixInformation(),
                                    transitions={'succeeded':'CHECKTURN','far':'error','close':'error','error':'error'},
                                    remapping = {'information_list':'information_list','name':'name'}
                                    )     
            StateMachine.add('CHECKTURN',
                                CheckTurn(),
                                transitions={'succeeded':'BACK','continue':'FINDONE','error':'error'},
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
                             remapping={'sentences_information': 'sentences'}
                             )
            StateMachine.add('TALK',
                             Speak(),
                             transitions={'succeeded': 'EXIT','aborted': 'aborted', 'error': 'error'},
                             remapping={'sentences': 'sentences'}
                             )
            StateMachine.add('EXIT',
                             NavStack(),
                             transitions={'succeeded': 'succeeded','aborted': 'aborted', 'error': 'error'},
                             remapping={"pos_xm": 'exit_pos'}
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


if __name__ == "__main__":
    try:
        FindMyMates()
    except Exception, e:
        rospy.logerr(e)
