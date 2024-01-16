#!/usr/bin/env python
# encoding:utf8
import rospy
from smach import *
from smach_ros import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
import subprocess
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from environment_info import *

'''
Author: jacy
Date: 2020-09-11 21:17:17
LastEditTime: 2022-08-25 13:51:11
LastEditors: yishui
Description: In User Settings Edit
FilePath: ~/catkin_ws/src/xm_smach/xm_smach/smach_lib/smach_common
'''


def get_pid(name):
    pid = ''
    with open("/home/xm/vision_pid/{}.txt".format(name)) as f:
        pid = f.read()
    return pid

class GetObjectPosition_new(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target','objectName','current_task'],
                       output_keys=['target_camera_point' ,'table_depth','objectName'])
        self.xm_findobject = rospy.ServiceProxy('findObject', xm_find_object)


    def execute(self, userdata):
        try:
        #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        #     rospy.sleep(2.0)
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_find_object.py','-d','true'] ,shell =False)
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
             return 'error'
        
        goal = Point()
        

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)

            req = xm_find_objectRequest()
            rospy.logwarn('the target to find is ' + userdata.target[userdata.current_task])
            obj=str(userdata.target[userdata.current_task])
            if "_" in obj:
                L=obj.split("_")
                if "three_" in obj:
                    a = "_".join(L[2:])
                    if a!="object" and a!="objects":req.type=a
                    req.adj = "three_"+L[1]
                elif L[0] in ["biggest","largest","thinnest","smallest","heaviest","lightest"]:
                    a = "_".join(L[1:])
                    if a!="object" and a!="objects":req.type=a
                    req.adj = L[0]
                else: 
                    if obj in objects:req.name=obj
                    elif obj in categories:req.type=obj
            else:
                if obj in objects:req.name=obj
                elif obj in categories:req.type=obj
            
            rospy.logwarn(req)

            
            res = self.xm_findobject.call(req)
            userdata.objectName = res.name   
            rospy.logwarn(res)
            if res.position.point.x != 0 or res.position.point.y != 0 or res.position.point.z != 0:
                rospy.loginfo("find object!")
        except Exception, e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        if res.position.point.x == -10.000:
            rospy.logerr('find nothing')
            self.killPro()
            return 'aborted'

        
        rospy.logwarn(res.position)

        userdata.target_camera_point = res.position
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

class GetObjectPosition(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target'],
                       output_keys=['target_camera_point' ,'object_state','table_depth'])
        self.xm_findobject = rospy.ServiceProxy('get_position', xm_ObjectDetect)


    def execute(self, userdata):
        try:
        #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        #     rospy.sleep(2.0)
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_vison_server.py','-d','true'] ,shell =False)
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
        
        goal = Point()
        try:
            name =str( userdata.target)
            name.replace(' ' , '' , name.count(' '))    
        except Exception ,e:
            rospy.logerr(e)
            rospy.logerr('No param specified')
            self.killPro()
            return 'error'

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)

            req = xm_ObjectDetectRequest()
            req.object_name = name
            req.people_id = 0
            rospy.logwarn(name)
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
        userdata.target = name
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

class TargerPosReload(State):
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
                        input_keys= ['target'],
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
            name = userdata.target
            target_camera_point = userdata.target_camera_point
            if name =='milk':
                userdata.target_camera_point = target_camera_point
            else:
                userdata.target_camera_point = target_camera_point
            return 'succeeded'

class GetCallingPos(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted','error'],
                       output_keys = ['pos_xm'])
        self.tf_listener = tf.TransformListener()
        self.ps = Pose()
    def execute(self, userdata):
        try:
            subprocess.call("xterm -e python3 Vision/Function/Phone.py &",shell=True)
            rospy.sleep(2.0)
        except:
            rospy.logerr('No param specified')
            return 'error'
        try:
            filename = '/home/domistic/Vision/TXT/calling.txt'
            information = ""
            with open(filename) as f:
                information = f.readlines()
                
                information = information[0].rstrip('\n')
            pos_list = information.split(" ")

            pos = PointStamped()
            pos.point.x = float(pos_list[0])
            pos.point.y = float(pos_list[1])
            pos.point.z = float(pos_list[2])

            userdata.pos_xm = self.data_deal(pos,1.0)

            return 'succeeded'
        except:
            return 'error'
        return 'succeeded'

    def data_deal(self,pos_xm,distance):
        '''
        获得距离distance距离的位置，修改pos_xm和self.ps
        '''
        # the simple deal for data from the cv
        person_x = pos_xm.point.x
        person_y = pos_xm.point.y
        angle = math.atan2(person_y, person_x)
        person_x = person_x - distance*math.cos(angle)
        person_y = person_y - distance*math.sin(angle)
        pos_xm.point.x = person_x
        pos_xm.point.y =person_y
        pos_xm.point.z =0
        new_header =Header()
        new_header.frame_id = 'camera_base'
        pos_xm.header = new_header
        # change 
        q_angle = quaternion_from_euler(0,0,angle)
        self.q = Quaternion(*q_angle)
        qs = QuaternionStamped()
        qs.header  =pos_xm.header
        qs.quaternion = self.q
      
   
        self.tf_listener.waitForTransform('map','cameara_base',rospy.Time(),rospy.Duration(60.0))    
    
        rospy.logwarn('wait for tf succeeded ')    
        

        pos_xm =self.tf_listener.transformPoint('map',pos_xm)
        rospy.logwarn('tf point succeeded ')    

        #修改self.ps
        qs =self.tf_listener.transformQuaternion('map',qs)
        rospy.logwarn('tf quaternion succeeded ')    
        self.ps = Pose(pos_xm.point,qs.quaternion)
        self.ps.position.z = 0

        return self.ps