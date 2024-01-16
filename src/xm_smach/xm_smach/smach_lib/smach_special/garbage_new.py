#!/usr/bin/env python
# encoding:utf8
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import *
import tf
from control_msgs.msg import *
import subprocess
import math  
from speech.srv import *
from copy import deepcopy

def get_pid(name):
    pid = ''
    with open('/home/xm/vision_pid/{}.txt'.format(name)) as f:
        pid = f.read()
    return  pid


class GetTaskG(State):
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

class NextDoG(State):###跳转action
    def __init__(self):
        State.__init__(self, 
                        #nav_grasp is something interesting
                    outcomes=['succeeded','aborted','go','error'],
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
        # current_task目前测使的次数
        # task_num 测试总次数
        if userdata.current_task ==  task_num:
            return 'succeeded'
            
        current_action =  action[userdata.current_task]

        if current_action == 'go':
            return 'go'
        
        else:
            # no avaiable action find
            # userdata.current_task_out -1
            userdata.current_task -=1
            return 'aborted'

class CheckTurnG(State):
    def __init__(self):
        State.__init__(self, outcomes=['succeeded','move1','move2','move3','error'],
                        io_keys=['current_turn','task_num','current_task','check_first','check_second'],
                        input_keys=['turn'])
    def execute(self,userdata):
        try:
            self.current_turn = userdata.current_turn
            self.turn = userdata.turn
            rospy.logwarn(self.current_turn)
            rospy.logwarn(self.turn)
            rospy.logwarn('check first :' + str(userdata.check_first))
            rospy.logwarn('check second :' + str(userdata.check_second))

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
            if userdata.check_first == 0:
                return 'move1'
            elif userdata.check_second == 0:
                return 'move2'
            else :
                return 'move3'
            

class GetTargetG(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','error'],
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
        
        rospy.logwarn(userdata.target)

        #if userdata.current_task == 'speaker':
        #     userdata.current_target = gpsr_target[userdata.target['speaker2']]['pos']
        # else:
        target0 =userdata.target[0]
        if target0 =='show' or target0=='shop':
            target0 = 'shelf'
        if target0 == 'living_room':
            target0 = 'livingroom'
        if target0 == 'dining_room':
            target0 = 'diningroom'
        print(target0)
        userdata.current_target = gpsr_target[target0]['pos']#存目标位置
        rospy.logwarn(userdata.current_task)

        return 'succeeded'

class GetTargetG_1(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','error'],
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
        
        rospy.logwarn(userdata.target)

        #if userdata.current_task == 'speaker':
        #     userdata.current_target = gpsr_target[userdata.target['speaker2']]['pos']
        # else:
        target0 =userdata.target[0]
        if target0 =='show' or target0=='shop':
            target0 = 'shelf'
        if target0 == 'living_room':
            target0 = 'livingroom'
        if target0 == 'dining_room':
            target0 = 'diningroom'
        
        target1 = target0 + '_1'
        print(target1)
        userdata.current_target = gpsr_target[target1]['pos']#存目标位置
        rospy.logwarn(userdata.current_task)

        return 'succeeded'

class GetTargetG_2(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','error'],
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
        
        rospy.logwarn(userdata.target)

        #if userdata.current_task == 'speaker':
        #     userdata.current_target = gpsr_target[userdata.target['speaker2']]['pos']
        # else:
        target0 =userdata.target[0]
        if target0 =='show' or target0=='shop':
            target0 = 'shelf'
        if target0 == 'living_room':
            target0 = 'livingroom'
        if target0 == 'dining_room':
            target0 = 'diningroom'
        
        target1 = target0 + '_2'
        print(target1)
        userdata.current_target = gpsr_target[target1]['pos']#存目标位置
        rospy.logwarn(userdata.current_task)

        return 'succeeded'

class GetObjectName(State):
    def __init__(self):
        State.__init__(self,outcomes =['succeeded','error'],
                            input_keys =['targetName','targetType'],
                            output_keys =['current_target'])
    def execute(self,userdata):
        try:#判断这些属性是否存在
            getattr(userdata,'targetType')
        except:
            rospy.logerr('No params specified ')
            return 'error'
        
        rospy.logwarn(userdata.targetType)

        #if userdata.current_task == 'speaker':
        #     userdata.current_target = gpsr_target[userdata.target['speaker2']]['pos']
        # else:
        target0 =userdata.targetType
        
        print(target0)
        userdata.current_target = gpsr_target[target0]['pos']#存目标位置

        return 'succeeded'

class GetObjectPosition_G(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['objectName'],
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

class GetObjectPositionG_new_1(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target','objectName','objectType','current_task','check_first'],
                       output_keys=['target_camera_point' ,'table_depth','objectName','objectType'])
        self.xm_findobject = rospy.ServiceProxy('find_garbage', xm_find_object)


    def execute(self, userdata):
        # try:
        #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        #     rospy.sleep(2.0)
            # a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_find_garbage.py'] ,shell =False)
            # with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
            #     print('-------------------')
            #     print('pid',a.pid)
            #     print('pid',a.pid)
            #     print('pid',a.pid)
            #     print('pid',a.pid)
            #     print('pid',a.pid)
            #     print('pid',a.pid)
            #     print('-------------------')
            #     f.write(str(a.pid))
            # rospy.sleep(2.0)
            # print a.poll()
            # if a.returncode != None:
            #     a.wait()
            #     self.killPro()
            #     return 'aborted'
        # except:
        #      rospy.logerr('No param specified')
        #      return 'error'
        
        goal = Point()
        

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)

            req = xm_find_objectRequest()
            
            res = self.xm_findobject.call(req)
            userdata.objectName = res.name   
            userdata.objectType = res.type
            rospy.logwarn(res)
            if res.position.point.x != 0 or res.position.point.y != 0 or res.position.point.z != 0:
                rospy.loginfo("find object!")
        except Exception as e:
            rospy.logerr(e)
            rospy.logwarn('bad call the service')
            self.killPro()
            return 'aborted'

        if res.position.point.x == -10.000:
            rospy.logerr('find nothing')
            userdata.check_first = 1
            rospy.logwarn('check first: ' + str(userdata.check_first))
            
            # self.killPro()
            return 'aborted'

        
        rospy.logwarn(res.position)

        userdata.target_camera_point = res.position
        #self.killPro()
        return 'succeeded'

    def killPro(self):
        try:
            # pid_str = subprocess.check_output('ps -aux | grep mrsupw_detect_object.py' , shell= True)
            # pid_str1 = pid_str.splitlines()[0].split()[1]
            # rospy.logwarn(pid_str1)
            # subprocess.call('kill -9 '+pid_str1 , shell = True)

            pid = get_pid("people_tracking")
            subprocess.Popen(['kill','-9',pid],shell=False)
            # with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
            #     f.write('')
                
        except Exception,e:
            rospy.logerr('No such process ')

class GetObjectPositionG_new_2(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=['target','objectName','objectType','current_task','check_second'],
                       output_keys=['target_camera_point' ,'table_depth','objectName','objectType'])
        self.xm_findobject = rospy.ServiceProxy('find_garbage', xm_find_object)


    def execute(self, userdata):
        # try:
        # #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        # #     rospy.sleep(2.0)
        #     a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/GPSR/mrsupw_find_garbage.py'] ,shell =False)
        #     with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
        #         print('-------------------')
        #         print('pid',a.pid)
        #         print('pid',a.pid)
        #         print('pid',a.pid)
        #         print('pid',a.pid)
        #         print('pid',a.pid)
        #         print('pid',a.pid)
        #         print('-------------------')
        #         f.write(str(a.pid))
        #     rospy.sleep(2.0)
        #     print a.poll()
        #     if a.returncode != None:
        #         a.wait()
        #         self.killPro()
        #         return 'aborted'
        # except:
        #      rospy.logerr('No param specified')
        #      return 'error'
        
        goal = Point()
        

        try:
            self.xm_findobject.wait_for_service(timeout=30.0)

            req = xm_find_objectRequest()
            
            res = self.xm_findobject.call(req)
            userdata.objectName = res.name   
            userdata.objectType = res.type
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
            userdata.check_second = 1
            rospy.logwarn('check first: ' + str(userdata.check_second))
            #self.killPro()
            return 'aborted'

        
        rospy.logwarn(res.position)

        userdata.target_camera_point = res.position
        #self.killPro()
        return 'succeeded'

    def killPro(self):
        try:
            # pid_str = subprocess.check_output('ps -aux | grep mrsupw_detect_object.py' , shell= True)
            # pid_str1 = pid_str.splitlines()[0].split()[1]
            # rospy.logwarn(pid_str1)
            # subprocess.call('kill -9 '+pid_str1 , shell = True)

            pid = get_pid("people_tracking")
            subprocess.Popen(['kill','-9',pid],shell=False)
            # with open("/home/xm/vision_pid/people_tracking.txt",'w') as f:
            #     f.write('')
                
        except Exception,e:
            rospy.logerr('No such process ')


class ArmTrajectory_Before(State):
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

class ArmTrajectory_After(State):
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


      
