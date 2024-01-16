#!/usr/bin/env python
# encoding:utf8
from distutils.log import error
import rospy
from smach import *
from smach_ros import *
from xm_msgs.srv import *
from xm_msgs.msg import *
from geometry_msgs.msg import *
import subprocess


def get_pid(name):
    pid = ''
    with open("/home/xm/vision_pid/{}.txt".format(name)) as f:
        pid = f.read()
    return pid

class TakePhoto(State):
    def __init__(self):
        State.__init__(self,
                       outcomes=['succeeded', 'aborted', 'error'],
                       io_keys=["num"])
        self.xm_find_people = rospy.ServiceProxy('findPeople', xm_find_people)


    def execute(self, userdata):
        try:
        #     subprocess.call("xterm -e python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py",shell=True)
        #     rospy.sleep(2.0)
            a = subprocess.Popen(['python3','/home/xm/catkin_ws/src/xm_vision/src/scripts/Carry/take_a_photo.py','-d','true'] ,shell =False)
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
            self.xm_find_people.wait_for_service(timeout=30.0)

            req = xm_find_peopleRequest()
            req.index=0
            
            rospy.logwarn(req)

            
            res = self.xm_find_people.call(req.index)
        
        except:
            self.killPro()
            rospy.logerr("xxxxxxxxxxxxxx")
        rospy.logwarn(res)
        self.killPro()
        userdata.num+=1
        if userdata.num>=3:
            return "error"
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




class Carry():
    def __init__(self):
        rospy.init_node("Gpsr_Smach")
        rospy.on_shutdown(self.shutdown)
        rospy.logerr("Welcome to Carry!")
        self.smach_bool = False

         # 顶层状态机
        self.xm_Carry = StateMachine(
            outcomes=['succeeded', 'aborted', 'error'])
        with self.xm_Carry:
            self.xm_Carry.userdata.num=0
            StateMachine.add('RECEIVE_TASKS',
                             TakePhoto(),
                             transitions={'succeeded': 'RECEIVE_TASKS',
                                          'aborted': 'RECEIVE_TASKS', 'error': 'error'})
            

        intro_server = IntrospectionServer(
            'xm_Carry', self.xm_Carry, '/XM_ROOT')
        intro_server.start()
        out = self.xm_Carry.execute()
        intro_server.stop()

    def shutdown(self):
        if self.smach_bool == True:
            rospy.loginfo('smach succeeded')
        else:
            rospy.loginfo('smach error')

if __name__ == "__main__":
    try:
        Carry()
    except:
        rospy.logerr(55555)

