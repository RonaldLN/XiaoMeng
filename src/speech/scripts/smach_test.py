#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from speech.srv import *

def speech_under(num):
    rospy.wait_for_service('speech_core')
    text="are you kidding?"
    try:
        under = rospy.ServiceProxy('speech_core', speech_to_smach)
        res = under(num,text)
        print(res.num)
        print(res.action)
        print(res.object)
        print(res.answer)
        print(res.gesture)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [num]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        num = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    speech_under(num)
