#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from speech.srv import *

def sr_online_file_client():
    rospy.wait_for_service('sr_online_file')
    try:
        add_two_ints = rospy.ServiceProxy('sr_online_file', speech)
        resp = add_two_ints("START!")
        return resp.output
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    print(sr_online_file_client())
    
