#!/usr/bin/env python 
#encoding:utf8 

"""

	it is a stuff list for GPSR 
	it contains the pose of all the stuff
        
"""
from geometry_msgs.msg import *
from geometry_msgs.msg._Pose import Pose


# mode = 1 在桌子上
# mode = 2 在架子上
gpsr_target={

#############################################################
###########################通用以A为入口###############################





# A门口迎客的位置
'door_pos': {'pos': Pose(Point(1.6328, 0.46815, 0), Quaternion(0, 0, 0.97512, -0.22167)), 'mode': 1 },

# 中间点
'med_pos': {'pos': Pose(Point(6.1346, 3.7437, 0), Quaternion(0, 0, 0.67904, 0.7341)), 'mode': 1 },
# 'med_pos': {'pos': Pose(Point(1.6328, 0.46815, 0), Quaternion(0, 0, 0.97512, -0.22167)), 'mode': 1 },

# 卧室的位置，确定可以看到整个卧室的全貌，并且指向原来主人的位置
'bedroom_pos': {'pos': Pose(Point(4.2631, 4.6244, 0), Quaternion(0, 0, 0.98964, 0.14358)), 'mode': 1 },

# 座位
'char1': {'pos': Pose(Point(4.2571, 4.6229, 0), Quaternion(0, 0, 0.99566, -0.093082)), 'mode': 1 },
'char2': {'pos': Pose(Point(4.0923, 4.742, 0), Quaternion(0, 0, 0.93736, 0.34835)), 'mode': 1 },



#########################卧室预备点##################
'temp1': {'pos': Pose(Point(4.2571, 4.6229, 0), Quaternion(0, 0, 0.99566, -0.093082)), 'mode': 1 },
'temp2': {'pos': Pose(Point(4.2631, 4.6244, 0), Quaternion(0, 0, 0.98964, 0.14358)), 'mode': 1 },
'temp3': {'pos': Pose(Point(4.0923, 4.742, 0), Quaternion(0, 0, 0.93736, 0.34835)), 'mode': 1 },
'temp4': {'pos': Pose(Point(4.1186, 4.769, 0), Quaternion(0, 0, 0.72943, 0.68405)), 'mode': 1 },


#########################客厅预备点########################3
'temp5': {'pos': Pose(Point(2.2631, 0.96031, 0), Quaternion(0, 0, 0.93473, 0.35537)), 'mode': 1 },
'temp6': {'pos': Pose(Point(2.2631, 0.96031, 0), Quaternion(0, 0, 0.73075, 0.68264)), 'mode': 1 },
'temp7': {'pos': Pose(Point(2.9716, 0.96944, 0), Quaternion(0, 0, 0.67849, 0.73461)), 'mode': 1 },
'temp8': {'pos': Pose(Point(2.9716, 0.96944, 0), Quaternion(0, 0, 0.67849, 0.73461)), 'mode': 1 },
# kitchen
# # 储物柜
# 'room_object_pos1': {'pos': Pose(Point(8.2833, 0.97924, 0), Quaternion(0, 0, 0.27641, 0.96104)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(8.6519, 0.58662, 0), Quaternion(0, 0, 0.3897, 0.92094)), 'mode': 1 },

# # living_room
# # 茶几 
# 'room_object_pos1': {'pos': Pose(Point(1.7373, 0.9539, 0), Quaternion(0, 0, 0.42352, 0.90589)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(2.7522, 0.68115, 0), Quaternion(0, 0, 0.65226, 0.75799)), 'mode': 1 },
# 电视柜
# 'room_object_pos1': {'pos': Pose(Point(2.3529, 0.96717, 0), Quaternion(0, 0, -0.63064, 0.77607)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(3.0907, 0.92901, 0), Quaternion(0, 0, -0.66196, 0.74954)), 'mode': 1 },


# bedroom
# 梳妆台
# 'room_object_pos1': {'pos': Pose(Point(4.5861, 4.6758, 0), Quaternion(0, 0, 0.72281, 0.69105)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(4.0998, 4.783, 0), Quaternion(0, 0, 0.61308, 0.79002)), 'mode': 1 },

# # 学习卓
# 'room_object_pos1': {'pos': Pose(Point(4.2757, 4.9711, 0), Quaternion(0, 0, -0.68753, 0.72616)), 'mode': 1 },
# 'room_object_pos2': {'pos': Pose(Point(3.5197, 4.9212, 0), Quaternion(0, 0, -0.59548, 0.80337)), 'mode': 1 },

# normal
# 'start_pos':{'pos':Pose(Point(6.417, 5.0378, 0), Quaternion(0, 0, -0.31509, 0.94906)), 'mode':1 },
# 'exit_pos':{'pos':Pose(Point(10.904, 6.4459, 0), Quaternion(0, 0, 0.65368, 0.75677)), 'mode':1 },


#############################################################



}
