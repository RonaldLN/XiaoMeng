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
# 去卧室的中间点
# 'med_pos': {'pos': Pose(Point(6.1346, 3.7437, 0), Quaternion(0, 0, 0.67904, 0.7341)), 'mode': 1 },
# 去客厅的中间点
'med_pos': {'pos': Pose(Point(1.6328, 0.46815, 0), Quaternion(0, 0, 0.97512, -0.22167)), 'mode': 1 },

# 卧室/客厅的位置，确定可以看到整个卧室的全貌，并且指向原来主人的位置
'bedroom_pos': {'pos': Pose(Point(1.6328, 0.46815, 0), Quaternion(0, 0, 0.97512, -0.22167)), 'mode': 1 },

# 座位
'char1': {'pos': Pose(Point(2.2631, 0.96031, 0), Quaternion(0, 0, 0.93473, 0.35537)), 'mode': 1 },
'char2': {'pos': Pose(Point(2.9716, 0.96944, 0), Quaternion(0, 0, 0.67849, 0.73461)), 'mode': 1 },



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

'room_object_pos1': {'pos':Pose(Point(-0.555668, 10.2652, 0), Quaternion(0, 0, 0.00808197, 0.999967)), 'mode':1 },

#12
'room_object_pos2': {'pos': Pose(Point(-1.02133, 10.3692, 0), Quaternion(0, 0, 0.999728, -0.0233193)), 'mode': 1 },

#2
'room_object_pos3': {'pos': Pose(Point(7.83509, -1.32254, 0), Quaternion(0, 0, 0.687419, 0.726261)), 'mode': 1 },
#1
'room_object_pos4': {'pos': Pose(Point(2.80011, -9.37624, 0), Quaternion(0, 0, 0.0694549, 0.997585)), 'mode':1 },

'room_object_pos5': {'pos': Pose(Point(7.19218, -1.29891, 0), Quaternion(0, 0, 0.999728, -0.0196512)), 'mode': 1 },

'room_object_pos6': {'pos': Pose(Point(2.6266, -7.41669, 0), Quaternion(0, 0, 0.999915, -0.0130226)), 'mode': 1 },


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

'start_pos':{'pos':Pose(Point(-3.25205, 6.8939, 0), Quaternion(0, 0, 0.934965, 0.354739)), 'mode':1 },
'exit_pos':{'pos':Pose(Point(5.3841, 2.4464, 0), Quaternion(0, 0, 0.3858, 0.92279)), 'mode':1 },

# 'gettopoint1':{'pos':Pose(Point(-5.95525, 7.46056, 0), Quaternion(0, 0, 0.455917, 0.890022)), 'mode':1 },
'gettopoint2':{'pos':Pose(Point(-0.13689, 5.66002, 0), Quaternion(0, 0, 0.911169, -0.412032)), 'mode':1 },
#first
'start_pos2':{'pos':Pose(Point(5.1723, -6.27738, 0), Quaternion(0, 0, -0.2357, 0.9718)), 'mode':1 },

'gettopoint3':{'pos':Pose(Point(2.60525, -4.14056, 0), Quaternion(0, 0, 0.455917, 0.890022)), 'mode':1 },
#2
'gettopoint4':{'pos':Pose(Point(6.37119, -7.06262, 0), Quaternion(0, 0, -0.721868, 0.692031)), 'mode':1 },
'gettopoint1':{'pos':Pose(Point(5.77504, -11.7585, 0), Quaternion(0, 0, 0.612679, 0.790332)), 'mode':1 },

#############################################################

}
