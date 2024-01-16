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
'speaker': {'pos': Pose(Point( 3.689,1.117, 0.000), Quaternion(0.000, 0.000, -0.093, 1)), 'mode': 1 },
#'speaker':{'pos':Pose(Point(7.408, -4.733 ,0.000), Quaternion(0.000, 0.000, 0.0403 , 1)),'mode': 1},
'end': {'pos': Pose(Point(11.363, 8.234 , 0.000), Quaternion(0.000, 0.000, 0.619,0.780)), 'mode': 1 },
'Gray': {'pos': Pose(), 'mode': 1 },
'David': {'pos': Pose(), 'mode': 1 },
'Daniel': {'pos': Pose(), 'mode': 1 },
'Jack': {'pos': Pose(), 'mode': 1 },
'Jenny': {'pos': Pose(), 'mode': 1 },
'Michael': {'pos': Pose(), 'mode': 1 },
'Lucy': {'pos': Pose(), 'mode': 1 },
'Peter': {'pos': Pose(), 'mode': 1 },
'Tom': {'pos': Pose(), 'mode': 1 },
'Jordan': {'pos': Pose(), 'mode': 1 },

# 'kitchen_table_1': {'pos': Pose(Point(2.390, -6.656, 0.000),Quaternion(0.000, 0.000, 0.999, -0.036)), 'mode': 1 },
# 'kitchen_table_2': {'pos': Pose(Point(2.390, -6.656, 0.000),Quaternion(0.000, 0.000, 0.999, -0.036)), 'mode': 1 },
####################                 winter_test                                 ################################
'kitchen': {'pos': Pose(Point(5.797, 4.168, 0.000),Quaternion(0.000, 0.000, 0.999, 0.039)), 'mode': 1 },
'diningroom': {'pos': Pose(Point(7.029,3.284, 0.000), Quaternion(0.000, 0.000,0.327, 0.945)), 'mode': 1 },########gai
'hallway': {'pos': Pose(Point(7.029,3.284, 0.000), Quaternion(0.000, 0.000,0.327, 0.945)), 'mode': 1 },
'bedroom': {'pos': Pose(Point(8.061, 0.359, 0.000),Quaternion(0.000, 0, -0.014,1 )), 'mode': 1 },##########gai
'livingroom':{'pos':Pose(Point(1.606, 0.032 , 0.000), Quaternion(0.000, -0.000, 0.025,1)), 'mode':1 },'kitchen_door_in':{'pos':Pose(Point(1.736,2.014,0.000),Quaternion(0.000,0.000,0.989,0.145)),'mode':1},
'kitchen_door_out':{'pos':Pose(Point(1.945,2.107,0.000),Quaternion(0.000,0.000,-0.084,0.996)),'mode':1},
'livingroom_door_in':{'pos':Pose(Point(2.908,1.733,0.000),Quaternion(0.000,0.000,0.027,1.000)),'mode':1},
'livingroom_door_out':{'pos':Pose(Point(2.786,1.672,0.000),Quaternion(0.000,0.000,1.000,0.015)),'mode':1},
'bedroom_door_in':{'pos':Pose(Point(3.095,-1.780,0.000),Quaternion(0.000,0.000,0.003,1.000)),'mode':1},
'bedroom_door_out':{'pos':Pose(Point(2.964,-1.772,0.000),Quaternion(0.000,0.000,0.999,-0.048)),'mode':1},
'diningroom_door_in':{'pos':Pose(Point(2.040,-1.013,0.000),Quaternion(0.000,0.000,0.998,-0.065)),'mode':1},
'diningroom_door_out':{'pos':Pose(Point(2.197,-0.955,0.000),Quaternion(0.000,0.000,0.219,0.976)),'mode':1},
###################################################################################################################
'kitchen_table_1': {'pos': Pose(Point( 5.529, 4.485 , 0.000),Quaternion(0.000, 0.000, 0.687, 0.727)), 'mode': 1 },
'kitchen_table': {'pos': Pose(Point( 5.529, 4.485 , 0.000),Quaternion(0.000, 0.000, 0.687, 0.727)), 'mode': 1 },
'kitchen_table_3': {'pos': Pose(Point(3.499, 4.464, 0.000),Quaternion(0.000, 0.000, -0.716, 0.698)), 'mode': 1 },
'kitchen_table_2':{'pos': Pose(Point(2.744,4.308,0) , Quaternion(0,0,-0.593,0.805)) , 'mode':1 },
'kitchen_table_4':{'pos': Pose(Point(1.172,4.552,0) , Quaternion(0,0,-0.720,0.694)) , 'mode':1 },
'kitchen_counter':{'pos': Pose(Point(1.172,4.552,0) , Quaternion(0,0,-0.720,0.694)) , 'mode':1 },


'bedroom_table_2': {'pos': Pose(Point(9.442, -0.775, 0.000),Quaternion(0.000, 0.000,1, 0.031)), 'mode': 1 },
'bedroom_table_1': {'pos': Pose(Point(8.194, 0.310, 0.000),Quaternion(0.000, 0, 0.667, 0.748)), 'mode': 1 },
'bedroom_table_3':{'pos':Pose(Point(9.312,-0.218 , 0) , Quaternion(0,0,-0.755 , 0.696)), 'mode':1},
'bedroom_table_4':{'pos':Pose(Point(10.228,0.115 , 0) , Quaternion(0,0,0.697,0.717)), 'mode':1},
'shelf':{'pos':Pose(Point(10.228,0.115 , 0) , Quaternion(0,0,0.697,0.717)), 'mode':1},
'right_bedside_table':{'pos':Pose(Point(9.312,-0.218 , 0) , Quaternion(0,0,-0.755 , 0.696)), 'mode':1},

'diningroom_table_1': {'pos': Pose(Point(7.223, 4.237, 0.000),Quaternion(0.000, 0.000,-0.016,1)), 'mode': 1 },
'diningroom_table_3': {'pos': Pose(Point(9.368,  5.816, 0.000),Quaternion(0.000, 0.000, -0.719, 0.695)), 'mode': 1 },
'diningroom_table_2':{'pos':Pose(Point(7.632,4.800,0) , Quaternion(0,0,0.693,0.721)), 'mode':1},
'diningroom_table_5': {'pos': Pose(Point(9.545, 2.539, 0.000),Quaternion(0.000, 0.000,0.659, 0.752)), 'mode': 1 },
'diningroom_table_4': {'pos': Pose(Point(11.568, 3.662, 0.000),Quaternion(0.000, 0.000,-0.741, 0.671)), 'mode': 1 },
'hallway_table_1': {'pos': Pose(Point(7.223, 4.237, 0.000),Quaternion(0.000, 0.000,-0.016,1)), 'mode': 1 },
'hallway_table_3': {'pos': Pose(Point(9.368,  5.816, 0.000),Quaternion(0.000, 0.000, -0.719, 0.695)), 'mode': 1 },
'hallway_table_2':{'pos':Pose(Point(7.632,4.800,0) , Quaternion(0,0,0.693,0.721)), 'mode':1},
'hallway_table_5': {'pos': Pose(Point(9.545, 2.539, 0.000),Quaternion(0.000, 0.000,0.659, 0.752)), 'mode': 1 },
'hallway_table_4': {'pos': Pose(Point(11.568, 3.662, 0.000),Quaternion(0.000, 0.000,-0.741, 0.671)), 'mode': 1 },
'desk': {'pos': Pose(Point(9.368,  5.816, 0.000),Quaternion(0.000, 0.000, -0.719, 0.695)), 'mode': 1 },

'livingroom_table_1': {'pos': Pose(Point(6.490, -0.396, 0.000),Quaternion(0.000, 0.000, -0.474, 0.881)), 'mode': 1 },
'bar': {'pos': Pose(Point(6.490, -0.396, 0.000),Quaternion(0.000, 0.000, -0.474, 0.881)), 'mode': 1 },
#'livingroom_table_2': {'pos': Pose(Point(6.482, -0.381 , 0.000),Quaternion(0.000, 0.000, -0.349, 0.937)), 'mode': 1 },
#'livingroom_table_2': {'pos':Pose(Point(7.069,0.256,0) , Quaternion(0,0,-0.431,0.904)), 'mode' : 1},
#'kitchen_table_1': {'pos': Pose(Point(1.739, 4.203, 0.000),Quaternion(0.000, 0.000, -0.716, 0.698)), 'mode': 1 },

'init_pose':{'pos': Pose(Point(9.122, 1.874, 0.000),Quaternion(0.000, 0.000, -0.048, 0.999)), 'mode': 1 },
'point_1':{'pos': Pose(Point(2.603,2.223,0.000),Quaternion(0.000,0.000,-0.029,0.999)),'mode':1},
'point_2':{'pos': Pose(Point(5.397,1.894,0.000),Quaternion(0.000,0.000,-0.989,-0.149)),'mode':1},
'point_3':{'pos': Pose(Point(4.499,3.749,0.000),Quaternion(0.000,0.000,-0.167,0.986)),'mode':1},
'point_4':{'pos': Pose(Point(2.157,4.546,0.000),Quaternion(0.000,0.000,-0.329,0.944)),'mode':1},
'kitchen_turn':{'pos':Pose(Point(2.052,4.791,0.000),Quaternion(0,0,-0.736,0.677)),'mode':1},

'cooking_table': {'pos': Pose(Point(3.058, 5.605, 0.000),Quaternion(0.000, 0.000, 0.707, 0.708)), 'mode': 1 },
'TV_table': {'pos': Pose(Point(4.138, 2.179, 0.000),Quaternion(0.000, 0.000, -0.639, 0.770)), 'mode': 1 },
'book-cabinet': {'pos': Pose(Point(10.200, 1.910, 0.000),Quaternion(0.000, 0.000, 0.724, 0.690)), 'mode': 1 },
'sprite': {'pos': Pose(), 'mode': 1 },
'red-bull': {'pos': Pose(), 'mode': 1 },


'tea': {'pos': Pose(), 'mode': 1 },
'juice': {'pos': Pose(), 'mode': 1 },
'coffee': {'pos': Pose(), 'mode': 1 },
'biscuit': {'pos': Pose(), 'mode': 1 },
'chips': {'pos': Pose(), 'mode': 1 },
'roll-paper': {'pos': Pose(), 'mode': 1 },
'toothpaste': {'pos': Pose(), 'mode': 1 },


'soap':{'pos': Pose(), 'mode': 1 },
'red bull':{'pos': Pose(), 'mode': 2 },
'coconut':{'pos': Pose(), 'mode': 2 },
'green bean':{'pos': Pose(), 'mode': 2 },
'cola':{'pos': Pose(), 'mode': 2 },
'tooth brush':{'pos': Pose(), 'mode': 1 },
'milk tea':{'pos': Pose(), 'mode': 2 },
'paper':{'pos': Pose(), 'mode': 1 },

#########################      help_me_carry          #############
'door':{'pos' : Pose(Point(0.000 , 0.000 ,0.000), Quaternion(0.000, 0.000, 0.000 , 1.000)),'mode': 1},
'out_door': {'pos': Pose(Point( 11.738,6.717, 0.000), Quaternion(0.000, 0.000, 0.693, 0.721)), 'mode': 1 },

#'out_door':{'pos' : Pose(Point(0.000 , 0.000 ,0.000), Quaternion(0.000, 0.000, 0.000 , 1.000)),'mode': 1},
'car':{'pos':Pose(Point(0.000 , 0.000 , 0.000),Quaternion(0.000, 0.000, 0.000 , 1.000)),'mode':1},
########################   store ############################

'place':{'pos':Pose(Point(0.269, 2.938,0.000),Quaternion(0.000,0.000,-0.013, 1.00)),'mode':1},
#'pick':{'pos': Pose(Point(0.000 , 0.000 , 0.000), Quaternion(0.000, 0.000, 0.000, 1.000)), 'mode': 1 },
'pick':{'pos': Pose(Point(-0.168 , -0.026 , 0.000), Quaternion(0.000, 0.000, -0.019, 0.998)), 'mode': 1 },

'exit':{'pos' : Pose(Point(10.899, 9.309 , 0.000), Quaternion(0.000,0.000,0.726,0.688)) , 'mode' : 1}

}
