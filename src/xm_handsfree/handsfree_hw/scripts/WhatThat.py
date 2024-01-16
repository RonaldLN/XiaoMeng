#!/usr/bin/env python3
# encoding:utf8
import pyrealsense2 as rs
import cv2
import numpy as np
import tensorflow as tf
import tensorflow_hub as hub
import rospy
from speech.srv import *
from geometry_msgs.msg import *
from math import *
import time
from aip import AipBodyAnalysis

map_number = {
    1:"bread",
    2:'lays',
    3:'dishsoap',
    4:'chip',
    5:'handwash',
    6:'shampoo',
    7:'biscuit',
    8:'cola',
    9:'orange juice',
    10:'water',
    11:'sprite',
    12:'cookie'
}
object_range2 = [
    [map_number[8] , map_number[11] ,map_number[7]] , 
    [map_number[3] , map_number[12] , map_number[10]] ,
    [map_number[9],map_number[10],map_number[1] ] ,
    [map_number[4] , map_number[6] , map_number[5]] ,
    [map_number[2] , map_number[11] , map_number[8]]
]

object_map = {"bread":'the bread is soft',
    'lays':'the lays is crisp',
    'dishsoap':'the dishsoap is used in kitchen',
    'chip':'the chip is fryied',
    'handwash':'the handwish is used for hand',
    'shampoo': 'the shampoo is used for hair',
    'biscuit':'the biscuit is squared',
    'cola':'the cola is black',
    'orange juice':'the orange juice is orange',
    'water':'the sprite is tastless',
    'sprite':'the spirte is lemon',
    'cookie':'the cookie is green star'}




width = 640
height = 480
fps = 30

twist_xm = None

up_or_down = True

left_num = 0
right_num = 0

is_follow = False
is_follow_tts = False

cmd_publish = None
speech_client = None

APP_ID = '27385492'
API_KEY = 'bWPSTkU52188A1akPfPZAvnw'
SECRET_KEY = '9yvArkKFrxIQqLMeOe1BZBliSkGxVZbt'



def detect(movenet , frame):
    frame = cv2.resize(frame, (640, 480))
    cv2.imwrite('temp.jpg', frame)
    img = tf.io.read_file('temp.jpg')
    img = tf.image.decode_jpeg(img)
    img = tf.expand_dims(img, axis=0)
    img = tf.cast(tf.image.resize_with_pad(img , 256 , 256) , dtype=tf.int32)

    outputs = movenet(img)
    keypoints_with_scores = outputs['output_0']
    return darwPoint(frame, np.array(keypoints_with_scores[0]).tolist())
    # return (x , y)
 
def darwPoint(img , *args):
    global  left_num , right_num
    baseData = args[0]
    for imgPoint in baseData:
        Confidence = 0
        for i in range(0, 51, 3):
            Confidence = Confidence + imgPoint[i + 2]
            # ##print(imgPoint[i + 2])
        # ##print(imgPoint)
        # ##print(Confidence / 17)
        if Confidence / 17 < 0.3:
            cv2.imshow("img", img)
            continue
        # for i in range(0, 51, 3):
        #     # ##print(i)
        #     # ##print(imgPoint[i], imgPoint[i+1])
        #     cv2.circle(img, (int(640 * imgPoint[i + 1]), int(480 * imgPoint[i])), 1, (0, 0, 255), 1)
        #     cv2.putText(img, str(imgPoint[i + 2]), (int(640 * imgPoint[i + 1]), int(480 * imgPoint[i])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.rectangle(img, (int(640 * imgPoint[-4]) , int(480 * imgPoint[-5])), (int(480 * imgPoint[-2]) , int(640 * imgPoint[-3])), (0, 0, 255), 1)
        cv2.line(img, (int(640 * imgPoint[1]) , int(480 * imgPoint[0])), ( int(640 * imgPoint[4]) , int(480 * imgPoint[3])),
                    (0, 255, 0), 2) # 鼻子和左眼
        cv2.line(img, ( int(640 * imgPoint[1]) ,int(480 * imgPoint[0])), ( int(640 * imgPoint[7]) , int(480 * imgPoint[6])),
                    (0, 255, 0), 2) # 鼻子和右眼
        cv2.line(img, ( int(640 * imgPoint[4]) , int(480 * imgPoint[3])), ( int(640 * imgPoint[10]) ,int(480 * imgPoint[9])),
                    (0, 255, 0), 2) # 左眼和左耳
        cv2.line(img, ( int(640 * imgPoint[7]) , int(480 * imgPoint[6])), ( int(640 * imgPoint[13]) ,int(480 * imgPoint[12])),
                    (0, 255, 0), 2) # 右眼和右耳
        cv2.line(img, ( int(640 * imgPoint[1]) , int(480 * imgPoint[0])), ( int(640 * imgPoint[16]) ,int(480 * imgPoint[15])),
                    (0, 255, 0), 2) # 鼻子和左肩
        cv2.line(img, ( int(640 * imgPoint[1]) , int(480 * imgPoint[0])), ( int(640 * imgPoint[19]) ,int(480 * imgPoint[18])),
                    (0, 255, 0), 2) # 鼻子和右肩
        cv2.line(img, ( int(640 * imgPoint[16]) , int(480 * imgPoint[15])), ( int(640 * imgPoint[22]) ,int(480 * imgPoint[21])),
                    (0, 255, 0), 2) # 左肩和左肘
        cv2.line(img, ( int(640 * imgPoint[19]) , int(480 * imgPoint[18])), ( int(640 * imgPoint[25]) ,int(480 * imgPoint[24])),
                    (0, 255, 0), 2) # 右肩和右肘
        cv2.line(img, ( int(640 * imgPoint[22]) , int(480 * imgPoint[21])), ( int(640 * imgPoint[28]) ,int(480 * imgPoint[27])),
                    (0, 255, 0), 2) # 左肘和左手
        cv2.line(img, ( int(640 * imgPoint[25]) , int(480 * imgPoint[24])), ( int(640 * imgPoint[31]) ,int(480 * imgPoint[30])),
                    (0, 255, 0), 2) # 右肘和右手
        cv2.line(img, ( int(640 * imgPoint[16]) , int(480 * imgPoint[15])), ( int(640 * imgPoint[34]) ,int(480 * imgPoint[33])),
                    (0, 255, 0), 2) # 左肩和左腰
        cv2.line(img, ( int(640 * imgPoint[19]) , int(480 * imgPoint[18])), ( int(640 * imgPoint[37]) ,int(480 * imgPoint[36])),
                    (0, 255, 0), 2) # 右肩和右腰
        cv2.line(img, ( int(640 * imgPoint[34]) , int(480 * imgPoint[33])), ( int(640 * imgPoint[40]) ,int(480 * imgPoint[39])),
                    (0, 255, 0), 2) # 左腰和左膝
        cv2.line(img, ( int(640 * imgPoint[37]) , int(480 * imgPoint[36])), ( int(640 * imgPoint[43]) ,int(480 * imgPoint[42])),
                    (0, 255, 0), 2) # 右腰和右膝
        cv2.line(img, ( int(640 * imgPoint[40]) , int(480 * imgPoint[39])), ( int(640 * imgPoint[46]) ,int(480 * imgPoint[45])),
                    (0, 255, 0), 2) # 左膝和左脚
        cv2.line(img, ( int(640 * imgPoint[43]) , int(480 * imgPoint[42])), ( int(640 * imgPoint[49]) ,int(480 * imgPoint[48])),
                    (0, 255, 0), 2) # 右膝和右脚
        if (imgPoint[18] - imgPoint[24] ) / (imgPoint[19] - imgPoint[25]) > 0 and (imgPoint[30] - imgPoint[24]) / (imgPoint[31] - imgPoint[25]) > 0:
            cv2.putText(img, "right hand up", (0 , 0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            right_num += 1
        else:
            right_num = 0
        if (imgPoint[15] - imgPoint[21] ) / (imgPoint[16] - imgPoint[22]) < 0 and (imgPoint[27] - imgPoint[21]) / (imgPoint[28] - imgPoint[22]) < 0:
            cv2.putText(img, "left hand up", (0 , 0), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            left_num += 1
        else:
            left_num = 0
        cv2.imshow("img", img)
        ##print((((imgPoint[-4] + imgPoint[-2]) / 2) , ((imgPoint[-3] + imgPoint[-5]) / 2)))
    
        return True , (imgPoint[-4] + imgPoint[-2]) / 2 , (imgPoint[-3] + imgPoint[-5]) / 2
    return False , 0 , 0

def publish(publisher , x , y):
    cmd = Twist()
    person_x = x
    person_y = y
    if person_x == 0 or person_y == 0:
        return
    angle = atan(person_y / person_x)
    angle_rate = 2
    
    cmd.angular.z = 0.0
    cmd.linear.x = 0.0
    if 0.2 < angle < 1.0:
        #twist_xm.angular.z = 0.2
        cmd.angular.z = angle / angle_rate
        # if person_x < 0.6:
        # twist_xm.linear.x = -0.25
        if person_x > 2.2:
            cmd.linear.x = 0.35
        elif 1.6 < person_x < 2.2:
            #twist_xm.linear.x = 0.2
            cmd.linear.x = person_x / 4
    elif -0.2 > angle > -1:
        #twist_xm.angular.z = -0.2
        cmd.angular.z = angle / angle_rate
        # if person_x < 0.6:
        # twist_xm.linear.x = -0.25
        if person_x > 2.2:
            cmd.linear.x = 0.35
        elif 1.6 < person_x < 2.2:
            #twist_xm.linear.x = 0.2
            cmd.linear.x = person_x / 4
    elif angle > 1.0:
        cmd.angular.z = 0.4
        if person_x > 2.2:
            cmd.linear.x = 0.35
        elif 1.6 < person_x < 2.2:
            #twist_xm.linear.x = 0.2
            cmd.linear.x = person_x / 4
    elif angle < -1.0:
        cmd.angular.z = -0.4
        if person_x > 2.2:
            cmd.linear.x = 0.35
        elif 1.6 < person_x < 2.2:
            #twist_xm.linear.x = 0.2
            cmd.linear.x = person_x / 4
    else:
        # if person_x < 0.6:
        # twist_xm.linear.x = -0.25
        if person_x > 2.2:
            cmd.linear.x = 0.35
        elif 1.6 < person_x < 2.2:
            #twist_xm.linear.x = 0.2
            cmd.linear.x = person_x / 4

    # if preempt_requested():
    #    rospy.logerr('preemted')
    #    nav_client.send_goal(goal)
    # nav_client.cancel_goal()
    #    return 'aborted'
    if cmd.angular.z >= 0.15:
        cmd.angular.z = 0.15
    print((x , y))
    print(cmd)
    publisher.publish(cmd)

if __name__ == '__main__':
    rospy.init_node("object_detect")
    rsConfig = rs.config()
    rsConfig.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    rsConfig.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    rsConfig.enable_stream(rs.stream.infrared, 1, width, height, rs.format.y8, fps)
    rsConfig.enable_stream(rs.stream.infrared, 2, width, height, rs.format.y8, fps)
    up_pipeline = rs.pipeline()
    up_pipeline.start(rsConfig)

    model = hub.load("movenet_multipose_lightning_1")
    movenet = model.signatures['serving_default']

    #speech_client = rospy.ServiceProxy('speech_core', speech_to_smach)
    #speech_client.wait_for_service(timeout=10.0)
    #speech_client.call(2, "Now , I am ready to win champion.")  
    print("Now , I am ready to win champion.")

    cmd_publish = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    handClient = AipBodyAnalysis(APP_ID , API_KEY ,SECRET_KEY)

    number_ = 0

    while True:
        if up_or_down:
            up_frames = up_pipeline.wait_for_frames()
            up_color_frame = up_frames.get_color_frame()
            up_color_image = np.asanyarray(up_color_frame.get_data())
            depth_frame = up_frames.get_depth_frame()
            depth_data  = np.asanyarray(depth_frame.get_data())
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            is_detect , x , y = detect(movenet, up_color_image)
            if is_detect:
                x , y =int( x*640) ,int( y*480)
                dis = depth_frame.get_distance( int(x) ,int( y))
                camera_xyz = rs.rs2_deproject_pixel_to_point(depth_intrin , (x , y) ,dis)
                camera_xyz = np.round(np.array(camera_xyz) , 3)
                camera_xyz = camera_xyz.tolist()
                if is_follow:
                    publish(cmd_publish  , camera_xyz[2] , -camera_xyz[0])
            c = cv2.waitKey(10) & 0xff
            if c == 27:
                # 简单暴力释放所有窗口
                cv2.destroyAllWindows()
                break
            
            if is_follow_tts == False and is_follow == True:
                # speech_client.call(2 , "Now , I will follow you")
                is_follow_tts = True
            if right_num > 100:
                right_num = 0
                up_or_down = False
                is_follow = False
                is_follow_tts = False
                ##print("up")
                #speech_client.call(2 , "Now , I will detect object")
                print("Now , I will detect object")
                cv2.destroyAllWindows()

            if left_num > 100:
                left_num = 0
                up_or_down = True
                is_follow = True
                ##print("left")
                #speech_client.call(2 , "Now , I will follow you")
                print("Now , I will follow you")
        else:
            up_frames = up_pipeline.wait_for_frames()
            up_color_frame = up_frames.get_color_frame()
            up_color_image = np.asanyarray(up_color_frame.get_data())
            depth_frame = up_frames.get_depth_frame()
            depth_data  = np.asanyarray(depth_frame.get_data())

            # img = up_color_image[up_color_image.shape[0] / 4 :up_color_image.shape[0] * 3 / 4][up_color_image.shape[1] / 4 :up_color_image.shape[1] * 3 / 4]

            cv2.imwrite("test.jpg" , up_color_image)
            f = open("test.jpg" , "rb")

            result = handClient.gesture(f.read())
            f.close()
            print(result)
            if "result" in result:
                for name in result["result"]:
                    if name["classname"] == "One":
                        up_or_down = True
                        str1 = object_map[object_range2[number_][0]]
                        #speech_client.call(2 , "This is " + object_range2[number_][0] + str1)
                        print("This is " + object_range2[number_][0] + str1)
                        number_ += 1
                        print(object_range2[number_][0])
                        print("================================")
                    elif  name["classname"] == "Ok":
                        up_or_down = True
                        str1 = object_map[object_range2[number_][1]]
                        #speech_client.call(2 , "This is" +object_range2[number_][1] + str1)
                        print("This is" +object_range2[number_][1] + str1)
                        number_+=1
                        print(object_range2[number_][1])
                        print("================================")
                    elif name["classname"] == "Five":
                        up_or_down = True
                        str1 = object_map[object_range2[number_][2]]
                        #speech_client.call(2 , "This is" +object_range2[number_][2] + str1)
                        print("This is" +object_range2[number_][2] + str1)
                        number_ +=1 
                        print(object_range2[number_][2])
                        print("================================")
            time.sleep(0.5)
            if number_ == 4:
                number_ = 0





















