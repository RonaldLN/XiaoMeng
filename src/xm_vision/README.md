图像代码都在src/scripts

不带ros的脚本：
mrsupw_detect_object.py
mrsupw_following_person.py
运行命令：
python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_detect_object.py -d true
python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_following_person.py -d true


带ros的脚本：
mrsupw_vison_publisher.py
mrsupw_vison_server.py
运行命令：
python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_vison_publisher.py -d true
python3 /home/xm/catkin_ws/src/xm_vision/src/scripts/mrsupw_vison_server.py -d true


以上命令中 -d true 是用来控制是否显示摄像头图像的 
调试的时候可以加上 不调试则不用
