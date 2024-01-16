- 这个功能包中的```xm_arm_bringup.launch```启动了机械臂的所有控制器或moveit的demo.launch。

- 此外还启动了```xm_arm_stack_server.py```节点，用于提供自己设计的解算。

- 由于```xm_arm_stack_server```与控制器直接交互，故与moveit的自己的解算并不冲突。

- 同时，为了确定误差的来源，需要将图像发送的需要抓取的物体坐标显示的发送出去，在rviz上显示，来判断误差的来源

- 在```arm_stack_MathematicalPrinciples```文件夹中，有关于其解算的数学解释