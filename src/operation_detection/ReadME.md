## launch参数说明

- debug_flag: 用于控制程序调试用语句的输出，置为1输出；置为0不输出；
- cmd_vel_sub_topic_name: 车辆控制命令的topic名；
- robot_pose_topic_name: 机器人当前位置的topic名；
- op_det_pub_topic_name: 用于输出当前机器人的是否按照命令运行的检测结果topic名；
- threshold_decision: 判断机器人是否根据控制命令发生位移的阈值；
- span_time: 该参数用于指定间隔多久之后检测机器人的位置，单位为秒；

## Activation

``` shell
$ roslaunch operation_detection operation_detection.launch
```

## Requirements

``` shell
$ roslaunch move_base move_localization.launch
$ roslaunch move_base move_navigation.launch
$ roslaunch move_base move_navigation_bag.launch (if test based on a bag file)
```
