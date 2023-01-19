## launch参数说明
- debug_flag: 控制程序内调试用语句的输出，置为1输出；置为0不输出；
- costmap_topic: costmap的topic名；
- robot_pose_topic: 发出机器人当前位置的topic名；
- detection_res_topic: 障碍物检测的结果会发送到detection_res_topic指定的topic名上；
- uvrobot_radius: 设定机器人自身的几何尺寸；
- map_resolution: 地图分辨率，即地图对应图片中一个像素代表地图中多少一段距离；单位为米/像素;
- origin_x: 当前地图对应图片左下角在地图中的横坐标值，单位为米；
- origin_y: 当前地图对应图片左下角在地图中的纵坐标值，单位为米；
- map_w: 当前地图对应图片的宽度，单位像素；
- map_h: 当前地图对应图片的高度，单位像素；

## Activation

``` shell
$ roslaunch obstacle_detection obstacle_detection.launch
```

## Requirements

``` shell
$ roslaunch move_base move_localization.launch
$ roslaunch move_base move_navigation.launch
$ roslaunch move_base move_navigation_bag.launch
```