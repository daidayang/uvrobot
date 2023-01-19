## launch参数说明
- ws_port: WebSocket协议的监听端口；
- debug_flag: 控制程序内调试用的打印语句是否输出，置为1输出；置为0不输出；
- robot_position_topic_name: 报告当前机器人位置的topic名；
- remote_control_topic_name: 机器人远程控制命令的目的topic名；
                            即本节点收到的远程控制命令会发送到
                            remote_control_topic_name指定
                            的topic上；
- patrol_status_topic_name: 消毒巡逻任务topic名；即本节点会从
                            patrol_status_topic_name指定的topic
                            上接收当前巡逻消毒任务的状态，用于更新数据库内
                            当前巡逻消毒任务的状态；
- clean_task_topic_name: 消毒巡逻任务设定的目的topic名；即本节点接收到
                            的消毒巡逻任务信息会发送到clean_task_topic_name
                            指定的topic上；
- report_status_url: 服务器用于接收当前消毒巡逻任务状态的API地址；
- map_width: 当前地图对应图片的宽度，单位像素；
- map_height: 当前地图对应图片的高度，单位像素；
- map_origin_x: 当前地图对应图片左下角在地图中的横坐标值，单位为米；
- map_origin_y: 当前地图对应图片左下角在地图中的纵坐标值，单位为米；
- map_resolution: 地图分辨率，即地图对应图片中一个像素代表地图中多少一段距离；单位为米/像素;
- csi_or_usb: 选择使用的相机类型，置为1使用USB相机，置为0使用CSI相机；
- cam_id: 使用USB相机时，用于指定相机的ID

## Activation

``` shell
$ roslaunch wsserver wsserver.launch
```

## Requirements
- [OpenCV 3.3.1](https://opencv.org/releases/)
- [JSON](https://github.com/nlohmann/json)
- [Websocketpp](https://github.com/zaphoyd/websocketpp)
- [curl-7.58.0](https://github.com/curl/curl/releases)