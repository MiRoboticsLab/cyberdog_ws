# cyberdog_ws

## 项目名称
本项目是基于小米铁蛋四足开发者平台的主要功能包。
## 仓库介绍
该仓库为四足开发平台项目代码主仓库，拉取该仓库代码，进入cyberdog_ws目录，可以用以下命令进行子模块展开：

```
vcs import . < cyberdog.repos
```
备注：vcstools 是一个用于管理多个版本控制系统（Version Control System）的软件包。具体可参考https://github.com/dirk-thomas/vcstool



| 仓库名称               | 仓库地址                                                | 主要功能                                                     | 设计文档                                                     |
| ---------------------- | ------------------------------------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| cyberdog_ws            | https://github.com/MiRoboticsLab/cyberdog_ws            | 启动模块                                                     | [启动模块](https://miroboticslab.github.io/blogs/#/cn/cyberdog_bringup_cn)<br/> |
| manager                | https://github.com/MiRoboticsLab/manager                | 全局管理节点                                                 | [管理模块](https://miroboticslab.github.io/blogs/#/cn/cyberdog_manager_cn)<br/> |
| bridges                | https://github.com/MiRoboticsLab/bridges                | ros消息服务定义文件<br/>与app端通讯程序<br/>can数据收发封装库 | [grpc通信模块](https://miroboticslab.github.io/blogs/#/cn/cyberdog_grpc_cn) |
| devices                | https://github.com/MiRoboticsLab/devices                | 设备管理节点<br/>bms数据发布插件<br/>led设置插件<br/>touch插件<br/>uwb插件 | [设备管理模块](https://miroboticslab.github.io/blogs/#/cn/device_manager_cn)<br/>[bms模块](https://miroboticslab.github.io/blogs/#/cn/cyberdog_bms_cn)<br/>[LED模块](https://miroboticslab.github.io/blogs/#/cn/cyberdog_led_cn)<br/>[touch模块](https://miroboticslab.github.io/blogs/#/cn/cyberdog_touch_cn)<br/>[uwb模块](https://miroboticslab.github.io/blogs/#/cn/cyberdog_uwb_cn)<br/> |
| motion                 | https://github.com/MiRoboticsLab/motion                 | 运控管理                                                     | [运动管理模块](https://miroboticslab.github.io/blogs/#/cn/motion_manager_cn)<br/> |
| sensors                | https://github.com/MiRoboticsLab/sensors                | 传感器节点<br/>gps插件<br/>雷达插件<br/>tof插件<br/>超声插件 | [传感器模块](https://miroboticslab.github.io/blogs/#/cn/sensor_manager_cn)<br/>[gps模块](https://miroboticslab.github.io/blogs/#/cn/cyberdog_gps_cn)<br/>[雷达模块](https://miroboticslab.github.io/blogs/#/cn/cyberdog_lidar_cn)<br/>[tof模块](https://miroboticslab.github.io/blogs/#/cn/cyberdog_tof_cn)<br/>[超声模块](https://miroboticslab.github.io/blogs/#/cn/cyberdog_ultrasonic_cn)<br/> |
| interaction            | https://github.com/MiRoboticsLab/interaction            | 语音节点<br/>可视化编程节点<br/>小爱训练词节点<br/>图传节点<br/>快连节点 | [语音模块]()<br/>[可视化编程模块]()<br/>[语音训练词模块]()<br/>[图传模块]()<br/>[快连模块]()<br/> |
| cyberdog_nav2          | https://github.com/MiRoboticsLab/cyberdog_nav           | 算法任务管理相关                                             | [算法任务管理](https://miroboticslab.github.io/blogs/#/cn/algorithm_manager_cn)<br/> |
| cyberdog_tracking_base | https://github.com/MiRoboticsLab/cyberdog_tracking_base | 存放了基于navigation2实现的docking， navigation， tracking功能相关的参数<br/>附加模块等 |                                                              |
| utils                  | https://github.com/MiRoboticsLab/utils                  | 通用接口库                                                   | [通用接口库](https://miroboticslab.github.io/blogs/#/cn/cyberdog_common_cn)<br/> |



## 安装使用

进入tools目录下，可使用Dockerfile文件编译镜像，具体步骤可参考：[镜像编译](https://github.com/MiRoboticsLab/blogs/blob/rolling/docs/cn/dockerfile_instructions_cn.md)

## 使用示例
可作为开发用参考demo，非原生功能

| demo名称                | 功能描述     | github地址                                             |
| ----------------------- | ------------ | ------------------------------------------------------ |
| grpc_demo               | grpc通信例程 | https://github.com/WLwind/grpc_demo                    |
| audio_demos             | 语音例程     | https://github.com/jiayy2/audio_demos                  |
| cyberdog_ai_sports_demo | 运动计数例程 | https://github.com/Ydw-588/cyberdog_ai_sports_demo     |
| cyberdog_face_demo      | 人脸识别demo | https://github.com/Ydw-588/cyberdog_face_demo          |
| nav2_demo               | 导航         | https://github.com/duyongquan/nav2_demo                |
| cyberdog_vp_demo        | 可视化编程   | https://github.com/szh-cn/cyberdog_vp_demo             |
| cyberdog_action_demo    | 手势动作识别 | https://github.com/liangxiaowei00/cyberdog_action_demo |


## 文档

架构设计请参考： [平台架构](https://miroboticslab.github.io/blogs/#/cn/cyberdog_platform_software_architecture_cn)

详细文档请参考：[项目博客文档](https://miroboticslab.github.io/blogs/#/)

## 版权和许可

四足开发者平台遵循Apache License 2.0 开源协议。详细的协议内容请查看 [LICENSE.txt](./LICENSE.txt)

thirdparty：[第三方库](https://github.com/MiRoboticsLab/blogs/blob/rolling/docs/cn/third_party_library_management_cn.md)

## 联系方式

dukun1@xiaomi.com

liukai21@xiaomi.com

tianhonghai@xiaomi.com

wangruheng@xiaomi.com
