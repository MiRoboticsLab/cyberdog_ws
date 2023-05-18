# cyberdog_ws

## 项目名称
本项目是基于小米铁蛋四足开发者平台的主要功能包。
## 仓库介绍
该仓库为四组开发平台项目代码主仓库，拉取该仓库代码，进入cyberdog_ws目录，可以用以下命令进行子模块展开：

```
vcs import . < cyberdog.repos
```
| 仓库名称               | 仓库地址                                                | 主要功能                                                     |
| ---------------------- | ------------------------------------------------------- | ------------------------------------------------------------ |
| cyberdog_ws            | https://github.com/MiRoboticsLab/cyberdog_ws            | 传感器管理节点<br/>gps数据发布插件<br/>雷达数据发布插件<br/>tof数据发布插件<br/>超声数据发布插件 |
| manager                | https://github.com/MiRoboticsLab/manager                | 全局管理节点<br/>信息上传节点<br/>解锁节点<br/>账户管理接口<br/>低功耗接口 |
| bridges                | https://github.com/MiRoboticsLab/bridges                | ros消息服务定义文件<br/>与app端通讯程序<br/>can数据收发封装库 |
| devices                | https://github.com/MiRoboticsLab/devices                | 设备管理节点<br/>bms数据发布插件<br/>led设置插件<br/>touch插件<br/>uwb插件 |
| motion                 | https://github.com/MiRoboticsLab/motion                 | 运控管理                                                     |
| sensors                | https://github.com/MiRoboticsLab/sensors                | 传感器节点<br/>gps插件<br/>雷达插件<br/>tof插件<br/>超声插件 |
| interaction            | https://github.com/MiRoboticsLab/interaction            | 语音节点<br/>可视化编程节点<br/>小爱训练词节点<br/>图传节点<br/>快连节点 |
| cyberdog_nav2          | https://github.com/MiRoboticsLab/cyberdog_nav           | 导航代码                                                     |
| cyberdog_tracking_base | https://github.com/MiRoboticsLab/cyberdog_tracking_base | 存放了基于navigation2实现的docking， navigation， tracking功能相关的参数<br/>附加模块等 |
| utils                  | https://github.com/MiRoboticsLab/utils                  | 通用接口库                                                   |


## 安装使用

进入tools目录下，可使用Dockerfile文件编译镜像，具体步骤可参考：[镜像编译](https://github.com/MiRoboticsLab/blogs/blob/rolling/docs/cn/dockerfile_instructions_cn.md)

## 使用示例
可作为开发用参考demo，非原生功能
待补充


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
