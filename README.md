# cyberdog_ws

## 四足开发平台—项目介绍

### 简介

本项目是基于小米铁蛋四足开发者平台的主要功能包.

### 基本信息

- 铁蛋默认用户是`mi`, 密码为`123`
- 使用USB线连接`Download`接口, 可通过`ssh mi@192.168.55.1`连接铁蛋进行内部操作

### 参考文档
架构设计请参考： [平台架构](https://miroboticslab.github.io/blogs/#/cn/cyberdog_platform_software_architecture_cn)

详细文档请参考：[项目博客文档](https://miroboticslab.github.io/blogs/#/)

## 运行步骤

### 代码下载

该仓库为四组开发平台项目代码主仓库，拉取该仓库代码，进入cyberdog_ws目录，可以用以下命令进行子模块展开：

```
vcs import . < cyberdog.repos
```

### 镜像编译

进入tools目录下，可使用Dockerfile文件编译镜像，具体步骤可参考：[镜像编译](https://github.com/MiRoboticsLab/blogs/blob/rolling/docs/cn/dockerfile_instructions_cn.md)

## 文件夹说明

- cyberdog_bringup： 启动脚本及配置。
- tools： 开发调试工具脚本（不跟随版本发布）。
- thirdparty：[第三方库(源码形式引入)](https://github.com/MiRoboticsLab/blogs/blob/rolling/docs/cn/third_party_library_management_cn.md)。
