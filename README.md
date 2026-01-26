<img  src="resource/logo.png" width="800">

# LinkerHand灵巧手ROS2 SDK For R20

# 1. **概述**
LinkerHand灵巧手ROS SDK 是灵心巧手(北京)科技有限公司开发，用于R20等LinkerHand灵巧手的驱动软件和功能示例源码。可用于真机与仿真器使用。
LinkerHandROS2 SDK当前支持Ubuntu22.04 ROS humble Python3.10 及以上环境



| Name | Version | Link |
| --- | --- | --- |
| Python SDK | ![SDK Version](https://img.shields.io/badge/SDK%20Version-V3.0.1-brightgreen?style=flat-square) ![Python 3.8+](https://img.shields.io/badge/Python-3.8%2B-blue?style=flat-square&logo=python&logoColor=white) ![Windows 11](https://img.shields.io/badge/OS-Windows%2011-0078D4?style=flat-square&logo=windows&logoColor=white) ![Ubuntu 20.04+](https://img.shields.io/badge/OS-Ubuntu%2020.04%2B-E95420?style=flat-square&logo=ubuntu&logoColor=white) | [![GitHub 仓库](https://img.shields.io/badge/GitHub-grey?logo=github&style=flat-square)](https://github.com/linker-bot/linker-hand-r20-ros2.git) |
| ROS2 SDK | ![SDK Version](https://img.shields.io/badge/SDK%20Version-V3.0.1-brightgreen?style=flat-square) ![Python 3.11](https://img.shields.io/badge/Python-3.11-3776AB?style=flat-square&logo=python&logoColor=white) ![Ubuntu 24.04](https://img.shields.io/badge/OS-Ubuntu%2024.04-E95420?style=flat-square&logo=ubuntu&logoColor=white) ![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-00B3E6?style=flat-square&logo=ros) ![Windows 11](https://img.shields.io/badge/OS-Windows%2011-0078D4?style=flat-square&logo=windows&logoColor=white) | [![GitHub 仓库](https://img.shields.io/badge/GitHub-grey?logo=github&style=flat-square)](https://github.com/linker-bot/linker-hand-r20-ros2.git) |


# 2. **警告**

1. 请保持远离灵巧手活动范围，避免造成人身伤害或设备损坏。

2. 执行动作前请务必进行安全评估，以防止发生碰撞。

3. 请保护好灵巧手。

# 3. **版本说明**

V1.0.0
1. 支持R20版Linker Hand ROS2
2. GUI控制界面，带有手指舞动作
3. WIN Python Demo示例 



# 4. **准备工作**

## 4.1 系统与硬件需求

* 操作系统：Ubuntu24.04

* ROS2版本：Jazzy

* Python版本：V3.12

* 硬件：amd64_x86/arm64 配备 USB CANFD


将libcanbus用命令解压到/usr/local/lib/目录下面

tar -xvf libcanbus.tar -C /usr/local/lib/

配置环境变量:      export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib

libcanbus.a为静态库，libcanbus.so为共享库

libcanbus_arm        编译器为    arm-linux-gnueabihf-gcc
libcanbus_arm64      编译器为    aarch64-linux-gnu-gcc
libcanbus(ubuntu20)  编译器为    gcc version 9.4.0
libcanbus(ubuntu22)  编译器为    gcc version 11.3.0


编译时先加载libcanbus库，再加载libusb库(源码注释里面有编译方法)。

如果编译提示pthread相关错误，说明libcanbus.tar
自带的libusb库不匹配， 请安装libusb库        sudo apt-get install libusb-1.0-0-dev 并删掉/usr/local/lib目录下的libusb相关库文件

如果提示ludev错误，请安装libudev-dev：       sudo apt-get install libudev-dev

如果编译过程中找不到cc1plus 请安装           sudo apt-get install --reinstall build-essential


注意：root用户可直接读写usbcan设备，非root用户，需要修改usbcan模块的操作权限，可百度搜索修改方法。
或者尝试将99-canfd.rules文件放到 /etc/udev/rules.d/， 然后执行
```bash
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
```
重启系统。



## 4.2 下载

```bash
$ mkdir -p linker_hand_r20_ros2/src    #创建目录
$ cd linker_hand_r20_ros2/src    #进入目录
$ git clone https://github.com/linker-bot/linker-hand-r20-ros2.git
```

## 4.3 安装依赖与编译

```bash
$ cd linker_hand_r20_ros2/src/linker_hand_r20_ros2    #进入目录
$ pip install -r requirements.txt    #安装所需依赖
$ cd linker_hand_r20_ros2 # 回到工程目录
$ colcon build --symlink-install    #编译和构建ROS包
```

# 5 启动SDK
```bash
$ cd linker_hand_r20_ros2
$ source ./install/setup.bash
$ ros2 run linker_hand_r20_ros2_sdk linker_hand_r20_ros2_sdk
$ # 显示一下信息则连接成功
$ [INFO] [1769408666.004287900] [linker_hand_r20]: Linker Hand R20 ROS2 SDK 连接成功
$ ........
```
## 5.1启动GUI控制R20灵巧手
 - 启动SDK后，新开一个终端。注:由于有界面，不能使用ssh远程连接开启GUI 或者使用X11 终端
 - 修改gui_control/launch/gui_control.launch.py 按照参数说明，修改左手 or 右手
```bash
$ cd linker_hand_r20_ros2
$ source ./install/setup.bash
$ ros2 launch gui_control gui_control.launch.py
```
<img  src="resource/gui.png" width="550">


# 6 TOPIC说明
```bash
/cb_right_hand_control_cmd # 控制右手运动话题  (0~255)
/cb_right_hand_state # 实时当前状态 0-255
/cb_right_hand_state_arc # 实时当前状态角度值
```
## 6.1 position说明
```bash
["拇指根部", "食指根部", "中指根部", "无名指根部", "小指根部", "拇指侧摆", "食指侧摆", "中指侧摆", "无名指侧摆", "小指侧摆", "拇指旋转", "预留", "预留", "预留", "预留", "拇指尖部", "食指末端", "中指末端", "无名指末端", "小指末端"]
```


## 7 WIN上位机使用方法
 - 接通电源，USB转CANFD插在WIN11上位机下直接运行"R20灵巧手控制系统for win.exe"即可
