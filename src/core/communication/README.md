# rm_serial_driver

RoboMaster 算法系统与电控系统的串口通讯模块

该项目为 [pb_rm_vision](https://gitee.com/SMBU-POLARBEAR/PB_RM_Vision) 的子模块

当前分支为**哨兵机器人分支**，主要改动为 rm_serial_driver，为适配决策树，添加了多种自定义消息的串口传输。与 [rm_behavior_tree](https://gitee.com/SMBU-POLARBEAR/rm_behavior_tree) 存在强依赖关系

## Overview

本模块基于 [transport_drivers](https://github.com/ros-drivers/transport_drivers) 实现了上位机与电控部分通讯的功能

## 使用指南

安装依赖 `sudo apt install ros-humble-serial-driver`

更改 [serial_driver.yaml](config/serial_driver.yaml) 中的参数以匹配与电控通讯的串口

提权 `sudo chmod 777 /dev/ttyACM0`

启动串口模块 `ros2 launch rm_serial_driver serial_driver.launch.py`

## 接口

| **packet**          | **header** | **information** |
|:-------------------:|:----------:|:---------------:|
| SendPacketVision    | 0xA5       | 输出敌方机器人状态用于电控解算 |
| SendPacketTwist     | 0xA4       | 底盘导航控制                |
| SendPacketTwist     | 0xA3       | 发送机器人控制命令   |
| ReceivePacketVision | 0x5A       | 接收云台姿态用于自瞄         |
| ReceivePacketAllRobotHP  | 0x5B  | 全体机器人血量信息           |
| ReceivePacketGameStatus  | 0x5C  | 比赛阶段与时间信息           |
| ReceivePacketRobotStatus | 0x5D  | 机器人状态相关信息           |

详情请参考 [packet.hpp](include/rm_serial_driver/packet.hpp) 和 [rm_decision_interfaces](https://gitee.com/SMBU-POLARBEAR/rm_behavior_tree/tree/master/rm_decision_interfaces/msg)

### SendPacketVision

- 视觉端发送 armor_tracker 的输出，即对于目标机器人的观测，具体的运动预测、装甲板选择、弹道解算在电控端完成

### SendPacketTwist

由导航模块输出，用于哨兵机器人底盘运动控制

- linear: 线速度，包含 x, y, z 分量，分别代表沿 x, y, z 轴的线速度。单位 m/s
- angular: 角速度，包含 x, y, z 分量，分别代表绕 x, y, z 轴的角速度。单位 rad/s

### SendPacketRobotControl

- stop_gimbal_scan: bool 型，云台是否停止扫描模式
- chassis_spin_vel: float 型，底盘小陀螺速度。单位 rad/s

### ReceivePacketVision

- 机器人的自身颜色 `robot_color` 以判断对应的识别目标颜色
- 云台姿态 `pitch` 和 `yaw`, 单位和方向请参考 <https://www.ros.org/reps/rep-0103.html>
- 当前云台瞄准的位置 `aim_x, aim_y, aim_z`，用于发布可视化 Marker

### ReceivePacketAllRobotHP

从裁判系统转发的敌我双方全体机器人、前哨站、基地的血量信息：[AllRobotHP.msg](https://gitee.com/SMBU-POLARBEAR/rm_behavior_tree/blob/master/rm_decision_interfaces/msg/AllRobotHP.msg)

### ReceivePacketGameStatus

从裁判系统转发的比赛进行阶段与时间：[GameStatus.msg](https://gitee.com/SMBU-POLARBEAR/rm_behavior_tree/blob/master/rm_decision_interfaces/msg/GameStatus.msg)

### ReceivePacketRobotStatus

从裁判系统转发的本机器人（哨兵）信息：[RobotStatus.msg](https://gitee.com/SMBU-POLARBEAR/rm_behavior_tree/blob/master/rm_decision_interfaces/msg/RobotStatus.msg)

  `robot_id` - 裁判系统直接转发，本机 id

  `current_hp` - 裁判系统直接转发，本机实时血量

  `shooter_heat` - 裁判系统直接转发，本机实时枪管热量（仅留单枪管的热量）

  `team_color` - 下位机处理后的数据，我方颜色，0 - Red, 1 - Blue

  `is_attacked` - 下位机处理后的数据，血量是否下降，0 - False, 1 - True
