# reATRM

## 施工中
*本系统基于[无下位机控制系统 RMCS](https://github.com/Alliance-Algorithm/RMCS)二次开发而来*<br>
*感谢Alliance Algorithm Team团队的开源与努力*

## 备忘与一些常见问题的解决方案
0. 如何克隆本项目
    ```bash
    git clone https://github.com/JinGangRobo/reATRM --recursive
    ```

1. 关于docker engine: 
    **在linux端不要用Docker Desktop, 可能会出现奇怪的问题**

2. 关于REBUILD容器后拓展无法被安装: 
    手动点击在开发容器中安装，并使用命令面板重启拓展主机

3. 关于赛场上怎么调试: 
    使用网线, SSH连接到机载计算机, 并使用X11转发连接GUI应用

4. 关于如何永久设置SLAVE权限
    ```bash
    echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="0d00", MODE="0666"' | sudo tee /etc/udev/rules.d/95-rmcs-slave.rules && sudo udevadm control --reload-rules && sudo udevadm trigger
    ```
5. 关于云台飘移
    - 请检查是否校准遥控器
    - 请检查是否校准IMU零飘
    - 请检查是否妥善设置IMU安装映射与电机标零