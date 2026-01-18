# 项目介绍
本项目是一个基于 Jetson Orin 的 UGV 智能小车项目，采用 ROS2 作为主要的开发框架。目的是将一个lerobot的smolVLA模型，部署到这个基于jetson orin nano的移动小车上，实现小车的自动控制功能：
1. 小车可以根据摄像头采集到的图像，自动识别并跟随人或物体；
2. 小车可以根据语音输入指令，自动执行简单的移动和旋转操作；

小车具备单摄像头和电机驱动的四个轮子；小车的中间件基于ROS2

# 硬件说明
小车硬件参考链接：
https://www.waveshare.net/wiki/UGV_Rover_Jetson_Orin_ROS2

# 架构说明
架构图参考：
![架构图](arch_intro.png)

# comment
本项目中不涉及eps32部分的开发，小车的底盘控制等基本功能复用自waveshare的ugv rover项目；

# todo
当前小车的语音输入模块的解决方案还未完成，后续会根据实际情况进行完善；