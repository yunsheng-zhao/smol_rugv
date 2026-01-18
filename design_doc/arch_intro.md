一、 ROS2 功能包（Package）划分
系统被划分为四个核心功能包，实现了感知、交互、驱动与智能决策的分离 ：
camera/：负责视觉数据采集 。
speech/：负责语音交互 。
chassis/：负责底层底盘控制与传感器反馈 。
vla/：负责大模型桥接与推理决策 。

二、 各功能包详细职责与通信接口
1. Camera Package (视觉包)

主要任务：读取摄像头原始数据并将其转换为 ROS2 标准图像格式（Image）进行发布 。
设计原则：该节点保持轻量化，不包含感知、推理、同步或预处理逻辑 。
通信话题：
发布：/camera/image_raw (消息类型：sensor_msgs/Image) 。

2. Speech Package (语音包)

主要任务：将麦克风采集的音频输入实时转换为文本指令 。
通信话题：
发布：/instruction_text (消息类型：std_msgs/String) 。

3. Chassis Package (底盘包)

主要任务：通过串口与 ESP32 通信，实现底层协议解析、轮速及 IMU 数据解码，并将控制指令下发给电机 。
通信话题 ：
订阅：/cmd_vel (消息类型：geometry_msgs/Twist) 。
发布：/odom (消息类型：nav_msgs/Odometry) 。
发布：/imu/data (消息类型：sensor_msgs/Imu) 。

4. VLA Package (决策桥接包)

主要任务：作为“大脑”，负责连接 ROS2 环境与 smolVLA / LeRobot 推理框架 。
节点设置：该包仅包含一个核心节点 vla_bridge_node 。
通信话题 ：
订阅：/camera/image_raw、/odom、/imu/data 以及 /instruction_text 。
发布：/cmd_vel (发送至 chassis_driver_node) 。

三、 VLA 功能包工程结构

VLA 模块遵循清晰的代码组织架构，确保算法与 IO 逻辑分离：

vla/
├── vla_bridge_node.py      # 唯一的 ROS2 Node 入口 
├── io/
│   └── ros_io.py           # 处理订阅、发布及 Buffer 写入
├── core/
│   ├── shared_buffer.py    # 线程间共享数据缓冲区
│   └── sync_policy.py      # 数据同步策略 
├── inference/
│   ├── vla_loop.py         # 独立于 ROS 的模型推理线程 
│   └── preprocess.py       # 多模态数据预处理 
├── model/
│   └── smol_vla_policy.py  # 封装 LeRobot / SmolVLA 模型
