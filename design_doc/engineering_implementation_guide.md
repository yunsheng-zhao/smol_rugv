# 工程实施指导（架构与接口基线）

## 范围与约定
- 本文作为后续编码实现的参考基线，约束系统的子系统组成、接口契约、时序与故障降级行为
- 入口包 smol_bringup 只负责 launch 与 config，不引入任何 Node；其职责是声明子系统、定义组合方式、管理系统级参数
- 底盘安全仲裁为 chassis_driver_node 的内部模块，不作为独立包或独立节点存在

## 子系统与节点
- camera_node：采集并发布原始图像
- speech_node：将语音转为文本指令并发布
- chassis_driver_node：串口驱动与里程计/IMU发布；内置安全仲裁模块；接收控制输入并下发电机
- vla_bridge_node：连接 ROS2 与 smol_vla_policy.py；异步推理动作块并输出控制建议

## 系统级数据流与控制流
- 输入数据流：camera_node -> /camera/image_raw；chassis_driver_node -> /odom/odom_raw、/imu/data_raw；speech_node -> /instruction_text
- 控制流：vla_bridge_node -> /cmd_vel -> chassis_driver_node -> 电机

## ROS 接口表

### Camera
| 名称 | 方向 | 类型 | 频率 | QoS | frame_id | 异常/超时行为 |
|---|---|---|---|---|---|---|
| /camera/image_raw | 发布 | sensor_msgs/Image | 30 Hz | sensor_data（BestEffort, depth=10, volatile） | camera_link | 采集失败或超时>200ms时视为输入失效 |

### Speech
| 名称 | 方向 | 类型 | 频率 | QoS | frame_id | 异常/超时行为 |
|---|---|---|---|---|---|---|
| /instruction_text | 发布 | std_msgs/String | 事件驱动（0–2 Hz） | Reliable, depth=10, volatile | n/a | 无语音输入不影响底盘安全 |

### Chassis（含内置安全仲裁模块）
| 名称 | 方向 | 类型 | 频率 | QoS | frame_id | 异常/超时行为 |
|---|---|---|---|---|---|---|
| /cmd_vel | 订阅 | geometry_msgs/Twist | 0–20 Hz（异步） | Reliable, depth=10, volatile | base_link | 超时>200ms将其视为无效输入并刹停 |
| /e_stop | 订阅 | std_msgs/Bool | 事件驱动 | Reliable, depth=10, volatile | n/a | true 时立即输出零速度并保持；人为触发链路 TBD（待 VLA 完成后明确） |
| /odom/odom_raw | 发布 | nav_msgs/Odometry | 50 Hz | sensor_data（BestEffort, depth=10） | odom | 串口/编码器异常时降频并标记状态 |
| /imu/data_raw | 发布 | sensor_msgs/Imu | 100 Hz | sensor_data（BestEffort, depth=10） | base_link | IMU异常时发布诊断并启用退化策略 |

### VLA
| 名称 | 方向 | 类型 | 频率 | QoS | frame_id | 异常/超时行为 |
|---|---|---|---|---|---|---|
| /camera/image_raw | 订阅 | sensor_msgs/Image | 30 Hz | sensor_data（BestEffort, depth=10） | camera_link | 图像超时则停止产生控制建议 |
| /odom/odom_raw | 订阅 | nav_msgs/Odometry | 50 Hz | sensor_data（BestEffort, depth=10） | odom | 失联时进入保守模式或停驶 |
| /imu/data_raw | 订阅 | sensor_msgs/Imu | 100 Hz | sensor_data（BestEffort, depth=10） | base_link | 失联时进入保守模式或停驶 |
| /instruction_text | 订阅 | std_msgs/String | 事件驱动 | Reliable, depth=10 | n/a | 指令解析失败时忽略该事件 |
| /cmd_vel | 发布 | geometry_msgs/Twist | 0–20 Hz（异步） | Reliable, depth=10 | base_link | 推理不可用时不发布或发布零速度建议 |

## 时序策略（适配 smol_vla 动作块异步推理）

### 输入一致性（采样对齐）
- 触发：以新图像到达触发一次动作块更新（可做速率限制），或以定时器触发但每次选择“最新可用图像”
- 取数：为 odom/imu 维护短缓存；推理开始时取“最接近图像时间戳”的样本；若时间差超过阈值（建议 50–100ms）则降级
- 语音：按事件处理，不与 image/imu 强制同步；指令需携带时间戳并设置有效期（例如 2–5s）

### 动作队列与提前触发
- 控制循环：固定频率执行（例如 20Hz），从动作队列取下一步动作并形成 /cmd_vel 建议流
- 推理线程：异步预测动作块并补充队列（非阻塞）
- 提前触发：当队列剩余低于阈值（例如剩余 <30% 或 <T_low 秒）时触发新的动作块预测
- 块融合：新旧动作块存在重叠区间时按简单拼接/加权融合规则合并，避免切换抖动
- 队列耗尽：立即降级为安全策略（零速度或保持最后安全动作）并触发报警/接管建议
- 部署：默认策略模块运行在 vla 进程内（smol_vla_policy.py）

### 控制输出治理（在底盘内置仲裁模块实现）
- 以 /cmd_vel 为输入进行处理

## 故障与降级
- 摄像头断开/超时：VLA 停止产生控制建议；底盘在输入超时后刹停
- 语音模块故障：不影响底盘安全；仅失去语义驱动能力
- 模型推理不可用或过慢：动作队列耗尽后刹停；建议切换接管
- 串口/底盘异常：底盘侧优先自刹；上位机记录诊断并尝试重连

## 部署与启动（Jetson Orin）
- 进程：
  - camera_node
  - speech_node
  - chassis_driver_node（含内置安全仲裁模块）
  - vla_bridge_node（含 smol_vla_policy.py）
- 启动顺序：
  1. chassis_driver_node
  2. camera_node / speech_node
  3. vla_bridge_node
- rosbag 录制（建议）：/camera/image_raw、/odom/odom_raw、/imu/data_raw、/cmd_vel、/instruction_text

## 配置管理
- 参数分层：
  - 硬件参数：串口、IMU 标定、轮距/半径、摄像头内参、frame_id
  - 模型参数：模型路径、推理相关参数、动作队列阈值
  - 运行模式：自动/接管/演示、速度/加速度限幅、watchdog 超时阈值
- 参数覆盖顺序：base -> hardware -> model -> mode -> site
- 启动时输出最终参数快照到日志，保证可追溯

