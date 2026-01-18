# P0 架构改进说明

## 系统级数据流与控制流
- 控制产生者：VLA 自动决策、语音指令解析后的意图、人为接管（遥控/键盘）、急停输入
- 最终控制权：由 chassis_driver_node 内置的安全仲裁模块统一决定，并下发电机控制
- 优先级：急停 > 人为接管 > 安全限幅 > 自动决策（VLA/语音意图） 
- 急停：硬件按钮或 /e_stop (std_msgs/Bool)，触发后立即输出零速度并保持
- 接管：/teleop/cmd_vel 接入仲裁层，接管期间覆盖自动决策输出
- 限幅：对线速度/角速度以及加速度进行限幅，结合地面约束与碰撞预防策略

## 接口表：Camera 包

| 名称 | 方向 | 类型 | 频率 | QoS | frame_id | 异常/超时行为 |
|---|---|---|---|---|---|---|
| /camera/image_raw | 发布 | sensor_msgs/Image | 30 Hz | sensor_data（BestEffort, depth=10, volatile） | camera_link | 采集失败或超时>200ms时发布停止事件，VLA进入保守模式 |

## 接口表：Speech 包

| 名称 | 方向 | 类型 | 频率 | QoS | frame_id | 异常/超时行为 |
|---|---|---|---|---|---|---|
| /instruction_text | 发布 | std_msgs/String | 事件驱动（0–2 Hz） | Reliable, depth=10, volatile | n/a | 无语音输入不影响底盘安全；仅失去语义驱动能力 |

说明：建议后续扩展为结构化指令消息（含 intent/slots/confidence/stamp/source），当前保持 String 不改动。

## 接口表：Chassis 包

| 名称 | 方向 | 类型 | 频率 | QoS | frame_id | 异常/超时行为 |
|---|---|---|---|---|---|---|
| /cmd_vel_vla | 订阅 | geometry_msgs/Twist | 0–20 Hz（异步） | Reliable, depth=10, volatile | base_link | 超时>200ms将其视为无效输入并刹停 |
| /teleop/cmd_vel | 订阅 | geometry_msgs/Twist | 0–20 Hz | Reliable, depth=10, volatile | base_link | 接管输入优先级高于 /cmd_vel_vla |
| /e_stop | 订阅 | std_msgs/Bool | 事件驱动 | Reliable, depth=10, volatile | n/a | true 时立即输出零速度并保持 |
| /odom | 发布 | nav_msgs/Odometry | 50 Hz | sensor_data（BestEffort, depth=10） | odom | 串口/编码器异常时降频并标记状态 |
| /imu/data | 发布 | sensor_msgs/Imu | 100 Hz | sensor_data（BestEffort, depth=10） | base_link | IMU异常时发布诊断并启用里程计退化策略 |

## 接口表：VLA 包

| 名称 | 方向 | 类型 | 频率 | QoS | frame_id | 异常/超时行为 |
|---|---|---|---|---|---|---|
| /camera/image_raw | 订阅 | sensor_msgs/Image | 30 Hz | sensor_data（BestEffort, depth=10） | camera_link | 图像超时时进入保守模式，不输出控制 |
| /odom | 订阅 | nav_msgs/Odometry | 50 Hz | sensor_data（BestEffort, depth=10） | odom | 失联时仅基于IMU做短时推断或停驶 |
| /imu/data | 订阅 | sensor_msgs/Imu | 100 Hz | sensor_data（BestEffort, depth=10） | base_link | 失联时降低控制频率或停驶 |
| /instruction_text | 订阅 | std_msgs/String | 事件驱动 | Reliable, depth=10 | n/a | 指令解析失败时忽略该事件 |
| /cmd_vel_vla | 发布 | geometry_msgs/Twist | 0–20 Hz（异步） | Reliable, depth=10 | base_link | 推理不可用时不发布或发布零速度建议 |

## 时序与同步
- 时间戳来源：相机驱动/底盘驱动原始时间戳，统一使用 ROS2 时钟体系进行表达与比较
- 输入一致性（采样对齐）：
  - 触发时机：以新图像到达触发推理（可做速率限制），或以定时器触发但每次选择“最新可用图像”
  - 取数策略：为 odom/imu 维护短缓存，从缓存中取“最接近图像时间戳”的样本；若时间差超过阈值（建议 50–100ms）则判为缺失并降级
  - 语音文本：按事件处理，不与 image/imu 强制同步；只需要保证指令有时间戳并设置有效期（例如 2–5s）
  - 说明：该对齐发生在推理开始时，和推理耗时是否稳定无关
- 动作块异步推理（smol_vla 特性适配）：
  - 目标：将“动作执行”与“动作块预测”解耦，避免推理延迟直接进入控制回路
  - 动作队列：VLA 维护一个动作队列（长度 n 个离散动作或持续 T 秒的动作轨迹），控制循环从队列取出下一步动作
  - 提前触发：当队列长度低于阈值（例如剩余 <30% 或 <T_low 秒）时，向策略模块请求新的动作块并行补充队列
  - 解耦线程：控制循环固定频率运行（例如 20Hz），推理线程异步运行（非阻塞）
  - 块融合：新旧动作块存在重叠区间时，按简单拼接/加权融合规则合并，避免切换抖动
  - 队列耗尽：若推理过慢导致队列耗尽，立即降级为安全策略（零速度或保持最后安全动作）并触发报警/接管建议
  - 部署说明：默认策略模块运行在 vla 进程内（例如 smol_vla_policy.py）；仅在资源隔离或远端推理需求下才拆分为独立服务
- 输出安全性（控制输出治理）：
  - VLA 推理耗时不确定，因此允许 /cmd_vel_vla 以“异步、非严格周期”方式输出控制建议；对上层可视为“动作队列驱动的控制建议流”
  - chassis_driver_node 内置安全仲裁模块负责把不定周期输入整形为可执行控制：保持最近一次有效建议、限幅与加速度平滑，并在输入超时（建议 >200ms）时自动刹停
  - 端到端延迟关注点：不是保证固定周期，而是限制“数据过期”和“控制建议过期”
- 队列与丢帧：各输入队列 depth=10，落后过多时丢弃最旧帧，避免延迟堆积

## 故障与降级
- 摄像头断开/超时：进入保守模式，若>200ms无新图像则发布零速度
- 语音模块故障：不影响底盘安全，自动决策仍可运行
- 模型推理不可用：仲裁层将输出零速度；可提示切换人为接管
- 串口/底盘异常：底盘侧优先自刹；上位机记录诊断并尝试重连

参考架构图：见 arch.png

