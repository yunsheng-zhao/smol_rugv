# VLA 模块设计方案 (Sprint 4)

## 1. 设计目标
本设计旨在建立 ROS 2 系统与 `LeRobot` / `SmolVLA` 框架的桥接，实现端到端的视觉-语言-动作 (VLA) 推理。核心目标是让小车能够利用预训练的 `SmolVLA` 模型，根据摄像头图像和语音指令输出底盘控制信号。

## 2. 核心策略：引用而非移植
鉴于 `LeRobot` 框架的复杂性及其频繁的更新，我们采取 **“引用库”** 而非 **“复制代码”** 的策略。

*   **依赖管理**：
    *   `ref_code/lerobot-main/src` 将被视为外部依赖库。
    *   在运行时，通过设置 `PYTHONPATH` 或在代码中动态 `sys.path.append` 的方式，让 `src/vla` 能够直接 `import lerobot`。
    *   **不**将 `lerobot` 的源码复制到 `src/vla` 目录中，避免维护噩梦。

*   **职责分离**：
    *   `lerobot`：负责模型加载、权重管理、数据预处理 (Processor) 和推理。
    *   `src/vla`：负责 ROS 2 通信、数据格式转换 (ROS msg <-> Numpy/Tensor)、线程调度和异常处理。

## 3. 模块结构与职责
`src/vla` 包将遵循以下结构，严格分离 IO、逻辑与模型封装：

```text
src/vla/
├── package.xml             # ROS 2 包定义
├── setup.py                # Python 包安装脚本
├── resource/               # 资源文件
└── vla/
    ├── __init__.py
    ├── vla_bridge_node.py  # [入口] ROS 2 节点，负责组装各模块并启动线程
    ├── io/
    │   ├── __init__.py
    │   └── ros_io.py       # [IO层] 封装 ROS 订阅/发布，将 ROS 消息转为 Python 对象
    ├── core/
    │   ├── __init__.py
    │   ├── shared_buffer.py # [核心] 线程安全的环形缓冲区，处理读写锁
    │   └── sync_policy.py   # [核心] 数据同步策略 (如：取最近图像 + 匹配最近 Odom)
    ├── inference/
    │   ├── __init__.py
    │   ├── vla_loop.py      # [逻辑] 独立推理循环：从 Buffer 取数 -> 推理 -> 发布
    │   └── preprocess.py    # [逻辑] 数据适配：Numpy Image -> LeRobot Tensor
    └── model/
        ├── __init__.py
        └── smol_vla_policy.py # [适配层] LeRobot SmolVLAPolicy 的 Wrapper 类
```

### 详细职责：
1.  **`vla_bridge_node.py`**:
    *   初始化 ROS 节点。
    *   实例化 `ROSIO`、`SharedBuffer`、`SmolVLAPolicyWrapper`。
    *   启动 `VLALoop` 线程。
    *   主线程执行 `rclpy.spin()` 处理 ROS 回调。

2.  **`io/ros_io.py`**:
    *   提供 `subscribe_image()`, `subscribe_odom()`, `subscribe_instruction()` 方法。
    *   提供 `publish_cmd_vel()` 方法。
    *   回调函数仅负责将 ROS 消息解析并推入 `SharedBuffer`，不做耗时处理。

3.  **`core/shared_buffer.py`**:
    *   维护最新的观测数据 (`image`, `odom`, `imu`, `instruction`)。
    *   提供线程安全的 `update()` 和 `get_snapshot()` 接口。

4.  **`model/smol_vla_policy.py`**:
    *   内部持有 `lerobot.policies.smolvla.modeling_smolvla.SmolVLAPolicy` 实例。
    *   负责加载预训练权重（从 HuggingFace Hub 或本地缓存）。
    *   提供 `step(observation) -> action` 接口。

## 4. 数据流与线程模型
系统采用 **双线程模型** 以解耦高频的 ROS 通信与低频/耗时的模型推理。

### 线程 1: Main Thread (ROS Event Loop)
*   **触发源**：ROS 订阅回调 (Image @ 30Hz, Odom @ 50Hz, IMU @ 100Hz)。
*   **动作**：
    1.  接收消息。
    2.  转换为基础数据类型 (cv2 image, float array)。
    3.  写入 `SharedBuffer` (覆盖旧数据，保持最新)。
*   **特点**：非阻塞，极低延迟。

### 线程 2: Inference Thread (VLA Loop)
*   **触发源**：`While True` 循环 (受限于推理速度，约 5-15Hz)。
*   **动作**：
    1.  从 `SharedBuffer` 获取数据快照 (Snapshot)。
    2.  执行 `sync_policy` (例如：检查图像时间戳是否过旧)。
    3.  调用 `preprocess` 将数据转为 Tensor。
    4.  调用 `model.step()` 获取 Action (Action 可能包含未来几帧的动作序列)。
    5.  (可选) 执行动作平滑或插值。
    6.  调用 `ros_io.publish_cmd_vel()` 发布控制指令。
*   **特点**：计算密集型，允许阻塞。

## 5. 接口定义 (ROS)

| 话题名称 | 方向 | 类型 | 频率 | 用途 |
| :--- | :--- | :--- | :--- | :--- |
| `/camera/image_raw` | Sub | `sensor_msgs/Image` | 30Hz | 视觉输入 |
| `/odom/odom_raw` | Sub | `nav_msgs/Odometry` | 50Hz | 自身运动状态输入 |
| `/imu/data_raw` | Sub | `sensor_msgs/Imu` | 100Hz | 姿态输入 |
| `/instruction_text` | Sub | `std_msgs/String` | Event | 语言指令 (e.g. "Move forward") |
| `/cmd_vel` | Pub | `geometry_msgs/Twist` | ~10Hz | 输出到底盘的控制指令 |

## 6. 异常处理与安全
1.  **数据陈旧检测**：
    *   推理线程在取数时检查 `image` 时间戳。如果当前时间 - 图像时间 > 500ms，视为摄像头故障或卡顿，停止发布 `cmd_vel` (或发布零速)。
2.  **指令超时**：
    *   语音指令设置有效期 (TTL)，过期的指令不再触发特定动作。
3.  **模型加载失败**：
    *   节点启动时若无法加载模型，应打印 Error 并退出，避免僵尸进程。

## 7. 部署注意事项
*   **显存管理**：Jetson Orin Nano 显存有限 (8GB)，需确保 `SmolVLA` 模型加载在 GPU 上，且预留部分显存给系统。
*   **FP16 推理**：强制使用 FP16 精度以提升速度并降低显存占用。
*   ## 关键策略
- Receding Horizon Control (滚动时域控制) : 模型一次输出未来 N 步动作（Chunk），但我们只执行其中一部分，并随着新推理结果的产生不断用新的 Chunk 覆盖旧的 Chunk。这能有效抵抗推理抖动。
- Reference-Based Integration (引用式集成) : 不将 lerobot 源码复制到包内，而是通过 sys.path 动态引用外部 ref_code ，降低维护成本。
- FP16 精度 : 强制使用半精度推理，适配 Jetson Orin 硬件特性，兼顾速度与显存

## 8. src/vla 文件结构与实现细节

### 8.1 入口与调度
文件 : vla_bridge_node.py

- 职责 : 系统的指挥官。
- 主要逻辑 :
  1. 初始化 ROS 节点 vla_bridge_node 。
  2. 创建核心组件： SharedBuffer (数据池), ActionQueue (指令池), ROSIO (通信接口)。
  3. 启动 推理线程 ( VLALoop.start() )。
  4. 启动 控制定时器 ( create_timer(1/20Hz, _control_loop) )。
- 关键代码 :
  ```
  # 20Hz 控制循环
  def _control_loop(self):
      action = self.action_queue.get_next_action()
      if action:
          self.ros_io.publish_cmd_vel(*action)
      else:
          self.ros_io.publish_cmd_vel(0.0, 0.0) # 安全刹车
  ```
### 8.2 IO 层 (ROS 通信)
文件 : ros_io.py

- 职责 : ROS 2 接口的封装，负责数据格式转换。
- 主要逻辑 :
  - 订阅 :
    - /camera/image_raw -> 转 cv2 (RGB) -> SharedBuffer
    - /odom/odom_raw -> 转 numpy dict (pos, vel) -> SharedBuffer
    - /instruction_text -> SharedBuffer
  - 发布 :
    - publish_cmd_vel(vx, wz) -> /cmd_vel
  - QoS : 传感器使用 BestEffort (丢弃旧数据)，指令使用 Reliable (保证到达)。
### 8.3 Core 层 (数据同步与缓存)
文件 : shared_buffer.py

- 职责 : 线程安全的“黑板”，存储最新的传感器读数。
- 实现 : 使用 threading.Lock 保护读写操作。
文件 : sync_policy.py

- 职责 : 数据“保鲜员”。
- 逻辑 :
  - 检查图像是否过期（ current_time - image_time > 0.5s ）。
  - 检查 Odom 是否过期。
  - 修复点 : 强制使用 ROS 系统时间进行比较，避免仿真时间与物理时间不一致导致的误判。
文件 : action_queue.py

- 职责 : 动作序列缓冲区。
- 策略 : Overwrite (覆盖) 。当新的推理结果 (Chunk) 到达时，清空队列中剩余的旧动作，填入新动作。这是 Receding Horizon 的典型实现。
### 8.4 Inference 层 (推理逻辑)
文件 : vla_loop.py

- 职责 : 独立的推理线程。
- 流程 :
  1. buffer.get_snapshot() : 获取数据。
  2. sync_policy.is_valid() : 检查数据有效性。
  3. input_mapper.map() : 组装输入。
  4. model.step() : 执行推理 。
  5. action_queue.put_chunk() : 将结果推入队列。
文件 : preprocess.py

- 职责 : 数据适配器。
- 逻辑 : 将 ROS 的 odom 字典转换为模型需要的特征向量 (如 [vx, vy, wz] )。
### 8.5 Model 层 (模型封装)
文件 : smol_vla_policy.py

- 职责 : LeRobot 的适配器 (Wrapper)。
- 主要逻辑 :
  - 动态导入 : 将 ref_code 路径加入 sys.path ，导入 SmolVLAPolicy 。
  - FP16 优化 : 检测到 CUDA 时，自动执行 .half() 。
  - Pipeline : 初始化 lerobot 的预处理和后处理流水线。