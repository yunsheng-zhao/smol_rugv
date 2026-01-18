# P1 部署视图与配置管理

## 部署视图（Jetson Orin）
- 进程/容器划分：
  - camera_node（ROS2 原生节点）
  - speech_node（ROS2 原生节点）
  - chassis_driver_node（ROS2 原生节点，串口）
  - vla_bridge_node（Python 节点，模型推理线程独立）

- 入口包（smol_bringup）：
  - 仅包含 launch 与 config，不引入任何 Node
  - 声明系统包含的子系统（camera/speech/chassis/vla）
  - 定义子系统如何组合（启动顺序、命名空间、话题重映射）
  - 管理系统级参数（hardware + model + mode + site），并在启动时输出参数快照到日志
  - 确保控制链路进入底盘内置仲裁模块（例如 remap vla 输出到 /cmd_vel_vla，由 chassis_driver_node 内部仲裁后下发电机）
- 启动顺序：
  1. chassis_driver_node（保证底盘安全可控）
  2. camera_node / speech_node（提供输入）
  3. vla_bridge_node（加载模型后开始发布控制建议）
  4. 系统进入运行态（底盘内置仲裁模块持续工作）
- 资源预算（建议范围）：
  - vla_bridge_node：CPU 2–4 核，GPU 显存 2–6GB（视模型而定），内存 2–4GB
  - camera_node：CPU 0.5–1 核，内存 0.5GB
  - speech_node：CPU 0.5–1 核，内存 0.5GB
  - chassis_driver_node：CPU <0.5 核，内存 <0.5GB
- 日志与 rosbag 录制：
  - 录制主题：/camera/image_raw、/odom、/imu/data、/cmd_vel、/instruction_text
  - 分片策略：按时间或大小分片（例如 10min / 2GB），启用压缩
  - 元数据：记录参数快照与模型版本号，便于回放与回归

## 配置管理
- 参数分层：
  - 硬件参数（hardware.yaml）：串口、IMU 标定、轮距/半径、摄像头内参、frame_id
  - 模型参数（model.yaml）：模型名称/路径、推理频率、阈值/置信度、预处理尺寸
  - 运行模式（mode.yaml）：自动/接管/演示模式、速度/加速度限幅、仲裁优先级
- 目录结构建议：
  - config/
    - hardware/
      - default.yaml
      - jetson_orin_rover.yaml
    - model/
      - smol_vla.yaml
    - mode/
      - auto.yaml
      - teleop.yaml
    - sites/
      - lab.yaml
      - outdoor.yaml
- 复用与覆盖：
  - 通过 launch 参数选择不同场地/设备预设（sites/ 与 hardware/）
  - 支持参数覆盖顺序：base -> hardware -> model -> mode -> site
  - 在启动时输出最终参数快照到日志，确保可追溯

## 运行与运维建议
- 健康检查：定期检查输入主题频率与延迟，异常阈值触发报警
- 回放流程：用 rosbag2 回放关键主题，验证不同模型版本的控制一致性
- 资源监控：采集 CPU/GPU/内存与 I/O，用于定位性能瓶颈

