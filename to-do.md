# Sprint 0：架构确认与基线整理
- [x] 对照 design_doc 完成系统模块与接口基线清单
- [x] 评估 src 现有包与架构差异并输出迁移策略
- [x] 确定缺失包与节点清单（speech、vla、smol_bringup）
- [x] 检查并替换代码中的 ugv_vision 命名为 camera
- [x] 评估 EKF 与 IMU 参数文件的必要性

# Sprint 1：底盘与安全仲裁框架
- [x] 梳理底盘串口数据流并映射到 /odom/odom_raw 与 /imu/data_raw
- [x] 设计并实现 e_stop 急停仲裁（人为触发链路 TBD，待 VLA 完成后明确）
- [x] 设计底盘单元测试与接口验证

# Sprint 2：视觉采集与发布
- [x] 明确 camera_node 发布 /camera/image_raw 的实现路径
- [x] 对齐相机参数与 QoS 配置

# Sprint 3：语音模块
- [x] 定义 speech_node 输入与 /instruction_text 输出规范
- [x] 选型语音识别实现并完成最小可用版本

# Sprint 4：VLA 决策桥接
- [x] 建立 vla 包结构与 vla_bridge_node
- [x] 实现同步策略与动作队列控制逻辑

# Sprint 5：系统级启动与集成
- [ ] 建立 smol_bringup 启动包与系统参数分层
- [ ] 建立系统级启动顺序与降级策略验证
- [ ] 明确 e_stop 人为触发链路（TBD，待 VLA 完成后明确）
- [ ] 执行底盘单元测试与接口验证（移自 Sprint 1）
- [ ] 执行视觉发布的单元测试与接口验证（移自 Sprint 2）
- [ ] 执行语音指令处理单元测试（移自 Sprint 3）
- [ ] 执行 VLA 推理接口测试（移自 Sprint 4）
- [ ] 设计与执行端到端集成测试
