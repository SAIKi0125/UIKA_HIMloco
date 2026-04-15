# UIKA 四足机器人 RL 部署指南

## 概述

UIKA 是 12-DOF 四足机器人，使用 HIM-PPO 策略进行强化学习控制。训练在 Isaac Gym 中完成，部署通过 ONNX 模型在 ROS2 节点中实时推理。

**控制流程：**
```
传感器 → ROS2 话题 → rl_real_uika 节点（ONNX 推理）→ 关节目标位置 → 下位机
```

---

## 1. 编译

```bash
cd /home/saiki/Quadruped_Uika-main/rl_sar-main

# 激活环境
eval "$(conda shell.bash hook)" && conda activate ros2_humble

# CMake 编译（硬件部署用，无需完整 ROS 依赖）
./build.sh -m

# 输出二进制
# cmake_build/bin/uika                  ← RL 推理节点
# cmake_build/bin/uika_mock_publisher   ← 模拟传感器（测试用）
```

其他编译选项：
| 命令 | 用途 |
|------|------|
| `./build.sh -m` | CMake 编译，硬件部署 |
| `./build.sh -mj` | CMake + MuJoCo 编译 |
| `./build.sh -c` | 清理所有编译产物 |
| `colcon build --merge-install --packages-select robot_msgs rl_sar` | ROS2 完整编译（需要所有依赖） |

---

## 2. 运行

### 2.1 实机部署

需要两个终端，先确保 `source /opt/ros/humble/setup.bash`。

**终端 1 — 传感器端（下位机节点）：**
负责发布关节状态和 IMU 数据。

**终端 2 — RL 推理节点：**
```bash
cd /home/saiki/Quadruped_Uika-main/rl_sar-main
source /opt/ros/humble/setup.bash
./cmake_build/bin/uika
```

节点启动后会打印：
```
[INFO] UIKA RL Node initialized successfully
[INFO]   [Inference]
[INFO]     Rate: 50.0 Hz (dt=0.0050, decimation=4)
[INFO]     Observation: 45 dims x 6 frames = 270 total
```

### 2.2 模拟测试（无实机）

用 mock publisher 模拟传感器数据，验证推理节点是否正常工作。

**方式 A — 手动启动三个终端：**

```bash
# 终端 1: 模拟传感器
source /opt/ros/humble/setup.bash
./cmake_build/bin/uika_mock_publisher

# 终端 2: RL 推理节点
source /opt/ros/humble/setup.bash
./cmake_build/bin/uika

# 终端 3: 查看输出的关节目标角度
source /opt/ros/humble/setup.bash
ros2 topic echo /uika/target_position
```

**方式 B — launch 文件一键启动：**

```bash
cd /home/saiki/Quadruped_Uika-main/rl_sar-main
source install/setup.bash   # 需要先 colcon build
ros2 launch rl_sar uika_test.launch.py
```

---

## 3. ROS2 话题

### 输入（RL 节点订阅）

| 话题 | 类型 | 维度 | 说明 |
|------|------|------|------|
| `/uika/joint_state` | Float64MultiArray | 24D | `[q0..q11, dq0..dq11]` 位置+速度 |
| `/uika/imu` | Float64MultiArray | 7D | `[qw, qx, qy, qz, gx, gy, gz]` |

### 输出（RL 节点发布）

| 话题 | 类型 | 维度 | 说明 |
|------|------|------|------|
| `/uika/target_position` | Float64MultiArray | 12D | 关节目标位置 (rad) |

### 调试命令

```bash
# 检查话题频率
ros2 topic hz /uika/target_position    # 应为 ~50Hz
ros2 topic hz /uika/joint_state        # 应为 ~200Hz

# 查看数据
ros2 topic echo /uika/target_position
```

---

## 4. 关节顺序

12 个关节按以下顺序排列（与 config.yaml 和 URDF 一致）：

| 索引 | 关节名 | 默认角度 (rad) |
|------|--------|----------------|
| 0 | FL_hip_joint | -0.70 |
| 1 | FL_thigh_joint | 0.20 |
| 2 | FL_calf_joint | 0.70 |
| 3 | FR_hip_joint | 0.70 |
| 4 | FR_thigh_joint | -0.20 |
| 5 | FR_calf_joint | -0.70 |
| 6 | RL_hip_joint | 0.70 |
| 7 | RL_thigh_joint | 0.20 |
| 8 | RL_calf_joint | 0.70 |
| 9 | RR_hip_joint | -0.70 |
| 10 | RR_thigh_joint | -0.20 |
| 11 | RR_calf_joint | -0.70 |

---

## 5. 配置文件

策略相关配置位于 `policy/uika/himloco/config.yaml`：

| 参数 | 值 | 说明 |
|------|---|------|
| `model_name` | `uika_20260410_*.onnx` | ONNX 模型文件名 |
| `dt` | 0.005 | 控制周期 5ms |
| `decimation` | 4 | 推理频率 = 1/(0.005×4) = 50Hz |
| `action_scale` | 0.25 | 动作缩放系数 |
| `fixed_kp` | 30.0 | 部署 PD 刚度 |
| `fixed_kd` | 1.0 | 部署 PD 阻尼 |
| `torque_limits` | Hip/Thigh: 6.0, Calf: 11.2 | 力矩限制 (Nm)，额定力矩 |
| 初始站立高度 | 0.3357m | 基座离地高度 |

**更换策略模型：** 修改 `config.yaml` 中 `model_name` 字段，将新 `.onnx` 文件放入 `policy/uika/himloco/` 目录。

---

## 6. 关键路径

```
rl_sar-main/
├── build.sh                              # 编译脚本
├── cmake_build/bin/
│   ├── uika                              # RL 推理节点可执行文件
│   └── uika_mock_publisher               # 模拟传感器可执行文件
├── policy/uika/
│   ├── base.yaml                         # 基础配置（dt, decimation, 关节名）
│   └── himloco/
│       ├── config.yaml                   # HIM-PPO 推理配置
│       └── *.onnx                        # 策略模型
├── src/rl_sar/
│   ├── src/
│   │   ├── rl_real_uika.cpp              # RL 推理节点源码
│   │   └── uika_mock_publisher.cpp       # 模拟传感器源码
│   └── launch/
│       └── uika_test.launch.py           # 测试 launch 文件
```

---

## 7. 故障排查

| 现象 | 可能原因 | 解决方法 |
|------|---------|---------|
| 节点启动报 `Failed to load config` | config.yaml 路径错误 | 检查 `policy/uika/himloco/config.yaml` 是否存在 |
| 节点启动报 `Failed to load model` | ONNX 文件缺失 | 检查 `model_name` 对应的 `.onnx` 文件是否在目录中 |
| `/uika/target_position` 无输出 | 未收到传感器数据 | 确认 `/uika/joint_state` 和 `/uika/imu` 都在发布 |
| 输出频率不到 50Hz | 传感器话题频率不足 | 确认 joint_state 和 imu 频率 >= 50Hz |
| 机器人抖动 | PD 参数不合适 | 调整 `fixed_kp`/`fixed_kd`，参考 CLAUDE.md 中的调参建议 |
| 编译报 CMakeCache 冲突 | 旧缓存残留 | `rm -rf cmake_build` 后重新 `./build.sh -m` |
