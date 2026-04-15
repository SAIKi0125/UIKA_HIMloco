# Quadruped UIKA - RL 强化学习部署框架

[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/)
[![ROS2 Humble](https://img.shields.io/badge/ros2-humble-brightgreen.svg?logo=ros)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg?logo=apache)](https://opensource.org/license/apache-2-0)

基于 [rl_sar](https://github.com/fan-ziqi/rl_sar) 的四足机器人强化学习部署框架，专为 **UIKA** 12-DOF 四足机器人优化。

## 机器人规格

| 项目 | 参数 |
|------|------|
| 自由度 | 12-DOF (每腿 3 关节) |
| 关节类型 | RobStride 电机 |
| 策略 | HIM-PPO (IsaacGym 训练) |
| 推理框架 | ONNX Runtime |
| 控制频率 | 50 Hz |
| 通信 | ROS2 Humble |

## 快速开始

### 1. 环境配置 (一键脚本)

```bash
cd /home/saiki/Quadruped_Uika-main/rl_sar-main

# 完整配置 (ROS2 + 推理库 + 构建)
./scripts/setup_env.sh

# 仅 CMake 编译 (无 ROS，适合硬件部署)
./scripts/setup_env.sh --cmake-only
```

### 2. 编译

```bash
# CMake 编译 (硬件部署)
./build.sh -m

# 或完整 ROS2 编译
./build.sh
```

### 3. 运行

**实机部署：**
```bash
# 终端 1: 传感器节点 (下位机发布)
# 终端 2: RL 推理节点
./cmake_build/bin/uika
```

**模拟测试：**
```bash
# 终端 1: 模拟传感器
./cmake_build/bin/uika_mock_publisher

# 终端 2: RL 推理节点
./cmake_build/bin/uika

# 终端 3: 查看输出
ros2 topic echo /uika/target_position
```

## 项目结构

```
rl_sar-main/
├── build.sh                    # 编译脚本
├── scripts/
│   └── setup_env.sh           # 一键环境配置脚本
├── cmake_build/bin/
│   ├── uika                   # RL 推理节点
│   └── uika_mock_publisher   # 模拟传感器
├── policy/uika/
│   ├── base.yaml              # 基础配置 (关节顺序)
│   └── himloco/
│       └── config.yaml        # HIM-PPO 推理配置
├── src/rl_sar/
│   ├── src/
│   │   ├── rl_real_uika.cpp  # RL 推理节点源码
│   │   └── uika_mock_publisher.cpp
│   └── launch/
│       └── uika_test.launch.py
└── UIKA_DEPLOY_README.md     # 详细部署文档
```

## ROS2 话题

| 话题 | 方向 | 类型 | 维度 | 说明 |
|------|------|------|------|------|
| `/uika/joint_state` | 订阅 | Float64MultiArray | 24D | `[q0..q11, dq0..dq11]` |
| `/uika/imu` | 订阅 | Float64MultiArray | 7D | `[qw,qx,qy,qz,gx,gy,gz]` |
| `/uika/target_position` | 发布 | Float64MultiArray | 12D | 关节目标位置 (rad) |

## 关节顺序

```
FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf,
RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf
```

## 详细文档

- [UIKA 部署详细指南](UIKA_DEPLOY_README.md) - 编译、运行、话题、配置、故障排查
- [项目架构说明](CLAUDE.md) - 代码结构、模块说明
- [原始 rl_sar 文档](https://github.com/fan-ziqi/rl_sar) - 更多信息

## 编译选项

| 命令 | 用途 |
|------|------|
| `./build.sh -m` | CMake 编译 (硬件部署) |
| `./build.sh -mj` | CMake + MuJoCo |
| `./build.sh -c` | 清理编译产物 |
| `./build.sh` | 完整 ROS2 编译 |

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- CUDA (可选，用于 GPU 推理)
- Conda (推荐 `ros2_humble` 环境)

## 鸣谢

基于 [rl_sar](https://github.com/fan-ziqi/rl_sar) 框架构建。
