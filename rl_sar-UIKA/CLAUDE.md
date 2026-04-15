# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

  source ~/UIKA/Quadruped_Uika/install/setup.bash           
  cd ~/UIKA/rl_sar-main                                                                                                          
  rm -rf cmake_build CMakeCache.txt                                                                                              
  ./build.sh -m
```bash
cd /home/saiki/Quadruped_Uika-main/rl_sar-main

# Full ROS build (colcon)
./build.sh

# CMake only (hardware deployment, no ROS dependencies)
./build.sh -m

# CMake + MuJoCo simulator
./build.sh -mj

# Clean build artifacts
./build.sh -c
```

### Running Simulation

```bash
# Gazebo (ROS 2)
source install/setup.bash
ros2 launch rl_sar gazebo.launch.py rname:=<ROBOT>

# MuJoCo
./cmake_build/bin/rl_sim_mujoco <ROBOT> <SCENE>
# Example: ./cmake_build/bin/rl_sim_mujoco g1 scene_29dof
```

### Running on Real Robots

```bash
# Unitree Go2
./cmake_build/bin/rl_real_go2 <NETWORK_INTERFACE> [wheel]

# Unitree G1
./cmake_build/bin/rl_real_g1 <NETWORK_INTERFACE>

# Unitree A1
./cmake_build/bin/rl_real_a1
```

## Architecture

### Three-Layer Structure

1. **`src/rl_sar/`** - Main C++ runtime with FSM implementations
   - `src/` - Executable entry points (`rl_sim`, `rl_real_*`, `rl_real_uika`)
   - `fsm_robot/` - Robot-specific finite state machines
   - `library/core/` - Reusable runtime components

2. **`src/rl_sar_zoo/`** - Robot descriptions (URDF/xacro) and configs
   - Each robot has `*_description/` with xacro models and Gazebo configs
   - Supported: a1, go2, go2w, b2, b2w, g1, gr1t1, gr1t2, l4w4, lite3, d1, tita

3. **`policy/<ROBOT>/`** - Pre-trained policies organized by robot
   - `base.yaml` defines joint ordering (must match physical robot)
   - `<CONFIG>/` subdirectories contain `.pt` (libtorch) or `.onnx` (onnxruntime) models

### Core Library Components (`library/core/`)

- **inference_runtime/** - Abstraction over ONNXRuntime and libtorch
- **fsm/** - State machine for mode transitions (idle, locomotion, skill, etc.)
- **observation_buffer/** - Rolling buffer for temporal observations
- **loop/** - High-precision control loop timing (50Hz typical)
- **rl_sdk/** - Base class defining `Forward()`, `GetState()`, `SetCommand()` interface

### UIKA-Specific Deployment

The UIKA robot (12-DOF quadruped) uses HIM-PPO policy with ONNX inference:

- **Config**: `policy/uika/base.yaml` and `policy/uika/himloco/config.yaml`
- **Node**: `src/rl_sar/src/rl_real_uika.cpp` - subscribes to sensor topics, runs ONNX inference, publishes joint targets
- **ROS2 Topics**:
  - Input: `/uika/joint_state` (24D: [q0..q11, dq0..dq11]), `/uika/imu` (7D: quat + gyro)
  - Output: `/uika/target_position` (12D joint targets)
- **Control rate**: 50Hz inference (dt=0.005s, decimation=4)

**Joint order** (as defined in `base.yaml`): FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf

**PD gains**: `fixed_kp=30.0`, `fixed_kd=1.0` (configured in `config.yaml`)

### ONNX Model Loading

**`src/rl_sar/src/rl_real_uika.cpp`** 实现 `FindLatestOnnx()` — 启动时自动扫描 `policy/uika/himloco/` 目录下所有 `.onnx` 文件，按修改时间选最新的。

模型路径确定顺序：
1. **优先**: `config.yaml` 中的 `model_name` 字段
2. **Fallback**: 扫描 `policy/uika/himloco/` 目录，选最新 `.onnx`
3. **备选**: `policy/uika/` 根目录（如果 himloco 子目录不存在）

`POLICY_DIR` 在 CMakeLists.txt 编译时定义，指向项目根目录的 `policy/` 文件夹。

**使用方式**: 把新的 `.onnx` 文件扔进 `policy/uika/himloco/`，**不需要改 config.yaml**，重启节点自动加载最新的。

### Key File Paths

| Purpose | Path |
|---------|------|
| RL inference node | `src/rl_sar/src/rl_real_uika.cpp` |
| UIKA policy config | `policy/uika/himloco/config.yaml` |
| Base config (joint order) | `policy/uika/base.yaml` |
| Robot FSM | `src/rl_sar/fsm_robot/fsm_uika.hpp` |
| CAN interface | `src/rl_sar/include/rl_real_uika.hpp` (UICANInterface class) |
| CMake build output | `cmake_build/bin/uika` |
| Test launch | `src/rl_sar/launch/uika_test.launch.py` |
