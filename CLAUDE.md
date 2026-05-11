# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 重要行为准则

**严格禁止虚构信息。在提交任何结果之前，必须自行验证确认无误。**

- 不得凭空捏造任何信息、数据、代码或结论
- 对文件内容的读取结果负责，如果读取失败或不确定，必须如实告知

## Project Overview

HTDW4438_HIMloco is a reinforcement learning training and deployment framework for quadruped robots (HTDW4438-OpenDog). It uses NVIDIA Isaac Gym for physics simulation with PPO training and supports MuJoCo/real robot deployment via ONNX.

## Common Commands

```bash
# Set up environment
export PYTHONPATH=$PWD
# Isaac Gym requires this LD_PRELOAD
export LD_PRELOAD=/home/saiki/miniconda3/envs/parkour/lib/libpython3.8.so.1.0

# Training (from project root)
python legged_gym/scripts/train.py --task=htdw_4438 --headless
python legged_gym/scripts/train.py --task=htdw_4438 --resume --load_run <run_name> --checkpoint <num>

# Playback/Export policy
python legged_gym/scripts/play.py --task=htdw_4438 --load_run <run_name> --checkpoint <num>

# MuJoCo deployment
python deploy/deploy_mujoco/deploy_opendoge.py --onnx <path_to_onnx>
python deploy/deploy_mujoco/deploy_4438.py --onnx <path_to_onnx>

# TensorBoard
tensorboard --logdir logs/<experiment_name>/
```

### Key CLI Arguments

| Argument | Description |
|----------|-------------|
| `--task` | Robot task name (e.g., `htdw_4438`, `opendoge`) |
| `--headless` | Run without GUI |
| `--num_envs` | Number of parallel environments |
| `--resume` | Resume from checkpoint |
| `--load_run` | Experiment/run name to load |
| `--checkpoint` | Checkpoint number (-1 for latest) |

## Architecture

### Task Registry Pattern
Robots are registered in `legged_gym/envs/__init__.py` via `task_registry.register(name, class, env_cfg, train_cfg)`. Available tasks: `a1`, `go1`, `htdw_4438`, `htdw_4438_v2`, `opendoge`.

### Configuration Classes
Robot configs inherit from `LeggedRobotCfg` (env) and `LeggedRobotCfgPPO` (training) in `legged_gym/envs/base/legged_robot_config.py`. Configs use nested classes:
- `init_state` - Initial robot pose, joint angles
- `control` - Control type, stiffness, damping
- `env` - Number of envs, observation dimensions
- `asset` - URDF path, foot name, collision settings
- `terrain` - Terrain type, heightmap settings
- `rewards` - Reward function weights
- `domain_rand` - Randomization parameters (friction, mass, motor strength, delays, disturbances)

### HIM (Hybrid Internal Model) Architecture
- Policy: `HIMActorCritic` in `rsl_rl/rsl_rl/modules/`
- Algorithm: `HIMPPO` in `rsl_rl/rsl_rl/algorithms/`
- Runner: `HIMOnPolicyRunner` in `rsl_rl/rsl_rl/runners/`
- Uses 6-frame observation history for environment parameter inference

### Deployment Pipeline
Isaac Gym training → export via `play.py` → convert to ONNX → deploy in MuJoCo (`deploy_mujoco/`) or on real robot (`deploy_real/` via LCM)

## Key Files

| File | Purpose |
|------|---------|
| `legged_gym/scripts/train.py` | Training entry point |
| `legged_gym/scripts/play.py` | Playback/policy export |
| `legged_gym/utils/task_registry.py` | Environment registration |
| `legged_gym/envs/base/legged_robot_config.py` | Base config classes |
| `rsl_rl/rsl_rl/runners/him_on_policy_runner.py` | HIM training runner |
| `resources/robots/` | URDF, meshes, XML for each robot |
| `deploy/deploy_mujoco/configs/` | YAML configs per robot for deployment |

## Package Structure

```
HTDW4438_HIMloco/
├── legged_gym/               # Isaac Gym environment definitions
│   └── envs/                 # Robot environments
│       ├── base/             # BaseLeggedRobot, LeggedRobotCfg, LeggedRobotCfgPPO
│       ├── htdw_4438/        # HTDW4438-specific config
│       ├── opendoge/         # OpenDoge robot config
│       └── scripts/           # train.py, play.py entry points
├── rsl_rl/                   # RL algorithm (PPO, HIM-PPO)
│   └── rsl_rl/
│       ├── algorithms/        # PPO, HIM-PPO
│       ├── modules/           # Actor-Critic networks
│       ├── runners/           # OnPolicyRunner, HIMOnPolicyRunner
│       └── storage/           # Rollout storage
├── deploy/                   # Deployment scripts
│   ├── deploy_mujoco/         # MuJoCo simulation
│   └── deploy_real/           # Real robot via LCM
└── resources/robots/          # URDF, meshes for each robot
```

## Environment

- Python 3.8-3.10
- `PYTHONPATH` must include project root
- `LEGGED_GYM_ROOT_DIR` is auto-set by the code
- Isaac Gym for training, MuJoCo for deployment

## UIKA Robot Configuration

### Key Files
| File | Purpose |
|------|---------|
| `legged_gym/envs/uika/uika_config.py` | UIKA training config |
| `resources/robots/uika/urdf/uika.urdf` | URDF with joint limits |
| `resources/robots/uika/xml/uika.xml` | MuJoCo model with actuator ctrlrange |
| `deploy/deploy_mujoco/deploy_uika_sim2sim.py` | UIKA MuJoCo deployment |
| `deploy/deploy_mujoco/configs/uika.yaml` | Deployment config |

### Joint Names (12 DOFs)
- Hip: FL_hip_joint, FR_hip_joint, RL_hip_joint, RR_hip_joint
- Thigh: FL_thigh_joint, FR_thigh_joint, RL_thigh_joint, RR_thigh_joint
- Calf: FL_calf_joint, FR_calf_joint, RL_calf_joint, RR_calf_joint

### Motor Specs (额定力矩)
| Joint | Torque Limit |
|-------|-------------|
| Hip/Thigh | 6 Nm |
| Calf | 11.2 Nm |

**Note**: XML `ctrlrange` must match URDF `effort` for consistency.

### PD Control Parameters
```python
stiffness = {'joint': 20.0}   # Default in uika_config.py
damping = {'joint': 0.5}
```
For standing stability issues, try increasing to:
```python
stiffness = {'joint': 40.0}   # Increase if too soft
damping = {'joint': 1.0}       # Increase if jittering
```

## Troubleshooting

### Standing Jitter/Shaking
**Root causes and solutions:**

| Cause | Solution |
|-------|----------|
| PD stiffness too low | Increase `stiffness` from 20 to 40-60 |
| Damping too low | Increase `damping` from 0.5 to 1-3 |
| `stand_still` reward weak | Increase penalty weight to -2.0 or -3.0 |
| `dof_acc` reward weak | Increase to -5e-7 or -1e-6 |
| `smoothness` reward weak | Increase to -0.1 |

### Key Reward Tuning
| Reward | Purpose | Typical Range |
|--------|---------|---------------|
| `stand_still` | Penalize joint motion at zero command | -1.0 to -3.0 |
| `dof_acc` | Penalize joint acceleration (smoothness) | -5e-7 to -1e-6 |
| `action_rate` | Penalize action change rate | -0.01 to -0.05 |
| `smoothness` | Penalize non-smooth actions | -0.05 to -0.1 |

### Deployment PD Control Flow
```
Policy (ONNX) → raw_action → target_dof_pos → PD Controller → tau → data.ctrl
```
- Policy outputs normalized action [-1, 1]
- `target_dof_pos = raw_action * action_scale + default_dof_pos`
- `tau = (target - current) * kp + (0 - velocity) * kd`
- `tau_limit = model.actuator_ctrlrange[:, 1]`

### Force Limit in MuJoCo
- URDF: `<limit effort="XX" ...>` in joint definition
- XML: `ctrlrange="-XX XX"` in actuator definition
- Both must match for consistency

### Torque Limit Guidelines
| Type | Definition | Use Case |
|------|-----------|----------|
| 额定力矩 (Rated) | Continuous operating torque | Safety limit |
| 最大力矩 (Max) | Short-term peak torque | Performance limit |

For RL policies trained with max torque limits, deployment should also use max torque limits to maintain consistency.

## Initial State Configuration
```python
pos = [0, 0, 0.3]  # Base height
default_joint_angles = {
    'FL_hip_joint': -0.7, 'FL_thigh_joint': 0.2, 'FL_calf_joint': 0.7,
    'FR_hip_joint': 0.7, 'FR_thigh_joint': -0.2, 'FR_calf_joint': -0.7,
    'RL_hip_joint': 0.7, 'RL_thigh_joint': 0.2, 'RL_calf_joint': 0.7,
    'RR_hip_joint': -0.7, 'RR_thigh_joint': -0.2, 'RR_calf_joint': -0.7,
}
```

## HIM Architecture
HIM uses 6-frame observation history for environment parameter inference:
- `observations_history: [1, 2, 3, 4, 5, 6]`
- Requires `ObservationBuffer` for deployment
- Four losses: value_loss, surrogate_loss, estimation_loss, swap_loss
