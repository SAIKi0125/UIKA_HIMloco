# UIKA_HIMloco

面向 UIKA 四足机器人的强化学习训练、策略导出与部署的项目。

本项目基于 [HIMLoco](https://github.com/InternRobotics/HIMLoco)，围绕 `uika` 任务整理了从训练、回放到 ONNX 导出和 MuJoCo sim2sim 部署的常用流程。


特别鸣谢 [Lain-Ego0/HTDW4438_HIMloco](https://github.com/Lain-Ego0/HTDW4438_HIMloco) 开源框架。

sim2real 部署部分特别感谢 [fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar) 仓库提供的思路与实现参考。

## 快速开始

```bash
conda env create -f HIMloco.yml
source env.sh
pip install -e .
wandb login
python legged_gym/scripts/train.py --task=uika --proj_name uika --exptid uika_run1
```

## 常用命令

### 训练

```bash
export PYTHONPATH=$PWD
python legged_gym/scripts/train.py --task=uika --proj_name uika --exptid uika_run1
python legged_gym/scripts/train.py --task=uika --proj_name uika --exptid uika_run1 --resume --load_run <run_name> --checkpoint <ckpt>
```

训练默认会上报到 `wandb`。如果只想本地跑，可以加 `--no_wandb`。

### 回放

```bash
export PYTHONPATH=$PWD
python legged_gym/scripts/play.py --task=uika
python legged_gym/scripts/play.py --task=uika --load_run <run_name> --checkpoint <ckpt>
```

### MuJoCo 部署

```bash
python deploy/deploy_mujoco/deploy_uika_sim2sim.py --onnx onnx/<model>.onnx
```

`deploy_uika_sim2sim.py` 使用方向键控制，空格键暂停，并从 `deploy/deploy_mujoco/configs/uika.yaml` 读取部署参数。

## 支持任务

当前 README 以 `uika` 为主：

- `uika`

## 项目结构

```text
legged_gym/          训练环境、任务配置、脚本入口
rsl_rl/              PPO / HIM-PPO 算法实现
deploy/              MuJoCo 与真机部署代码
resources/robots/    URDF、XML、mesh 等机器人资源
logs/                训练输出与导出策略
onnx/                导出的 ONNX 模型
```

## 常见输出位置

- 训练权重：`logs/<experiment>/<run>/model_<iteration>.pt`
- 导出策略：`logs/<experiment>/exported/policies/`
- ONNX 文件：`onnx/*.onnx`

## 常用排查

```bash
python legged_gym/tests/test_env.py --task=uika
```

如果 README 中的命令与实际代码不一致，请以当前仓库脚本和配置文件为准。
