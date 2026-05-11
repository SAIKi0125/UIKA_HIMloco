# Repository Guidelines

## Project Structure & Module Organization
`legged_gym/` contains Isaac Gym environments, task registration, utilities, scripts, and lightweight tests. Robot-specific configs live under `legged_gym/envs/<task>/`, while shared base classes are in `legged_gym/envs/base/`. `rsl_rl/` vendors the PPO/HIM-PPO training stack (`algorithms/`, `modules/`, `runners/`, `storage/`). `deploy/` holds MuJoCo and real-robot deployment code, with per-robot YAML configs in `deploy/deploy_mujoco/configs/` and `deploy/deploy_real/configs/`. Robot URDF, meshes, and XML assets live in `resources/robots/`. Training outputs and exports are written to `logs/` and `onnx/`.

## Build, Test, and Development Commands
Use the current local environment file and shell helper:

```bash
conda env create -f HIMloco.yml
source env.sh
pip install -e .
```

Common workflows:

```bash
python legged_gym/scripts/train.py --task=htdw_4438 --headless
python legged_gym/scripts/play.py --task=htdw_4438 --load_run <run> --checkpoint <id>
python legged_gym/tests/test_env.py --task=htdw_4438
python deploy/deploy_mujoco/deploy_opendoge.py --onnx onnx/<model>.onnx
tensorboard --logdir logs/<experiment_name>/
```

## Coding Style & Naming Conventions
Follow existing Python style: 4-space indentation, `snake_case` for functions/variables/files, and `CamelCase` for config and runner classes such as `LeggedRobotCfg` or `HIMOnPolicyRunner`. Keep task names lowercase with underscores, matching `task_registry.register(...)`. Prefer small, targeted changes; keep task-specific logic inside the corresponding `envs/<task>/` or deployment config directory. No formatter config is checked in, so match surrounding code closely.

## Testing Guidelines
Automated coverage is minimal; validate changes with the closest runnable path. For environment changes, run `python legged_gym/tests/test_env.py --task=<task>` first, then a short `train.py` or `play.py` smoke test. For deployment changes, run the relevant `deploy/deploy_mujoco/` entry point against a known ONNX file. Name new tests `test_<feature>.py`.

## Commit & Pull Request Guidelines
Recent history uses short bracketed subjects such as `[fix]...` and `[add]...`; keep that format and make the scope explicit, for example `[fix] adjust htdw_4438 thigh joint limits`. PRs should include purpose, affected tasks/robots, validation commands run, and screenshots or short videos for simulation/deployment behavior changes.

## Agent-Specific Instructions
Do not fabricate results. Verify file contents, commands, and assumptions before reporting them. If environment names, paths, or model artifacts differ between docs and code, call out the mismatch and prefer the current repository state.

For this repository, the assistant primarily acts as a research assistant for quadruped RL control, including training, policy export, sim2sim, and sim2real deployment. Default behavior is to produce detailed plans and code-review findings rather than writing code. When asked for a review, focus first on bugs, regressions, control-loop risks, sim2real breaks, unsafe outputs, missing validation, and experiment-design flaws. When asked for a plan, default to a Chinese, decision-complete research or execution plan for the user to review. Only generate direct implementation prompts for Claude Code when explicitly requested.

Treat Jetson-specific environment bindings as source-of-truth if the user says they are validated there. Do not flag include paths, generated ROS message headers, package names, or workspace overlay paths such as `interfaces/msg/...` as issues unless the user explicitly asks to audit Jetson environment integration. Prefer reviewing runtime behavior instead: loop structure, inference blocking, command timeout, deadband, zero-command fallback, output safety checks, sensor-ready handling, logging, and thread safety.
