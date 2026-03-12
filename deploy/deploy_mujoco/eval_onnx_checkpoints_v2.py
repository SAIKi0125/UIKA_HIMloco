import argparse
import csv
import glob
import json
import os
from collections import deque
from datetime import datetime

import mujoco
import numpy as np
import onnxruntime as ort
import yaml


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
DEFAULT_YAML_PATH = os.path.join(SCRIPT_DIR, "configs", "htdw_4438_v2.yaml")
DEFAULT_XML_PATH = os.path.join(PROJECT_ROOT, "resources", "robots", "htdw_4438V2", "xml", "scene.xml")
DEFAULT_ONNX_GLOB = os.path.join(PROJECT_ROOT, "onnx", "flat_htdw_4438_v2*.onnx")


def quat_rotate_inverse(q, v):
    q_w = q[0]
    q_vec = q[1:4]
    a = v * (2.0 * q_w ** 2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c


def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd


def parse_cases(cases_text):
    if not cases_text:
        return [
            ("stand", np.array([0.0, 0.0, 0.0], dtype=np.float32)),
            ("fwd_0p5", np.array([0.5, 0.0, 0.0], dtype=np.float32)),
            ("fwd_1p0", np.array([1.0, 0.0, 0.0], dtype=np.float32)),
            ("strafe_0p4", np.array([0.0, 0.4, 0.0], dtype=np.float32)),
            ("yaw_0p8", np.array([0.0, 0.0, 0.8], dtype=np.float32)),
        ]

    cases = []
    for idx, item in enumerate(cases_text.split(";")):
        item = item.strip()
        if not item:
            continue
        parts = [p.strip() for p in item.split(",")]
        if len(parts) != 3:
            raise ValueError(f"Invalid case '{item}'. Expected format: vx,vy,wz")
        cmd = np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=np.float32)
        cases.append((f"case_{idx}", cmd))
    if not cases:
        raise ValueError("No valid cases parsed from --cases.")
    return cases


def load_deploy_config(yaml_path):
    with open(yaml_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    return {
        "sim_dt": float(cfg.get("simulation_dt", 0.005)),
        "control_decimation": int(cfg.get("control_decimation", 2)),
        "kps": np.array(cfg["kps"], dtype=np.float32),
        "kds": np.array(cfg["kds"], dtype=np.float32),
        "default_dof_pos": np.array(cfg["default_angles"], dtype=np.float32),
        "ang_vel_scale": float(cfg["ang_vel_scale"]),
        "dof_pos_scale": float(cfg["dof_pos_scale"]),
        "dof_vel_scale": float(cfg["dof_vel_scale"]),
        "action_scale": float(cfg["action_scale"]),
        "cmd_scale": np.array(cfg["cmd_scale"], dtype=np.float32),
        "num_obs": int(cfg.get("num_obs", 270)),
        "num_one_step_obs": int(cfg.get("num_one_step_obs", 45)),
        "init_base_height": float(cfg.get("init_base_height", 0.15)),
    }


def build_policy_input(obs_raw, history_buffer, input_dim, num_obs):
    if input_dim == num_obs:
        history_buffer.appendleft(obs_raw.copy())
        return np.concatenate(list(history_buffer), axis=0).reshape(1, -1)
    if input_dim == 64:
        policy_input = np.zeros((1, 64), dtype=np.float32)
        policy_input[0, :45] = obs_raw
        return policy_input
    if input_dim == 45:
        return obs_raw.reshape(1, -1)
    raise ValueError(f"Unsupported ONNX input dim: {input_dim}")


def evaluate_one_case(model, ort_session, cfg, cmd, duration_s, fall_height):
    data = mujoco.MjData(model)
    data.qpos[7:] = cfg["default_dof_pos"]
    data.qpos[2] = cfg["init_base_height"]
    mujoco.mj_forward(model, data)

    use_gyro_sensor = True
    use_linear_vel_sensor = True
    try:
        _ = data.sensor("angular-velocity").data
    except KeyError:
        use_gyro_sensor = False
    try:
        _ = data.sensor("linear-velocity").data
    except KeyError:
        use_linear_vel_sensor = False

    input_name = ort_session.get_inputs()[0].name
    input_shape = ort_session.get_inputs()[0].shape
    input_dim = int(input_shape[-1]) if isinstance(input_shape[-1], int) else cfg["num_obs"]

    history_len = max(1, cfg["num_obs"] // cfg["num_one_step_obs"])
    history_buffer = deque(
        [np.zeros(cfg["num_one_step_obs"], dtype=np.float32) for _ in range(history_len)],
        maxlen=history_len,
    )

    target_dof_pos = cfg["default_dof_pos"].copy()
    action = np.zeros(12, dtype=np.float32)
    prev_action = action.copy()

    total_steps = int(duration_s / cfg["sim_dt"])
    policy_steps = 0
    sum_lin_err = 0.0
    sum_yaw_err = 0.0
    sum_tau_abs = 0.0
    sum_action_rate = 0.0
    fell = False

    for step in range(total_steps):
        if step % cfg["control_decimation"] == 0:
            qj = data.qpos[7:].astype(np.float32)
            dqj = data.qvel[6:].astype(np.float32)
            quat = data.qpos[3:7].astype(np.float32)

            if use_gyro_sensor:
                omega = data.sensor("angular-velocity").data.astype(np.float32)
            else:
                omega = data.qvel[3:6].astype(np.float32)

            gravity_vec = np.array([0.0, 0.0, -1.0], dtype=np.float32)
            proj_gravity = quat_rotate_inverse(quat, gravity_vec)

            obs_raw = np.concatenate(
                [
                    cmd * cfg["cmd_scale"],
                    omega * cfg["ang_vel_scale"],
                    proj_gravity,
                    (qj - cfg["default_dof_pos"]) * cfg["dof_pos_scale"],
                    dqj * cfg["dof_vel_scale"],
                    action,
                ],
                axis=0,
            ).astype(np.float32)

            policy_input = build_policy_input(obs_raw, history_buffer, input_dim, cfg["num_obs"])
            raw_action = ort_session.run(None, {input_name: policy_input})[0][0].astype(np.float32)
            raw_action = np.clip(raw_action, -10.0, 10.0)

            target_dof_pos = raw_action * cfg["action_scale"] + cfg["default_dof_pos"]
            sum_action_rate += float(np.mean(np.abs(raw_action - prev_action)))
            prev_action = raw_action.copy()
            action = raw_action

            if use_linear_vel_sensor:
                base_lin_vel = data.sensor("linear-velocity").data.astype(np.float32)
            else:
                base_lin_vel = data.qvel[0:3].astype(np.float32)
            base_ang_vel = omega

            sum_lin_err += abs(float(cmd[0] - base_lin_vel[0])) + abs(float(cmd[1] - base_lin_vel[1]))
            sum_yaw_err += abs(float(cmd[2] - base_ang_vel[2]))
            policy_steps += 1

        tau = pd_control(
            target_dof_pos,
            data.qpos[7:].astype(np.float32),
            cfg["kps"],
            np.zeros_like(cfg["kds"]),
            data.qvel[6:].astype(np.float32),
            cfg["kds"],
        )
        tau_limit = np.abs(model.actuator_ctrlrange[:, 1]).astype(np.float32)
        tau = np.clip(tau, -tau_limit, tau_limit)
        data.ctrl[:] = tau
        sum_tau_abs += float(np.mean(np.abs(tau)))

        mujoco.mj_step(model, data)

        if data.qpos[2] < fall_height:
            fell = True
            break

    denom_policy = max(policy_steps, 1)
    denom_total = max(step + 1, 1)
    return {
        "fell": int(fell),
        "lin_vel_mae": sum_lin_err / denom_policy / 2.0,
        "yaw_rate_mae": sum_yaw_err / denom_policy,
        "tau_abs_mean": sum_tau_abs / denom_total,
        "action_rate_mean": sum_action_rate / denom_policy,
        "policy_steps": policy_steps,
        "sim_steps": denom_total,
        "onnx_input_dim": input_dim,
    }


def evaluate_one_model(onnx_path, xml_path, cfg, cases, duration_s, fall_height):
    model = mujoco.MjModel.from_xml_path(xml_path)
    model.opt.timestep = cfg["sim_dt"]
    ort_session = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])

    case_metrics = {}
    for case_name, cmd in cases:
        case_metrics[case_name] = evaluate_one_case(
            model=model,
            ort_session=ort_session,
            cfg=cfg,
            cmd=cmd,
            duration_s=duration_s,
            fall_height=fall_height,
        )

    agg = {
        "model": os.path.basename(onnx_path),
        "path": onnx_path,
        "cases": case_metrics,
    }

    fell_sum = sum(v["fell"] for v in case_metrics.values())
    agg["fall_rate"] = fell_sum / max(len(case_metrics), 1)
    agg["lin_vel_mae_mean"] = float(np.mean([v["lin_vel_mae"] for v in case_metrics.values()]))
    agg["yaw_rate_mae_mean"] = float(np.mean([v["yaw_rate_mae"] for v in case_metrics.values()]))
    agg["tau_abs_mean"] = float(np.mean([v["tau_abs_mean"] for v in case_metrics.values()]))
    agg["action_rate_mean"] = float(np.mean([v["action_rate_mean"] for v in case_metrics.values()]))
    agg["onnx_input_dim"] = int(next(iter(case_metrics.values()))["onnx_input_dim"])
    return agg


def rank_key(item):
    # 先按不摔倒，再按跟踪误差，再按控制代价
    return (
        item["fall_rate"],
        item["lin_vel_mae_mean"],
        item["yaw_rate_mae_mean"],
        item["tau_abs_mean"],
        item["action_rate_mean"],
    )


def save_reports(results, report_dir):
    os.makedirs(report_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = os.path.join(report_dir, f"onnx_eval_{timestamp}.json")
    csv_path = os.path.join(report_dir, f"onnx_eval_{timestamp}.csv")

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "rank",
                "model",
                "onnx_input_dim",
                "fall_rate",
                "lin_vel_mae_mean",
                "yaw_rate_mae_mean",
                "tau_abs_mean",
                "action_rate_mean",
                "path",
            ]
        )
        for rank, item in enumerate(results, start=1):
            writer.writerow(
                [
                    rank,
                    item["model"],
                    item["onnx_input_dim"],
                    f"{item['fall_rate']:.4f}",
                    f"{item['lin_vel_mae_mean']:.6f}",
                    f"{item['yaw_rate_mae_mean']:.6f}",
                    f"{item['tau_abs_mean']:.6f}",
                    f"{item['action_rate_mean']:.6f}",
                    item["path"],
                ]
            )

    return json_path, csv_path


def main():
    parser = argparse.ArgumentParser(description="Batch evaluate ONNX checkpoints in MuJoCo.")
    parser.add_argument("--yaml", type=str, default=DEFAULT_YAML_PATH, help="Deploy YAML config path.")
    parser.add_argument("--xml", type=str, default=DEFAULT_XML_PATH, help="MuJoCo scene XML path.")
    parser.add_argument("--onnx_glob", type=str, default=DEFAULT_ONNX_GLOB, help="Glob pattern for ONNX files.")
    parser.add_argument("--duration", type=float, default=15.0, help="Simulation duration per test case (seconds).")
    parser.add_argument("--fall_height", type=float, default=0.12, help="Base height threshold for fall.")
    parser.add_argument(
        "--cases",
        type=str,
        default="",
        help="Semicolon-separated commands, e.g. '0,0,0;0.5,0,0;0,0,0.8'.",
    )
    parser.add_argument(
        "--report_dir",
        type=str,
        default=os.path.join(SCRIPT_DIR, "reports"),
        help="Directory to save evaluation reports.",
    )
    args = parser.parse_args()

    yaml_path = os.path.abspath(os.path.expanduser(args.yaml))
    xml_path = os.path.abspath(os.path.expanduser(args.xml))
    onnx_paths = sorted(glob.glob(os.path.abspath(os.path.expanduser(args.onnx_glob))))
    cases = parse_cases(args.cases)

    if not onnx_paths:
        raise FileNotFoundError(f"No ONNX files matched: {args.onnx_glob}")
    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"XML path not found: {xml_path}")
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"YAML path not found: {yaml_path}")

    cfg = load_deploy_config(yaml_path)
    print(f"[INFO] Evaluating {len(onnx_paths)} ONNX files on {len(cases)} cases.")
    print(f"[INFO] XML: {xml_path}")
    print(f"[INFO] YAML: {yaml_path}")
    for case_name, cmd in cases:
        print(f"[INFO] Case {case_name}: cmd={cmd.tolist()}")

    results = []
    for p in onnx_paths:
        print(f"[RUN] {os.path.basename(p)}")
        result = evaluate_one_model(
            onnx_path=p,
            xml_path=xml_path,
            cfg=cfg,
            cases=cases,
            duration_s=args.duration,
            fall_height=args.fall_height,
        )
        results.append(result)
        print(
            "[RES] "
            f"fall_rate={result['fall_rate']:.2f}, "
            f"lin_mae={result['lin_vel_mae_mean']:.4f}, "
            f"yaw_mae={result['yaw_rate_mae_mean']:.4f}, "
            f"tau={result['tau_abs_mean']:.4f}, "
            f"act_rate={result['action_rate_mean']:.4f}, "
            f"input_dim={result['onnx_input_dim']}"
        )

    results_sorted = sorted(results, key=rank_key)
    json_path, csv_path = save_reports(results_sorted, args.report_dir)

    print("\n=== Ranking ===")
    for idx, item in enumerate(results_sorted, start=1):
        print(
            f"{idx:02d}. {item['model']} | fall={item['fall_rate']:.2f} | "
            f"lin={item['lin_vel_mae_mean']:.4f} | yaw={item['yaw_rate_mae_mean']:.4f} | "
            f"tau={item['tau_abs_mean']:.4f} | act_rate={item['action_rate_mean']:.4f} | "
            f"input={item['onnx_input_dim']}"
        )

    print(f"\nSaved JSON: {json_path}")
    print(f"Saved CSV : {csv_path}")


if __name__ == "__main__":
    main()
