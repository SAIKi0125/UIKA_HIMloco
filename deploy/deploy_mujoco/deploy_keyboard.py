import time

import mujoco.viewer
import mujoco
import numpy as np
from legged_gym import LEGGED_GYM_ROOT_DIR
import torch
import yaml
from time import sleep


# 添加暂停控制变量
paused = False

def key_callback(keycode):
  global paused
  if chr(keycode) == ' ':
    paused = not paused

def get_gravity_orientation(quaternion):
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3)

    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

    return gravity_orientation


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd

# 定义全局变量 obs
obs = None

ctrl_f = [0,0] # ctrl_f key_f

def on_press(key):
    ctrl_f[1] = 1
    if key == keyboard.Key.up:
        cmd[0] = 1.0
    elif key == keyboard.Key.down:
        cmd[0] = -1.0
    if key == keyboard.Key.left:
        if ctrl_f[0] == 0:
            cmd[1] = 1.0
        else:
            cmd[2] = 1.0
    elif key == keyboard.Key.right:
        if ctrl_f[0] == 0:
            cmd[1] = -1.0
        else:
            cmd[2] = -1.0
    if key == keyboard.Key.ctrl:
        ctrl_f[0] = 1

def on_release(key):
    ctrl_f[1] = 0
    if key == keyboard.Key.up:
        cmd[0] = 0.0
    elif key == keyboard.Key.down:
        cmd[0] = 0.0
    if key == keyboard.Key.left:
        cmd[1] = 0.0
        cmd[2] = 0.0
    elif key == keyboard.Key.right:
        cmd[1] = 0.0
        cmd[2] = 0.0
    if key == keyboard.Key.ctrl:
        ctrl_f[0] = 0
        cmd[2] = 0.0



if __name__ == "__main__":
    # get config file name from command line
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("config_file", type=str, help="config file name in the config folder")
    args = parser.parse_args()
    config_file = args.config_file
    with open(f"{LEGGED_GYM_ROOT_DIR}/deploy/deploy_mujoco/configs/{config_file}", "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        policy_path = config["policy_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)
        xml_path = config["xml_path"].replace("{LEGGED_GYM_ROOT_DIR}", LEGGED_GYM_ROOT_DIR)

        simulation_duration = config["simulation_duration"]
        simulation_dt = config["simulation_dt"]
        control_decimation = config["control_decimation"]

        kps = np.array(config["kps"], dtype=np.float32)
        kds = np.array(config["kds"], dtype=np.float32)

        default_angles = np.array(config["default_angles"], dtype=np.float32)

        lin_vel_scale = config["lin_vel_scale"]
        ang_vel_scale = config["ang_vel_scale"]
        dof_pos_scale = config["dof_pos_scale"]
        dof_vel_scale = config["dof_vel_scale"]
        action_scale = config["action_scale"]
        cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)

        num_actions = config["num_actions"]
        num_obs = config["num_obs"]
        num_one_step_obs = config["num_one_step_obs"]
        
        cmd = np.array(config["cmd_init"], dtype=np.float32)

    # define context variables
    action = np.zeros(num_actions, dtype=np.float32)
    target_dof_pos = default_angles.copy()
    obs = np.zeros(num_obs, dtype=np.float32)
    current_obs = np.zeros(num_one_step_obs, dtype=np.float32)

    counter = 0

    # Load robot model
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    model.opt.timestep = simulation_dt

    # height
    # d.qpos[0:3] = [0, 0, 0.52]
    # mujoco.mj_forward(m, d)

    # load policy
    policy = torch.jit.load(policy_path)

    #ctrl
    from pynput import keyboard

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()


    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        # Close the viewer automatically after simulation_duration wall-seconds.
        start = time.time()
        # height
        # data.qpos[0:3] = [0, 0, 0.52]
        # mujoco.mj_forward(model, data)
        while viewer.is_running() and time.time() - start < simulation_duration:
            step_start = time.time()
            if not paused:
                tau = pd_control(target_dof_pos, data.qpos[7:], kps, np.zeros_like(kds), data.qvel[6:], kds)
                # 检查tau是否有NaN, Inf或过大的值
                # if np.any(np.isnan(tau)) or np.any(np.isinf(tau)) or np.any(np.abs(tau) > 1e6):
                #     print(f"Warning: Abnormal tau detected at time {data.time:.4f}. Tau: {tau}")
                #     # 将异常tau置零，避免模拟崩溃
                #     tau = np.zeros_like(tau)

                data.ctrl[:] = tau
                # mj_step can be replaced with code that also evaluates
                # a policy and applies a control signal before stepping the physics.
                mujoco.mj_step(model, data)

                counter += 1
                if counter % control_decimation == 0:
                    # Apply control signal here.
                    
                    # create observation
                    qj = data.qpos[7:]
                    dqj = data.qvel[6:]
                    quat = data.qpos[3:7]
                    lin_vel = data.qvel[:3]
                    ang_vel = data.qvel[3:6]

                    qj = (qj - default_angles) * dof_pos_scale

                    dqj = dqj * dof_vel_scale
                    gravity_orientation = get_gravity_orientation(quat)
                    lin_vel = lin_vel * lin_vel_scale
                    ang_vel = ang_vel * ang_vel_scale


                    # current_obs = torch.cat((   self.commands[:, :3] * self.commands_scale,
                    #                             self.base_ang_vel  * self.obs_scales.ang_vel,
                    #                             self.projected_gravity,
                    #                             (self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos,
                    #                             self.dof_vel * self.obs_scales.dof_vel,
                    #                             self.actions
                    #                             ),dim=-1)
                    

                    current_obs[:3] = cmd * cmd_scale
                    current_obs[3:6] = ang_vel
                    current_obs[6:9] = gravity_orientation
                    current_obs[9 : 9 + num_actions] = qj
                    current_obs[9 + num_actions : 9 + 2 * num_actions] = dqj
                    current_obs[9 + 2 * num_actions : 9 + 3 * num_actions] = action

                    # 将当前观测数据添加到 obs 的开头，并将历史数据向前移动
                    obs = np.concatenate((current_obs, obs[:-num_one_step_obs]))

                    
                    obs_tensor = torch.from_numpy(obs).unsqueeze(0)
                    # policy inference
                    action = policy(obs_tensor).detach().numpy().squeeze()
                    # transform action to target_dof_pos
                    target_dof_pos = action * action_scale + default_angles

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)