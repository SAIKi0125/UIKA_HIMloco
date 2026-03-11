from isaacgym import gymtorch, gymapi
# import isaac
import os

# 创建isaacgym API代理
gym = gymapi.acquire_gym()

# 修改仿真环境参数，创建仿真环境
sim_params = gymapi.SimParams()
sim_params.dt = 1.0 / 60.0 # 时间步
sim_params.substeps = 4 # 子时间步
sim_params.up_axis = gymapi.UP_AXIS_Z # Z轴朝上
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8) # 修改重力方向为Z轴朝上
sim_params.use_gpu_pipeline = False
sim_params.physx.use_gpu = True # physx使用GPU计算
sim_params.physx.solver_type = 1 # 时间高斯赛德尔求解器
sim_params.physx.num_position_iterations = 6 # 位置迭代器个数
sim_params.physx.num_velocity_iterations = 0 # 速度迭代器个数
sim_params.physx.bounce_threshold_velocity = 0.2 # 小于这个速度的碰撞不会反弹
sim_params.physx.max_depenetration_velocity = 100 # 求解器允许引入的最大速度
sim_params.physx.contact_offset = 0.01 # 判定为碰撞的距离
sim_params.physx.rest_offset = 0.0 # 判定两个物体接触然后静止的距离
sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params) # 创建仿真环境

# 创建地面
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1) # Z轴朝上
plane_params.distance = 1 # 与原点的距离，正数往下，负数往上
plane_params.static_friction = 1 # 静摩擦力
plane_params.dynamic_friction = 1 # 动摩擦力
plane_params.restitution = 0 # 弹性
gym.add_ground(sim, plane_params) # 创建地面
# 导入机器人模型
# asset_root = "resources/robots/galileo" # 指定机器人模型目录，用以搜索mesh等文件
# asset_file = "grq20_v1d6.urdf" # 指定urdf目录
# asset_root = "resources/robots/zsl1/urdf" # 指定机器人模型目录，用以搜索mesh等文件
# asset_file = "DOG.urdf" # 指定urdf目录
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
asset_root = os.path.join(PROJECT_ROOT, "resources", "robots", "htdw_4438", "urdf") # 指定机器人模型目录，用以搜索mesh等文件
asset_file = "htdw_4438.urdf" # 指定urdf目录

asset_options = gymapi.AssetOptions() 
asset_options.fix_base_link = True # 固定机器人的位置
asset_options.armature = 0.01 # 添加到惯性张量对角线元素的值
asset_options.flip_visual_attachments = False
asset_options.use_mesh_materials  = True # 使用mesh中的材质
asset_options.replace_cylinder_with_capsule = False
asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

# myutilits.print_asset_info(gym,asset,"anymal") # 输出asset信息

# 设置机器人个数、现实间距等
num_envs = 1 # 一共多少个
envs_per_row = 8 # 每列多少个
env_spacing = 1.0 # 每个机器人的空间参数
env_lower = gymapi.Vec3(-env_spacing, 0.0, -env_spacing) # 空间的起始位置
env_upper = gymapi.Vec3(env_spacing, env_spacing, env_spacing) # 空间的结束位置

# 用来装生成的环境和机器人句柄的列表
envs = []
actor_handles = []
# 循环创建env和actor
for i in range(num_envs):
    # 创建env
    env = gym.create_env(sim, env_lower, env_upper, envs_per_row)
    envs.append(env)
    # 机器人的位姿
    pose = gymapi.Transform()
    pose.p = gymapi.Vec3(0.0, 0.0, 0.5)
    # 创建actor
    actor_handle = gym.create_actor(env, asset, pose, "MyActor", i, 1)
    dof_props_asset = gym.get_asset_dof_properties(asset) # 修改关节的摩擦系数
    # dof_props_asset["friction"].fill(0.05)
    gym.set_actor_dof_properties(env, actor_handle, dof_props_asset)
    actor_handles.append(actor_handle)



# 创建可视化窗口
cam_props = gymapi.CameraProperties()
camera_position = gymapi.Vec3(2.0, 2.0, 2.0)
camera_target = gymapi.Vec3(0.5, 0.5, 0.5)
viewer = gym.create_viewer(sim, cam_props)
gym.viewer_camera_look_at(viewer, None, camera_position, camera_target)
gym.prepare_sim(sim)

# 获取关节状态
# num_dof = 12
# dof_state_tensor = gym.acquire_dof_state_tensor(sim)
# gym.refresh_actor_root_state_tensor(sim)
# dof_state = gymtorch.wrap_tensor(dof_state_tensor)
# dof_pos = dof_state.view(num_envs, num_dof, 2)[..., 0]
# dof_vel = dof_state.view(num_envs, num_dof, 2)[..., 1]

while not gym.query_viewer_has_closed(viewer):
    # step the physics simulation
    gym.simulate(sim)
    gym.fetch_results(sim, True)
    # refresh the state tensors
    gym.refresh_actor_root_state_tensor(sim)
    # gym.refresh_dof_state_tensor(sim)
    # 可视化计算
    gym.step_graphics(sim) # 进行摄像机画面渲染计算
    gym.draw_viewer(viewer, sim, True) # 现实到可视化窗口

    # print(dof_pos)

# 程序结束后释放资源
gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
