import isaacgym
from isaacgym import gymapi
from isaacgym import gymutil

# 初始化Isaac Gym
gym = gymapi.acquire_gym()

# 创建模拟参数
sim_params = gymapi.SimParams()
sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.dt = 1.0 / 60.0
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.8)
sim_params.physx.solver_type = 1
sim_params.physx.num_position_iterations = 8
sim_params.physx.num_velocity_iterations = 1
sim_params.physx.num_threads = 4
sim_params.physx.use_gpu = True

# 创建物理模拟
sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

# 创建环境
env = gym.create_env(sim, gymapi.Vec3(-2.0, -2.0, 0.0), gymapi.Vec3(2.0, 2.0, 2.0), 1)

# 创建一个物体（刚体 box）
asset_options = gymapi.AssetOptions()
asset_options.density = 1000.0  # 设置密度
box_asset = gym.create_box(sim, 1.0, 1.0, 1.0, asset_options)  # 创建一个1m边长的立方体

# 设置物体初始姿态
pose = gymapi.Transform()
pose.p = gymapi.Vec3(0.0, 0.0, 2.0)

# 加载物体到环境中
actor = gym.create_actor(env, box_asset, pose, "box", 0, 1)

# 获取刚体属性
props = gym.get_actor_rigid_body_properties(env, actor)

# 设置不合理的惯量矩阵
for prop in props:
    inertia_matrix = gymapi.Mat33(
        0.001, 0.0, 0.0,  # 第一行
        0.0, 0.001, 0.0,  # 第二行
        0.0, 0.0, 1000.0  # 第三行
    )
    prop.inertia = inertia_matrix  # 设置惯量矩阵
    prop.mass = 1.0  # 设置质量为 1kg

# 应用刚体属性
gym.set_actor_rigid_body_properties(env, actor, props, recompute_inertia=False)

# 模拟循环
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
while not gym.query_viewer_has_closed(viewer):
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # 渲染
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
