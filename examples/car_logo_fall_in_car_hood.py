"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Convex decomposition example
"""
import os.path

from isaacgym import gymapi
from isaacgym import gymutil

# initialize gym
gym = gymapi.acquire_gym()

# parse arguments
args = gymutil.parse_arguments(description="Convex decomposition example")

# create a simulator
sim_params = gymapi.SimParams()

sim_params.up_axis = gymapi.UP_AXIS_Z
sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)
sim_params.substeps = 2
sim_params.dt = 1.0 / 60.0

sim_params.physx.solver_type = 1
sim_params.physx.num_position_iterations = 4
sim_params.physx.num_velocity_iterations = 1
sim_params.physx.rest_offset = 0
sim_params.physx.contact_offset = 0.001
sim_params.physx.num_threads = args.num_threads
sim_params.physx.use_gpu = args.use_gpu

sim_params.use_gpu_pipeline = False
if args.use_gpu_pipeline:
    print("WARNING: Forcing CPU pipeline.")

# create sim
sim = gym.create_sim(args.compute_device_id, args.graphics_device_id, args.physics_engine, sim_params)
if sim is None:
    raise Exception("Failed to create sim")

# create viewer using the default camera properties
viewer = gym.create_viewer(sim, gymapi.CameraProperties())
if viewer is None:
    raise Exception("Failed to create viewer")

# add ground plane
plane_params = gymapi.PlaneParams()
plane_params.normal = gymapi.Vec3(0, 0, 1)
gym.add_ground(sim, plane_params)

# set up the env grid
num_envs = 4
envs_per_row = 2
spacing = 0.5
env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
env_upper = gymapi.Vec3(spacing, spacing, spacing)

# initial root pose for actors
initial_pose = gymapi.Transform()
initial_pose.p = gymapi.Vec3(0.0, 0.0, 0.2)

asset_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../IsaacGymEnvs/assets/mi_industreal/urdf")
asset_options = gymapi.AssetOptions()

# Load materials from meshes
asset_options.use_mesh_materials = True
asset_options.flip_visual_attachments = False
asset_options.mesh_normal_mode = gymapi.COMPUTE_PER_VERTEX

# Override the bogus inertia tensors and center-of-mass properties in the YCB assets.
# These flags will force the inertial properties to be recomputed from geometry.
asset_options.override_inertia = True
asset_options.override_com = True

# don't use convex decomposition
asset_options.vhacd_enabled = False
asset_options.fix_base_link = True
# asset_options.default_friction = 0.8
# asset_options.default_restitution = 0.5
# asset_options.default_linear_damping = 0.05
asset_options.thickness = 0.1  # default = 0.02
asset_options.armature = 0.0  # default = 0.0
asset_options.use_physx_armature = True
asset_options.density = 1000
asset_options.max_linear_velocity = 1000.0  # default = 1000.0
asset_options.linear_damping = 0.5  # default = 0.0
asset_options.angular_damping = 0.5  # default = 0.5
asset_options.max_angular_velocity = 64.0  # default = 64.0
asset_options.disable_gravity = False
asset_options.enable_gyroscopic_forces = True
asset_options.default_dof_drive_mode = gymapi.DOF_MODE_NONE
asset_options.use_mesh_materials = False
asset_options.mesh_normal_mode = gymapi.COMPUTE_PER_FACE

asset2 = gym.load_asset(sim, asset_root, "car_hood.urdf", asset_options)

# convex decomposition with custom params
# asset_options.vhacd_enabled = True
# asset_options.vhacd_params = gymapi.VhacdParams()
# asset_options.vhacd_params.resolution = 500000
asset_options.fix_base_link = False
asset3 = gym.load_asset(sim, asset_root, "car_logo.urdf", asset_options)

# create envs
env = gym.create_env(sim, env_lower, env_upper, envs_per_row)
actor = gym.create_actor(env, asset2, initial_pose, 'actor1', 2, 0)
initial_pose.p = gymapi.Vec3(0.0, 0.0, 0.23)
# initial_pose.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(1, 0, 0), -0.25*3.14)
actor = gym.create_actor(env, asset3, initial_pose, 'actor2', 2, 0)

cam_pos = gymapi.Vec3(3, 0, 3)
cam_target = gymapi.Vec3(0, 0, 0)
gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)
frame_id = 0
# Simulate
while not gym.query_viewer_has_closed(viewer):
    if frame_id > 600:
        sim_params = gym.get_sim_params(sim)
        sim_params.gravity.z = -9.8
        gym.set_sim_params(sim, sim_params)
    else:
        sim_params = gym.get_sim_params(sim)
        sim_params.gravity.z = 0.0
        gym.set_sim_params(sim, sim_params)
    # step the physics
    gym.simulate(sim)
    gym.fetch_results(sim, True)

    # update the viewer
    gym.step_graphics(sim)
    gym.draw_viewer(viewer, sim, True)

    # Wait for dt to elapse in real time.
    # This synchronizes the physics simulation with the rendering rate.
    gym.sync_frame_time(sim)
    frame_id += 1

print('Done')

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
