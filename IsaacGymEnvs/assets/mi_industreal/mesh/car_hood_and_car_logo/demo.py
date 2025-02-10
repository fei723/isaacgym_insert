"""
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.


Convex decomposition example
"""

from isaacgym import gymapi
from isaacgym import gymutil, gymtorch
import numpy as np
import rospy
import tf
from geometry_msgs.msg import TransformStamped

rospy.init_node('tf_broadcaster_node')
br = tf.TransformBroadcaster()
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
car_hood_height = 1.0
theta = 0 / 180 * np.pi
car_logo_delta_y = 0.0
car_logo_delta_z = 0.5

envs_per_row = 5
spacing = 1.0
env_lower = gymapi.Vec3(-spacing, -spacing, -spacing)
env_upper = gymapi.Vec3(spacing, spacing, spacing)

envs = []
car_hood_actor_handles = []
car_logo_actor_handles = []
car_logo_idxs = []
car_hood_idxs = []
asset_root = "/root/isaacgym/assets"
def creat_env():
    env = gym.create_env(sim, env_lower, env_upper, envs_per_row)
    envs.append(env)

    asset_options = gymapi.AssetOptions()

    # Load materials from meshes
    asset_options.use_mesh_materials = True
    asset_options.mesh_normal_mode = gymapi.COMPUTE_PER_VERTEX

    # Override the bogus inertia tensors and center-of-mass properties in the YCB assets.
    # These flags will force the inertial properties to be recomputed from geometry.
    asset_options.override_inertia = True
    asset_options.override_com = True

    # use default convex decomposition params
    asset_options.vhacd_enabled = True
    asset0 = gym.load_asset(sim, asset_root, "urdf/ycb/011_banana/011_banana.urdf", asset_options)
    asset2 = gym.load_asset(sim, asset_root, "urdf/ycb/061_foam_brick/061_foam_brick.urdf", asset_options)
    asset3 = gym.load_asset(sim, asset_root, "urdf/ycb/010_potted_meat_can/010_potted_meat_can.urdf", asset_options)
    # initial root pose for actors
    initial_pose = gymapi.Transform()
    initial_pose.p = gymapi.Vec3(0.0, 0.005, car_hood_height+0.05)
    initial_pose.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(1, 0, 0), -theta)
    # actor = gym.create_actor(env, asset0, initial_pose, 'actor', len(envs), 0, 0)

    car_hood_options = gymapi.AssetOptions()
    car_hood_options.use_mesh_materials = True
    car_hood_options.mesh_normal_mode = gymapi.COMPUTE_PER_VERTEX
    car_hood_options.flip_visual_attachments = False
    car_hood_options.fix_base_link = True
    car_hood_options.override_com = True
    car_hood_options.override_inertia = True
    # car_hood_options.vhacd_enabled = True
    # car_hood_options.vhacd_params = gymapi.VhacdParams()
    # car_hood_options.vhacd_params.resolution = 300000
    car_hood_asset = gym.load_asset(sim, "./urdf", "car_hood.urdf", car_hood_options)

    # initial root pose for actors
    car_hood_initial_pose = gymapi.Transform()
    car_hood_initial_pose.p = gymapi.Vec3(0.0, 0.0, car_hood_height)
    car_hood_initial_pose.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(1, 0, 0), -theta)
    car_hood_actor = gym.create_actor(env, car_hood_asset, car_hood_initial_pose, 'car_hood_asset', len(envs), 0)
    car_hood_actor_handles.append(car_hood_actor)
    car_hood_idx = gym.get_actor_rigid_body_index(env, car_hood_actor, 0, gymapi.DOMAIN_SIM)
    car_hood_idxs.append(car_hood_idx)


    car_logo_options = gymapi.AssetOptions()
    car_logo_options.use_mesh_materials = True
    car_logo_options.mesh_normal_mode = gymapi.COMPUTE_PER_VERTEX
    car_logo_options.flip_visual_attachments = False
    car_logo_options.fix_base_link = False
    car_logo_options.override_com = True
    car_logo_options.override_inertia = True
    # car_logo_options.vhacd_enabled = True
    # car_logo_options.vhacd_params = gymapi.VhacdParams()
    # car_logo_options.vhacd_params.resolution = 300000

    car_logo_asset = gym.load_asset(sim, "./urdf", "car_logo.urdf", car_logo_options)
    car_logo_initial_pose = gymapi.Transform()
    car_logo_initial_pose.p = gymapi.Vec3(0.0, car_logo_delta_y, car_hood_height + car_logo_delta_z)
    y = -(car_logo_delta_z * np.tan(theta) - car_logo_delta_y) * np.cos(theta)
    z = car_logo_delta_z / np.cos(theta) - (car_logo_delta_z*np.tan(theta)-car_logo_delta_y)*np.cos(theta)*np.tan(theta)

    car_logo_initial_pose.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(1, 0, 0), -theta)
    car_logo_actor = gym.create_actor(env, car_logo_asset, car_logo_initial_pose, 'car_logo_asset', len(envs), 0)
    car_logo_actor_handles.append(car_logo_actor)

    car_logo_idx = gym.get_actor_rigid_body_index(env, car_logo_actor, 0, gymapi.DOMAIN_SIM)
    car_logo_idxs.append(car_logo_idx)


# box_size = 0.01
# box_asset_options = gymapi.AssetOptions()
# box_asset_options.disable_gravity = False
# box_asset = gym.create_box(sim, box_size, box_size, box_size, box_asset_options)
# box_initial_pose = gymapi.Transform()
# box_initial_pose.p = gymapi.Vec3(0.063, 0.0, 0.50)
# box_initial_pose.r = gymapi.Quat.from_axis_angle(gymapi.Vec3(1, 0, 0), -0.5 * np.pi)
# box_actor = gym.create_actor(env, box_asset, initial_pose, 'car_logo_asset', 4)

creat_env()
# creat_env()
cam_pos = gymapi.Vec3(1, 0, 1)
cam_target = gymapi.Vec3(0, 0, 0)
gym.viewer_camera_look_at(viewer, envs[0], cam_pos, cam_target)


num_bodies = gym.get_env_rigid_body_count(envs[0])
body_state = gymtorch.wrap_tensor(gym.acquire_rigid_body_state_tensor(sim))


def publish_tf():
    gym.refresh_rigid_body_state_tensor(sim)
    car_hood_poses = body_state[car_hood_idxs, :7]
    car_logo_poses = body_state[car_logo_idxs, :7]
    print(["{:.4f}".format(val) for val in car_hood_poses[0].numpy()])
    print(["{:.4f}".format(val) for val in car_logo_poses[0].numpy()])
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = 'world'
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.child_frame_id = 'car_hood'
    tf_msg.transform.translation.x = car_hood_poses[0][0]
    tf_msg.transform.translation.y = car_hood_poses[0][1]
    tf_msg.transform.translation.z = car_hood_poses[0][2]
    tf_msg.transform.rotation.w = car_hood_poses[0][3]
    tf_msg.transform.rotation.x = car_hood_poses[0][4]
    tf_msg.transform.rotation.y = car_hood_poses[0][5]
    tf_msg.transform.rotation.z = car_hood_poses[0][6]
    br.sendTransformMessage(tf_msg)
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = 'world'
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.child_frame_id = 'car_logo'
    tf_msg.transform.translation.x = car_logo_poses[0][0]
    tf_msg.transform.translation.y = car_logo_poses[0][1]
    tf_msg.transform.translation.z = car_logo_poses[0][2]
    tf_msg.transform.rotation.w = car_logo_poses[0][3]
    tf_msg.transform.rotation.x = car_logo_poses[0][4]
    tf_msg.transform.rotation.y = car_logo_poses[0][5]
    tf_msg.transform.rotation.z = car_logo_poses[0][6]
    br.sendTransformMessage(tf_msg)


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

    # publish_tf()

print('Done')

gym.destroy_viewer(viewer)
gym.destroy_sim(sim)
