import trimesh
import pysdf
import numpy as np
import warp as wp

wp.init()


@wp.kernel
def transform_points(
        src: wp.array(dtype=wp.vec3), dest: wp.array(dtype=wp.vec3), xform: wp.transform
):
    tid = wp.tid()

    p = src[tid]
    m = wp.transform_point(xform, p)

    dest[tid] = m


# Step 1: 加载 OBJ 文件
obj_path = "/root/PycharmProjects/isaacgyms/IsaacGymEnvs/assets/industreal/mesh/industreal_pegs/industreal_tray_pick_round_peg_4mm.obj"  # 替换为你的 .obj 文件路径
obj_path = "/root/PycharmProjects/isaacgyms/IsaacGymEnvs/assets/industreal/mesh/industreal_pegs/industreal_tray_insert_round_peg_4mm.obj"  # 替换为你的 .obj 文件路径
obj_path = "/root/PycharmProjects/isaacgyms/IsaacGymEnvs/assets/industreal/mesh/industreal_pegs/industreal_round_peg_4mm.obj"  # 替换为你的 .obj 文件路径
mesh = trimesh.load(obj_path)

# Step 2: 确保网格是三角化的（OBJ 文件可能包含非三角形面）
if not mesh.is_watertight:
    print("Warning: The mesh is not watertight. SDF results may be inaccurate.")
if not mesh.is_winding_consistent:
    mesh.fix_normals()

# Step 3: 提取顶点和面数据
verts = np.ascontiguousarray(mesh.vertices, dtype=np.float32)  # 顶点数组 (float32)
faces = np.ascontiguousarray(mesh.faces, dtype=np.uint32)  # 面数组 (uint32)

# Step 4: 构建 SDF 对象
sdf = pysdf.SDF(verts, faces)

sampled_points, _ = trimesh.sample.sample_surface_even(mesh, 10)

wp_mesh_sampled_points = wp.array(sampled_points, dtype=wp.vec3, device="cuda:0")

# curr_transform = wp.transform([0, 0.05, 0.02], [0, 0, 0, 1])
# wp.launch(
#     kernel=transform_points,
#     dim=len(wp_mesh_sampled_points),
#     inputs=[wp_mesh_sampled_points, wp_mesh_sampled_points, curr_transform],
#     device="cuda:0",
# )

# Step 5: 测试点到网格表面的距离
points = np.array([
    [0.0, 0.02, 0.0],  # 点在物体内部+
    [0.0, 0.02, 0.003],  # 点在物体内部+
    [0.0, 0.0, 0.003],  # 点在物体内部+
    [0.0, 0.0024, 0.028],  # 点在物体内部+
], dtype=np.float32)

# 计算每个点的 SDF 值
# for i, point in enumerate(points):
#     distance = sdf(point)
#     is_ins = sdf.contains(point)
#     print(f"Point {i} at {point} has a signed distance of {distance[0]:.4f}, is_ins: {is_ins[0]}")

# Step 3: 创建一个球体来表示每个标记点
# 使用 Trimesh 的场景支持点的可视化
scene = trimesh.Scene()  # 创建场景
scene.add_geometry(mesh)  # 添加网格
for point in wp_mesh_sampled_points.numpy():
    point[0] *= 0.1
    point[1] *= 0.1
    marker = trimesh.primitives.Sphere(radius=0.0001, center=point)  # 标记点的小球
    marker.visual.face_colors = [255, 0, 0, 255]  # 红色小球
    scene.add_geometry(marker)  # 添加标记点到场景

    distance = sdf(point)
    is_ins = sdf.contains(point)
    print(f"Point {point} has a signed distance of {distance[0]:.4f}, is_ins: {is_ins[0]}")
    # 在物体内部+ 外部- 表面0

# Step 4: 显示网格和标记点
scene.show()
