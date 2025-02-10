import trimesh

# 加载一个包含多个部分的mesh
mesh = trimesh.load('industreal_tray_insert_round_peg_8mm.obj')

# 定义裁剪平面
plane_origin = [0, 0, 0.022]  # 平面通过的点
plane_normal = [0, 0, 1]           # 平面法向量（指向 Z 轴正方向）

# 对网格进行裁剪
sliced = mesh.slice_plane(plane_origin=plane_origin, plane_normal=plane_normal)

# 导出裁剪后的网格
sliced.export('socket_up.obj')

