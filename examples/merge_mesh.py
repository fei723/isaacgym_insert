import trimesh

# 加载两个网格
mesh1 = trimesh.load('socket_up.obj')
mesh2 = trimesh.load('socket_below.obj')

# 对 mesh2 进行平移
translation_matrix = trimesh.transformations.translation_matrix([0, 0, -0.012])  # 平移1个单位
mesh1.apply_transform(translation_matrix)

# 合并网格
combined_mesh = trimesh.util.concatenate([mesh1, mesh2])

# 保存合并后的网格
combined_mesh.export('socket.obj')
