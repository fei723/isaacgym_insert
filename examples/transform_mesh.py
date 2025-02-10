import trimesh

# 加载两个网格
mesh = trimesh.load('socket.obj')

# 对 mesh2 进行平移
# <!-- 0.013, -0.013, -0.041 -->
translation_matrix = trimesh.transformations.translation_matrix([0.013, -0.013, -0.016])
mesh.apply_transform(translation_matrix)

# 保存合并后的网格
mesh.export('socket_left_top.obj')
