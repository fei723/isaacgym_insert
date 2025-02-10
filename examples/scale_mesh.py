import trimesh

# 加载网格
mesh = trimesh.load('car_logo.obj')

# 缩放网格：缩放比例为 0.001
mesh.apply_scale(0.001)

# 保存缩放后的网格
mesh.export('car_logo/car_logo_scaled.obj')

# 可视化缩放后的网格
mesh.show()
