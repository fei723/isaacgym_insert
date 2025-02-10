import trimesh
import numpy as np

# 圆柱体的参数
radius = 0.003      # 半径
height = 0.012      # 高度
sections = 32     # 圆柱周围分段数（分辨率）

# 创建圆柱体
cylinder = trimesh.creation.cylinder(radius=radius, height=height, sections=sections)

# 平移操作：移动到 (x, y, z) = (2, 3, 4)
# 0.013 -0.013 -0.005
translation = np.array([0.013, -0.013, -0.005])
cylinder.apply_translation(translation)

# 保存为 STL 文件
output_file = "translated_cylinder.stl"
cylinder.export(output_file)
print(f"Translated cylinder mesh saved to {output_file}")

# 可视化
cylinder.show()
