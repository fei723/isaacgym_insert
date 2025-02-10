import torch
import numpy as np
import matplotlib.pyplot as plt

# 定义 x 的范围
x = torch.linspace(0.1, 10, 500)  # 避免 x=0，取正值范围

# 计算对数函数 y = log(x)
y = -torch.log(x)

# 转为 numpy 便于绘制
x_np = x.numpy()
y_np = y.numpy()

# 绘制曲线
plt.figure(figsize=(8, 6))
plt.plot(x_np, y_np, label="y = log(x)", color="blue")
plt.axhline(0, color="black", linewidth=0.8, linestyle="--")  # 添加 y=0 的参考线
plt.axvline(1, color="black", linewidth=0.8, linestyle="--")  # 添加 x=1 的参考线

# 设置图表信息
plt.title("Torch Log Function Curve")
plt.xlabel("x")
plt.ylabel("y = log(x)")
plt.grid()
plt.legend()
plt.show()
