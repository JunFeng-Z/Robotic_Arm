import pandas as pd
import matplotlib.pyplot as plt

# ===== 1. 文件路径 =====
file_path = "./build/joint_trajectory_log.txt"   # 改成你的txt文件名

# ===== 2. 读取数据 =====
df = pd.read_csv(file_path, sep=",")

# ===== 3. 取时间列 =====
t = df["time"]

# ===== 4. 画关节角度 q =====
plt.figure(figsize=(10, 5))
plt.plot(t, df["q1"], label="q1")
plt.plot(t, df["q2"], label="q2")
plt.plot(t, df["q3"], label="q3")
plt.xlabel("Time (s)")
plt.ylabel("Joint Position")
plt.title("Joint Position")
plt.legend()
plt.grid(True)
plt.tight_layout()

# ===== 5. 画关节速度 qd =====
plt.figure(figsize=(10, 5))
plt.plot(t, df["qd1"], label="qd1")
plt.plot(t, df["qd2"], label="qd2")
plt.plot(t, df["qd3"], label="qd3")
plt.xlabel("Time (s)")
plt.ylabel("Joint Velocity")
plt.title("Joint Velocity")
plt.legend()
plt.grid(True)
plt.tight_layout()

# ===== 6. 画关节加速度 qdd =====
plt.figure(figsize=(10, 5))
plt.plot(t, df["qdd1"], label="qdd1")
plt.plot(t, df["qdd2"], label="qdd2")
plt.plot(t, df["qdd3"], label="qdd3")
plt.xlabel("Time (s)")
plt.ylabel("Joint Acceleration")
plt.title("Joint Acceleration")
plt.legend()
plt.grid(True)
plt.tight_layout()

# ===== 7. 显示图像 =====
plt.show()
