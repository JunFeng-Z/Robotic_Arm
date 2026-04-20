import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# =========================
# 1. Settings
# =========================
file_path = "./build/runlog.txt"   # change to your txt file path
dt = 0.001               # sampling time, e.g. 1 ms

# Different colors for each joint
joint_colors = {
    "Joint 1": "#1f77b4",   # blue
    "Joint 2": "#ff7f0e",   # orange
    "Joint 3": "#2ca02c"    # green
}

plt.rcParams["font.size"] = 11
plt.rcParams["axes.unicode_minus"] = False

# =========================
# 2. Read data
# Each row:
# q1 q2 q3 qd1 qd2 qd3 qdd1 qdd2 qdd3 tau1 tau2 tau3
# =========================
df = pd.read_csv(
    file_path,
    sep=r"[\s,]+",   # supports spaces / tabs / commas
    engine="python",
    header=None
)

df.columns = [
    "q1", "q2", "q3",
    "qd1", "qd2", "qd3",
    "qdd1", "qdd2", "qdd3",
    "tau1", "tau2", "tau3"
]

# Time axis
t = np.arange(len(df)) * dt

# =========================
# 3. Plot
# =========================
fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
fig.suptitle("Robot Joint States and Torques", fontsize=16)

# ---- Joint Position ----
axes[0].plot(t, df["q1"], color=joint_colors["Joint 1"], label="Joint 1", linewidth=2)
axes[0].plot(t, df["q2"], color=joint_colors["Joint 2"], label="Joint 2", linewidth=2)
axes[0].plot(t, df["q3"], color=joint_colors["Joint 3"], label="Joint 3", linewidth=2)
axes[0].set_ylabel("Position (rad)")
axes[0].set_title("Joint Position")
axes[0].legend()
axes[0].grid(True, alpha=0.3)

# ---- Joint Velocity ----
axes[1].plot(t, df["qd1"], color=joint_colors["Joint 1"], label="Joint 1", linewidth=2)
axes[1].plot(t, df["qd2"], color=joint_colors["Joint 2"], label="Joint 2", linewidth=2)
axes[1].plot(t, df["qd3"], color=joint_colors["Joint 3"], label="Joint 3", linewidth=2)
axes[1].set_ylabel("Velocity (rad/s)")
axes[1].set_title("Joint Velocity")
axes[1].legend()
axes[1].grid(True, alpha=0.3)

# ---- Joint Acceleration ----
axes[2].plot(t, df["qdd1"], color=joint_colors["Joint 1"], label="Joint 1", linewidth=2)
axes[2].plot(t, df["qdd2"], color=joint_colors["Joint 2"], label="Joint 2", linewidth=2)
axes[2].plot(t, df["qdd3"], color=joint_colors["Joint 3"], label="Joint 3", linewidth=2)
axes[2].set_ylabel("Acceleration (rad/s²)")
axes[2].set_title("Joint Acceleration")
axes[2].legend()
axes[2].grid(True, alpha=0.3)

# ---- Joint Torque ----
axes[3].plot(t, df["tau1"], color=joint_colors["Joint 1"], label="Joint 1", linewidth=2)
axes[3].plot(t, df["tau2"], color=joint_colors["Joint 2"], label="Joint 2", linewidth=2)
axes[3].plot(t, df["tau3"], color=joint_colors["Joint 3"], label="Joint 3", linewidth=2)
axes[3].set_xlabel("Time (s)")
axes[3].set_ylabel("Torque (Nm)")
axes[3].set_title("Joint Torque")
axes[3].legend()
axes[3].grid(True, alpha=0.3)

plt.tight_layout(rect=[0, 0, 1, 0.97])
plt.show()
