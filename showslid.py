import pandas as pd
import matplotlib.pyplot as plt

# =========================
# 1. Basic settings
# =========================
file_path = "./build/control_log.txt"

# 每 STEP 行取 1 行
STEP = 20

# 是否只看某个时间段
USE_TIME_WINDOW = False
T_MIN = 0.0
T_MAX = 20.0

# =========================
# 2. Correct full column names
# =========================
columns = [
    "time",
    "q1", "q2", "q3",
    "qd1", "qd2", "qd3",
    "qdd1", "qdd2", "qdd3",
    "q_des1", "q_des2", "q_des3",
    "qd_des1", "qd_des2", "qd_des3",
    "qdd_des1", "qdd_des2", "qdd_des3",
    "ref_tau1", "ref_tau2", "ref_tau3",
    "control_tau1", "control_tau2", "control_tau3",
    "final_tau1", "final_tau2", "final_tau3",
    "e1", "e2", "e3",
    "V1", "V2", "V3",
    "de1", "de2", "de3"
]

# =========================
# 3. Read file with sampling
# 第0行是旧表头，跳过
# =========================
df = pd.read_csv(
    file_path,
    sep=",",
    header=None,
    names=columns,
    skiprows=lambda i: (i == 0) or ((i - 1) % STEP != 0)
)

# 转数值
for col in columns:
    df[col] = pd.to_numeric(df[col], errors="coerce")

df = df.dropna().reset_index(drop=True)

# =========================
# 4. Optional time window
# =========================
if USE_TIME_WINDOW:
    df = df[(df["time"] >= T_MIN) & (df["time"] <= T_MAX)].reset_index(drop=True)

t = df["time"]

# =========================
# 5. Debug print
# =========================
print("Columns:")
print(df.columns.tolist())

print("\nSampled row count:", len(df))

print("\nFirst 5 rows:")
print(df.head())

print("\nCheck qdd_des:")
print(df[["time", "qdd_des1", "qdd_des2", "qdd_des3"]].head(20))

print("\nqdd_des nonzero count:")
print((df[["qdd_des1", "qdd_des2", "qdd_des3"]].abs() > 1e-12).sum())

print("\nqdd_des max abs:")
print(df[["qdd_des1", "qdd_des2", "qdd_des3"]].abs().max())

# =========================
# 6. Color settings
# =========================
joint_colors = {
    1: ("#1f77b4", "#ff7f0e", "#2ca02c"),
    2: ("#2ca02c", "#d62728", "#9467bd"),
    3: ("#9467bd", "#8c564b", "#e377c2"),
}

torque_colors = {
    "ref": "#1f77b4",
    "control": "#d62728",
    "final": "#2ca02c"
}

# =========================
# 7. Joint Position Tracking
# =========================
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
fig.suptitle("Joint Position Tracking", fontsize=16)

for i in range(3):
    j = i + 1
    actual_color, desired_color, _ = joint_colors[j]
    axes[i].plot(t, df[f"q{j}"], label=f"q{j}", color=actual_color, linewidth=1.8)
    axes[i].plot(t, df[f"q_des{j}"], "--", label=f"q_des{j}", color=desired_color, linewidth=1.8)
    axes[i].set_ylabel(f"Joint {j} Pos")
    axes[i].grid(True, alpha=0.3)
    axes[i].legend(loc="best")

axes[-1].set_xlabel("Time (s)")
plt.tight_layout(rect=[0, 0, 1, 0.96])

# =========================
# 8. Joint Velocity Tracking
# =========================
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
fig.suptitle("Joint Velocity Tracking", fontsize=16)

for i in range(3):
    j = i + 1
    actual_color, desired_color, _ = joint_colors[j]
    axes[i].plot(t, df[f"qd{j}"], label=f"qd{j}", color=actual_color, linewidth=1.8)
    axes[i].plot(t, df[f"qd_des{j}"], "--", label=f"qd_des{j}", color=desired_color, linewidth=1.8)
    axes[i].set_ylabel(f"Joint {j} Vel")
    axes[i].grid(True, alpha=0.3)
    axes[i].legend(loc="best")

axes[-1].set_xlabel("Time (s)")
plt.tight_layout(rect=[0, 0, 1, 0.96])

# =========================
# 9. Joint Acceleration Tracking + V
# 左轴：qdd 和 qdd_des
# 右轴：V
# =========================
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
fig.suptitle("Joint Acceleration Tracking (qdd/qdd_des + V)", fontsize=16)

for i in range(3):
    j = i + 1
    actual_color, desired_color, v_color = joint_colors[j]

    ax1 = axes[i]
    ax2 = ax1.twinx()

    # 左轴：实际加速度 + 期望加速度
    l1, = ax1.plot(t, df[f"qdd{j}"], label=f"qdd{j}", color=actual_color, linewidth=1.8)
    l2, = ax1.plot(t, df[f"qdd_des{j}"], "--", label=f"qdd_des{j}", color=desired_color, linewidth=1.8)

    # 右轴：V
    l3, = ax2.plot(t, df[f"V{j}"], ":", label=f"V{j}", color=v_color, linewidth=2.2)

    ax1.set_ylabel(f"Joint {j} Acc")
    ax2.set_ylabel(f"Joint {j} V")

    ax1.grid(True, alpha=0.3)

    lines = [l1, l2, l3]
    labels = [line.get_label() for line in lines]
    ax1.legend(lines, labels, loc="best")

axes[-1].set_xlabel("Time (s)")
plt.tight_layout(rect=[0, 0, 1, 0.96])

# =========================
# 10. Torque Curves
# =========================
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
fig.suptitle("Torque Curves", fontsize=16)

for i in range(3):
    j = i + 1
    axes[i].plot(t, df[f"ref_tau{j}"], label=f"ref_tau{j}", color=torque_colors["ref"], linewidth=1.8)
    axes[i].plot(t, df[f"control_tau{j}"], "--", label=f"control_tau{j}", color=torque_colors["control"], linewidth=1.8)
    axes[i].plot(t, df[f"final_tau{j}"], ":", label=f"final_tau{j}", color=torque_colors["final"], linewidth=2.2)

    axes[i].set_ylabel(f"Joint {j} Tau")
    axes[i].grid(True, alpha=0.3)
    axes[i].legend(loc="best")

axes[-1].set_xlabel("Time (s)")
plt.tight_layout(rect=[0, 0, 1, 0.96])

# =========================
# 11. Error Curves
# =========================
fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
fig.suptitle("Tracking Error", fontsize=16)

for i in range(3):
    j = i + 1
    axes[i].plot(t, df[f"e{j}"], label=f"e{j}", linewidth=1.8)
    axes[i].plot(t, df[f"de{j}"], "--", label=f"de{j}", linewidth=1.8)
    axes[i].set_ylabel(f"Joint {j} Error")
    axes[i].grid(True, alpha=0.3)
    axes[i].legend(loc="best")

axes[-1].set_xlabel("Time (s)")
plt.tight_layout(rect=[0, 0, 1, 0.96])

# =========================
# 12. Show all
# =========================
plt.show()
