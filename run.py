import pandas as pd
import matplotlib.pyplot as plt

# ===== 只需要改这里 =====
timestamp = "20260419_110610"

# ===== 固定文件名 + 时间辍 =====
file_paths = [
    f"./build/logs/joint_velocity_{timestamp}.txt",
    f"./build/logs/joint_position_{timestamp}.txt",
    f"./build/logs/joint_torque_{timestamp}.txt",
]

channel_names = {
    0: "Joint 1",
    1: "Joint 2",
    2: "Joint 3",
}

def read_realtime_file(filepath):
    rows = []

    with open(filepath, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()

            if not line or line.startswith("#"):
                continue

            parts = [x.strip() for x in line.split(",")]
            if len(parts) != 3:
                continue

            try:
                t = float(parts[0])
                ch = int(parts[1])
                val = float(parts[2])
                rows.append([t, ch, val])
            except ValueError:
                continue

    return pd.DataFrame(rows, columns=["time", "channel", "value"])

def plot_files(file_paths):
    fig, axes = plt.subplots(len(file_paths), 1, figsize=(10, 12))

    if len(file_paths) == 1:
        axes = [axes]

    for ax, filepath in zip(axes, file_paths):
        df = read_realtime_file(filepath)

        for ch in sorted(df["channel"].unique()):
            ch_data = df[df["channel"] == ch].sort_values("time")
            ax.plot(
                ch_data["time"],
                ch_data["value"],
                marker="o",
                markersize=3,
                linewidth=1,
                label=channel_names.get(ch, f"Channel {ch}")
            )

        ax.set_title(filepath)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Value")
        ax.grid(True)
        ax.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_files(file_paths)
