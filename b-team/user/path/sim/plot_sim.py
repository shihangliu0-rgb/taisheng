#!/usr/bin/env python3
"""绘制全路线闭环仿真轨迹（数据与 path_map.c 一致）。"""
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib import font_manager

for name in ("Noto Sans CJK SC", "WenQuanYi Zen Hei", "Microsoft YaHei",
             "PingFang SC", "SimHei"):
    if any(name == f.name for f in font_manager.fontManager.ttflist):
        plt.rcParams["font.family"] = name
        break
plt.rcParams["axes.unicode_minus"] = False

WALLS_NORMAL = [
    (0.000, 0.000, 3.000, 0.049), (0.000, 5.951, 3.000, 6.000),
    (0.000, 0.000, 0.049, 6.000), (2.951, 0.000, 3.000, 6.000),
    (1.050, 1.070, 3.000, 1.120), (0.000, 2.075, 2.000, 2.125),
    (1.050, 3.075, 3.000, 3.125),
]
WALLS_MIRRORED = [
    (0.000, 0.000, 3.000, 0.049), (0.000, 5.951, 3.000, 6.000),
    (0.000, 1.070, 0.049, 6.000), (2.951, 0.000, 3.000, 6.000),
    (0.000, 1.070, 1.950, 1.120), (1.000, 2.075, 3.000, 2.125),
    (0.000, 3.075, 1.950, 3.125),
]
ROBOT_W, ROBOT_L = 0.440, 0.617


def load(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append(row)
    return rows


def plot(csv_path, mirrored, title, out_path):
    rows = load(csv_path)
    walls = WALLS_MIRRORED if mirrored else WALLS_NORMAL
    fig, ax = plt.subplots(figsize=(6, 10))
    ax.add_patch(Rectangle((0, 0), 3, 6, fill=False, lw=1.2, ec="k"))
    for x0, y0, x1, y1 in walls:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               fc="#555555", ec="k", lw=0.5))
    fx = [float(r["true_x"]) for r in rows]
    fy = [float(r["true_y"]) for r in rows]
    ax.plot(fx, fy, color="tab:green", lw=1.6, label="Outbound (true)")
    mx = [float(r["map_x"]) for r in rows]
    my = [float(r["map_y"]) for r in rows]
    ax.plot(mx, my, color="tab:blue", lw=0.7, ls="--", alpha=0.6,
            label="path.c map coord")
    x0, y0 = float(rows[0]["true_x"]), float(rows[0]["true_y"])
    ax.add_patch(Rectangle((x0 - ROBOT_W / 2, y0 - ROBOT_L / 2),
                           ROBOT_W, ROBOT_L, fill=False, ec="tab:green",
                           ls=":", lw=1.0))
    xe, ye = float(rows[-1]["true_x"]), float(rows[-1]["true_y"])
    ax.add_patch(Rectangle((xe - ROBOT_W / 2, ye - ROBOT_L / 2),
                           ROBOT_W, ROBOT_L, fill=False, ec="tab:red",
                           lw=1.2))
    ax.plot([x0], [y0], "o", color="tab:green", ms=6, label="Start")
    ax.plot([xe], [ye], "s", color="tab:red", ms=6, label="Final pose")
    ax.set_xlim(-0.15, 3.15)
    ax.set_ylim(-0.15, 6.15)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print("saved", out_path)


if __name__ == "__main__":
    out = Path(sys.argv[1] if len(sys.argv) > 1 else "user/path/sim/results")
    plot(out / "normal_manual.csv", False,
         "Normal side / manual outbound", out / "traj_normal_manual.png")
    plot(out / "normal_auto.csv", False,
         "Normal side / fully autonomous (5s auto start)",
         out / "traj_normal_auto.png")
    plot(out / "mirrored_manual.csv", True,
         "Mirrored side / manual outbound (careful)",
         out / "traj_mirrored_manual.png")
    plot(out / "mirrored_auto.csv", True,
         "Mirrored side / fully autonomous (5s auto start)",
         out / "traj_mirrored_auto.png")
