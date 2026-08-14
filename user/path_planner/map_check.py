# -*- coding: utf-8 -*-
"""
map_check.py —— 场地地图 + 路径点可视化检查工具(仅 PC 运行,不参与 Keil 编译)

用途:
  把 user/path_planner/path_config.h 里的真实场地(墙表)与路径点画出来,
  并标记用户给出的关键点,用于肉眼核对场地几何是否有问题。

数据来源(与 C 代码同一份配置,永远同步):
  - PATH_WALLS_TABLE      真实墙体
  - PATH_WAYPOINTS_TABLE  路径点(含注释标签)
  - 机器人尺寸 / 膨胀余量 / 终点与到达容差

输出:
  1) map_check.png         地图(真实场地 + yaml 旧地图对比)
  2) 控制台文字报告       每个路径点到墙的距离、线段是否穿墙、关键净距

运行:
  pip install matplotlib
  python3 map_check.py
"""
import math
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent
CFG = HERE / "path_config.h"
OUT = HERE / "map_check.png"

# 旧 field.yaml 反推的场地(仅用于对比,注意真实场地是 3m 宽)
YAML_WALLS = [
    (0.000, 0.000, 11.000, 0.049),   # south
    (0.000, 5.951, 11.000, 6.000),   # north
    (0.000, 0.000, 0.049, 6.000),    # west
    (10.951, 0.000, 11.000, 6.000),  # east
    (0.000, 2.075, 2.300, 2.125),    # wall_B
    (0.700, 3.075, 3.000, 3.125),    # wall_C
]
YAML_WAYPOINTS = [
    (0.50, 1.00), (1.00, 1.65), (2.00, 1.65), (2.50, 2.10),
    (2.50, 2.60), (1.00, 2.60), (0.50, 3.70),
]

# ---------------------------------------------------------------- 解析 config
def macro_block(text, name):
    """提取 #define NAME 的完整宏体(含续行),返回字符串。"""
    lines = text.splitlines()
    for idx, ln in enumerate(lines):
        m = re.match(r"#define\s+" + name + r"\b\s*(.*)$", ln)
        if not m:
            continue
        body = m.group(1)
        while body.rstrip().endswith("\\"):
            idx += 1
            body = body.rstrip()[:-1] + "\n" + lines[idx]
        return body
    raise KeyError(name)

def parse_rects(block):
    out = []
    for m in re.finditer(
        r"\{([\d.eE+-]+)f?\s*,\s*([\d.eE+-]+)f?\s*,\s*"
        r"([\d.eE+-]+)f?\s*,\s*([\d.eE+-]+)f?\s*\}\s*,?\s*(/\*.*?\*/)?",
        block, re.S):
        label = (m.group(5) or "").strip("/* ").strip().replace("\n", " ")
        out.append((float(m.group(1)), float(m.group(2)),
                    float(m.group(3)), float(m.group(4)), label))
    return out

def parse_pairs(block):
    out = []
    for m in re.finditer(
        r"\{([\d.eE+-]+)f?\s*,\s*([\d.eE+-]+)f?\s*\}\s*,?\s*(/\*.*?\*/)?",
        block, re.S):
        label = (m.group(3) or "").strip("/* ").strip().replace("\n", " ")
        out.append((float(m.group(1)), float(m.group(2)), label))
    return out

def parse_float(text, name):
    m = re.search(r"#define\s+" + name + r"\s+([\d.eE+-]+)f?", text)
    return float(m.group(1))

cfg = CFG.read_text(encoding="gbk")
WALLS = parse_rects(macro_block(cfg, "PATH_WALLS_TABLE"))
WP    = parse_pairs(macro_block(cfg, "PATH_WAYPOINTS_TABLE"))
ROBOT_L = parse_float(cfg, "PATH_ROBOT_LENGTH_M")
ROBOT_W = parse_float(cfg, "PATH_ROBOT_WIDTH_M")
SAFE_M  = parse_float(cfg, "PATH_SAFETY_MARGIN_M")
HARD_M  = parse_float(cfg, "PATH_HARD_MARGIN_M")
GOAL    = (parse_float(cfg, "PATH_GOAL_X_M"), parse_float(cfg, "PATH_GOAL_Y_M"))
TOL     = parse_float(cfg, "PATH_ARRIVE_TOL_M")

# 软/硬膨胀量(轴对齐矩形,yaw 锁定)
SOFT = (ROBOT_W / 2 + SAFE_M, ROBOT_L / 2 + SAFE_M)
HARD = (ROBOT_W / 2 + HARD_M, ROBOT_L / 2 + HARD_M)

# ---------------------------------------------------------------- 几何工具
def rect_dist(w, x, y):
    dx = (w[0] - x) if x < w[0] else ((x - w[2]) if x > w[2] else 0.0)
    dy = (w[1] - y) if y < w[1] else ((y - w[3]) if y > w[3] else 0.0)
    return math.hypot(dx, dy)

def wall_dist(walls, x, y):
    return min(rect_dist(w, x, y) for w in walls)

def inside(walls, x, y, eps=0.0):
    return any(w[0] + eps < x < w[2] - eps and w[1] + eps < y < w[3] - eps
               for w in walls)

def inflate_walls(walls, dx, dy):
    return [(w[0] - dx, w[1] - dy, w[2] + dx, w[3] + dy) for w in walls]

REAL = [(a, b, c, d) for a, b, c, d, _ in WALLS]
SOFT_WALLS = inflate_walls(REAL, *SOFT)
HARD_WALLS = inflate_walls(REAL, *HARD)

# ---------------------------------------------------------------- 文字报告
print("=" * 72)
print("map_check 报告(数据来自 path_config.h,与 C 代码同一份)")
print("=" * 72)
print(f"场地: 宽 3.0 m x 高 6.0 m  机器人: {ROBOT_L:.3f} x {ROBOT_W:.3f} m")
print(f"软膨胀(整形): 半宽 {SOFT[0]:.3f} / 半长 {SOFT[1]:.3f} m")
print(f"硬膨胀(验收): 半宽 {HARD[0]:.3f} / 半长 {HARD[1]:.3f} m")
print(f"终点: {GOAL}  到达容差: {TOL} m")
print("-" * 72)
print(f"{'#':>2}  {'x':>6}  {'y':>6}  距真实墙(cm)  软墙内?  硬墙内?  标签")
bad = []
for i, (x, y, label) in enumerate(WP):
    d = wall_dist(REAL, x, y)
    si = "是!" if inside(SOFT_WALLS, x, y) else "-"
    hi = "是!" if inside(HARD_WALLS, x, y) else "-"
    if si != "-" or hi != "-":
        bad.append((i, x, y))
    print(f"{i:>2}  {x:6.2f}  {y:6.2f}  {d * 100:9.1f}    {si:>4}    {hi:>4}   {label}")

print("-" * 72)
mid_bad = []
for i in range(1, len(WP)):
    mx = (WP[i - 1][0] + WP[i][0]) / 2
    my = (WP[i - 1][1] + WP[i][1]) / 2
    if inside(HARD_WALLS, mx, my):
        mid_bad.append((i - 1, i, mx, my))
print(f"路径点落在软/硬膨胀墙内: {len(bad)} 个", ("-> " + str(bad)) if bad else "(无,OK)")
print(f"相邻点弦线中点穿硬膨胀墙: {len(mid_bad)} 处",
      ("-> " + str(mid_bad)) if mid_bad else "(无,OK)")

print("-" * 72)
print("关键净距(手动核对项):")
d_apex = wall_dist(REAL, 2.48, 2.13)
print(f"  D 角圆弧顶点 (2.48,2.13) 到东墙: {d_apex * 100:.1f} cm"
      f"(车体余量 {(d_apex - ROBOT_W / 2) * 100:.1f} cm)")
d_gap_w = wall_dist(REAL, 0.36, 3.0)
print(f"  墙C缺口列 x=0.36 到西墙: {d_gap_w * 100:.1f} cm"
      f"(车体余量 {(d_gap_w - ROBOT_W / 2) * 100:.1f} cm)")
# 墙C 是 xmin > 0.36 的墙里 xmin 最小的那堵
wc = min((w for w in REAL if w[0] > 0.36), key=lambda w: w[0])
d_gap_c = rect_dist(wc, 0.36, 3.10)   # y=3.10 落在墙C y 范围内
print(f"  墙C缺口列 x=0.36 到墙C(xmin={wc[0]:.2f}): {d_gap_c * 100:.1f} cm"
      f"(车体余量 {(d_gap_c - ROBOT_W / 2) * 100:.1f} cm)")
# 按 y 范围定位内墙:墙B(y≈2.075-2.125)、墙A(y≈1.35-1.40)
wb = min((w for w in REAL if 2.0 < w[1] < 2.2), key=lambda w: w[1])
wa = min((w for w in REAL if 1.0 < w[1] < 2.0), key=lambda w: w[1])
d_b = rect_dist(wb, 1.0, 2.60)
print(f"  通道2 (1.0,2.60) 到墙B(ymin={wb[1]:.2f}): {d_b * 100:.1f} cm"
      f"(车体余量 {(d_b - ROBOT_L / 2) * 100:.1f} cm)")
d_b1 = rect_dist(wb, 2.0, 1.65)
print(f"  通道1 (2.0,1.65) 到墙B(ymin={wb[1]:.2f}): {d_b1 * 100:.1f} cm"
      f"(车体余量 {(d_b1 - ROBOT_L / 2) * 100:.1f} cm)")
# 墙A(通道墙1,最南侧内墙):通道1 中心线 y=1.65 对其南侧净距
d_a = rect_dist(wa, 2.0, 1.65)
print(f"  通道1 (2.0,1.65) 到墙1(墙A, ymax={wa[3]:.2f}): {d_a * 100:.1f} cm"
      f"(车体余量 {(d_a - ROBOT_L / 2) * 100:.1f} cm"
      + (" — 按 yaml 坐标通道1 走不通!需实测墙1 真实位置" if d_a < ROBOT_L / 2 else ")"))
d_goal = math.hypot(WP[-1][0] - GOAL[0], WP[-1][1] - GOAL[1])
print(f"  末路点 {WP[-1][0]:.2f},{WP[-1][1]:.2f} 到终点 {GOAL}: {d_goal * 100:.1f} cm"
      f"(到达容差 {TOL * 100:.0f} cm,{'OK' if d_goal <= TOL else '超容差!'})")
print("=" * 72)
print("注意: 墙1(墙A)、墙B东端 x=2.0 与墙C东端延伸到东墙均为反推值,需实车量测确认。")

# ---------------------------------------------------------------- 画图
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mp

fig = plt.figure(figsize=(9.5, 13.5))
gs = fig.add_gridspec(2, 1, height_ratios=[1.0, 0.42], hspace=0.32)

def draw_map(ax, walls, title, xmax):
    for w in walls:
        ax.add_patch(mp.Rectangle((w[0], w[1]), w[2] - w[0], w[3] - w[1],
                                  fc="#9a9a9a", ec="k", lw=0.8, zorder=1))
    for w in SOFT_WALLS:
        ax.add_patch(mp.Rectangle((w[0], w[1]), w[2] - w[0], w[3] - w[1],
                                  fc="none", ec="red", lw=0.8, ls="--", zorder=2))
    for w in HARD_WALLS:
        ax.add_patch(mp.Rectangle((w[0], w[1]), w[2] - w[0], w[3] - w[1],
                                  fc="none", ec="orange", lw=0.8, ls=":", zorder=2))
    ax.set_xlim(-0.35, xmax + 0.35)
    ax.set_ylim(-0.35, 6.35)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=11)
    ax.grid(alpha=0.25)

# ---- 主图:真实场地(3m) ----
ax = fig.add_subplot(gs[0])
draw_map(ax, REAL, "Real field (3.0m wide, east wall at x=3.0) - walls from path_config.h", 3.0)
ax.plot([p[0] for p in WP], [p[1] for p in WP], "b-", lw=1.4, zorder=3, label="waypoint polyline")
for i, (x, y, _) in enumerate(WP):
    ax.plot(x, y, "bo", ms=4, zorder=4)
    ax.annotate(str(i), (x, y), textcoords="offset points", xytext=(4, 4), fontsize=8, color="b")
# 用户给的点
ax.plot(2.00, 1.65, "mX", ms=14, mew=2.5, zorder=6)
ax.plot(2.00, 2.60, "mX", ms=14, mew=2.5, zorder=6)
ax.annotate("D arc start (2.0,1.65) - USER", (2.00, 1.65), textcoords="offset points",
            xytext=(-8, -18), fontsize=9, color="m", fontweight="bold")
ax.annotate("D arc end (2.0,2.6) - USER", (2.00, 2.60), textcoords="offset points",
            xytext=(-8, 8), fontsize=9, color="m", fontweight="bold")
# D 角参考半圆
th = [a * math.pi / 180 for a in range(-90, 91, 5)]
ax.plot([2.0 + 0.475 * math.cos(t) for t in th],
        [2.125 + 0.475 * math.sin(t) for t in th], "c--", lw=1.2, zorder=3,
        label="D semicircle ref (c=(2.0,2.125), R=0.475)")
# yaml 原始 7 点(淡灰,对照)
ax.plot([p[0] for p in YAML_WAYPOINTS], [p[1] for p in YAML_WAYPOINTS],
        "k--", lw=0.9, alpha=0.45, zorder=3, label="yaml original 7 waypoints")
ax.plot([p[0] for p in YAML_WAYPOINTS], [p[1] for p in YAML_WAYPOINTS],
        "k.", ms=4, alpha=0.5, zorder=4)
# 起点 / 终点
ax.plot(0.50, 1.00, "g*", ms=16, zorder=6)
ax.annotate("nominal start (0.5,1.0)\n(overwritten by measured pose)",
            (0.50, 1.00), textcoords="offset points", xytext=(8, -14), fontsize=8, color="g")
ax.plot(GOAL[0], GOAL[1], "r*", ms=18, zorder=6)
ax.annotate(f"goal {GOAL} (tol {TOL:.2f}m)", (GOAL[0], GOAL[1]),
            textcoords="offset points", xytext=(8, 6), fontsize=9, color="r", fontweight="bold")
ax.add_patch(mp.Circle(GOAL, TOL, fc="none", ec="r", lw=0.9, ls="--", zorder=3))
# 机器人在终点位置的轮廓(轴对齐)
ax.add_patch(mp.Rectangle((GOAL[0] - ROBOT_W / 2, GOAL[1] - ROBOT_L / 2),
                          ROBOT_W, ROBOT_L, fc="none", ec="k", lw=1.1, ls="-", zorder=5))
# 关键标注
ax.annotate("east wall x=3.0", (2.85, 3.1), rotation=90, fontsize=9, color="brown")
ax.annotate("wall C gap: x<0.7\n(path column x=0.36)", (0.42, 2.95),
            fontsize=8, color="darkred")
ax.annotate("wall B east end x=2.0\n(inferred, verify)", (1.1, 2.10), fontsize=8, color="darkred")
ax.legend(loc="lower left", fontsize=8, ncol=1)
ax.set_xlabel("x (m), origin = bottom-left corner")
ax.set_ylabel("y (m)")

# ---- 对照图:旧 field.yaml(11m 场地) ----
ax2 = fig.add_subplot(gs[1])
draw_map(ax2, YAML_WALLS, "OLD field.yaml map (11m wide) - for comparison only", 11.0)
ax2.plot([p[0] for p in YAML_WAYPOINTS], [p[1] for p in YAML_WAYPOINTS],
         "b-", lw=1.2)
for i, (x, y) in enumerate(YAML_WAYPOINTS):
    ax2.plot(x, y, "bo", ms=4)
    ax2.annotate(str(i), (x, y), textcoords="offset points", xytext=(4, 4), fontsize=8, color="b")
ax2.annotate("this was the 11m assumption\n(real field is 3m wide!)",
             (5.5, 3.0), fontsize=10, color="red", ha="center", fontweight="bold")
ax2.set_xlabel("x (m)")

fig.savefig(OUT, dpi=120, bbox_inches="tight")
print(f"\n地图已保存: {OUT}")
