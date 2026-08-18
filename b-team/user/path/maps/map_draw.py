#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成机器人对抗场地地图（常规侧 + 镜像侧）。

数据与 user/path/path_map.c / path_map.h 完全一致（镜像地图提交后版本）。

用法：
    pip install matplotlib
    python map_draw.py                      # 输出 map_normal.png 和 map_mirrored.png
    python map_draw.py --side normal        # 只画常规侧
    python map_draw.py --side mirrored      # 只画镜像侧
    python map_draw.py --outdir out/        # 指定输出目录
    python map_draw.py --dpi 200            # 输出分辨率

图例：
    灰色细矩形   = 4 面边界墙（高 4.9 cm）
    深灰矩形     = 3 段内墙（墙 1 / 墙 B / 墙 C）
    红色折线     = 人工单轴路线（带分段编号和终点）
    绿色方块     = 机器人起始位姿（车头朝 +Y）
"""

import argparse
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import fontManager
from matplotlib.patches import Rectangle

# ---------------------------------------------------------------- 场地常量
FIELD_W = 3.0
FIELD_H = 6.0

# 4 面边界墙（常规侧与镜像侧共用）：(x_min, y_min, x_max, y_max)
BOUNDARY_WALLS = [
    (0.000, 0.000, 3.000, 0.049),   # 南
    (0.000, 5.951, 3.000, 6.000),   # 北
    (0.000, 0.000, 0.049, 6.000),   # 西
    (2.951, 0.000, 3.000, 6.000),   # 东
]

# 3 段内墙：常规侧 vs 镜像侧（x' = 3 - x）
INNER_WALLS = {
    "normal": [
        (1.050, 1.070, 3.000, 1.120),   # 墙 1
        (0.000, 2.075, 2.000, 2.125),   # 墙 B
        (1.050, 3.075, 3.000, 3.125),   # 墙 C
    ],
    "mirrored": [
        (0.000, 1.070, 1.950, 1.120),   # 墙 1（贴西墙）
        (1.000, 2.075, 3.000, 2.125),   # 墙 B（贴东墙）
        (0.000, 3.075, 1.950, 3.125),   # 墙 C（贴西墙）
    ],
}
# 路线分段：(axis, direction, target)；direction +1 沿正轴，-1 沿负轴
ROUTES = {
    "normal": [
        ("Y", 1, 1.650),
        ("X", 1, 2.480),
        ("Y", 1, 2.600),
        ("X", -1, 0.360),
        ("Y", 1, 3.700),
        ("X", 1, 0.500),
    ],
    "mirrored": [
        ("Y", 1, 1.650),
        ("X", -1, 0.520),
        ("Y", 1, 2.600),
        ("X", 1, 2.640),
        ("Y", 1, 3.700),
        ("X", -1, 2.500),
    ],
}

# 回程路线（车头朝 -Y，掉头后从去程终点返回起点）
RETURN_ROUTES = {
    "normal": [
        ("X", -1, 0.360),
        ("Y", -1, 2.600),
        ("X", 1, 2.480),
        ("Y", -1, 1.650),
        ("X", -1, 0.374),
        ("Y", -1, 0.3085),
    ],
    "mirrored": [
        ("X", 1, 2.640),
        ("Y", -1, 2.600),
        ("X", -1, 0.520),
        ("Y", -1, 1.650),
        ("X", 1, 2.626),
        ("Y", -1, 0.3085),
    ],
}

# 机器人起始中心：(x, y)；常规侧贴西墙左光锚定，镜像侧假定贴东墙对称摆放
ROBOT_L = 0.617
ROBOT_W = 0.440
START = {
    "normal": (0.374, 0.3085),
    "mirrored": (2.626, 0.3085),
}

# ---------------------------------------------------------------- 中文字体
CJK_FONT_NAMES = [
    "Microsoft YaHei", "SimHei", "PingFang SC", "Hiragino Sans GB",
    "Noto Sans CJK SC", "Source Han Sans SC", "Noto Sans SC",
    "WenQuanYi Zen Hei", "Heiti SC",
]


def find_cjk_font():
    available = {f.name for f in fontManager.ttflist}
    for name in CJK_FONT_NAMES:
        if name in available:
            return name
    return None


def make_labels(has_cjk):
    if has_cjk:
        return {
            "title_normal": "常规侧场地地图（贴西墙起步）",
            "title_mirrored": "镜像侧场地地图（贴东墙起步）",
            "field_size": "场地 3.0 m × 6.0 m",
            "boundary": "边界墙（高 4.9 cm）",
            "inner": "内墙",
            "route": "人工单轴路线",
            "robot": "机器人起始位姿",
            "start": "起点",
            "yaw": "车头朝 +Y（yaw = 0°）",
            "note_normal": "左 DT35 扫描西墙定 X，y = 0.3085 m",
            "note_mirrored": "左 DT35 无目标；前光撞墙 B 自动识别并锚定",
            "wall_names": ["墙 1", "墙 B", "墙 C"],
        }
    return {
        "title_normal": "Normal side (start against west wall)",
        "title_mirrored": "Mirrored side (start against east wall)",
        "field_size": "Field 3.0 m x 6.0 m",
        "boundary": "Boundary walls (49 mm)",
        "inner": "Inner walls",
        "route": "Manual single-axis route",
        "robot": "Robot start pose",
        "start": "Start",
        "yaw": "Heading +Y (yaw = 0 deg)",
        "note_normal": "Left DT35 locks X to west wall, y = 0.3085 m",
        "note_mirrored": "No left target; auto-detect on front wall B hit",
        "wall_names": ["Wall 1", "Wall B", "Wall C"],
    }


# ---------------------------------------------------------------- 绘制
def route_polyline(route, start):
    """按分段生成折线顶点（起点 -> 各段终点）。"""
    points = [start]
    x, y = start
    for axis, _direction, target in route:
        if axis == "X":
            x = target
        else:
            y = target
        points.append((x, y))
    return points


def draw_side(ax, side, labels, show_note, returning=False):
    start = START[side]
    walls = INNER_WALLS[side]
    route = RETURN_ROUTES[side] if returning else ROUTES[side]
    if returning:
        # 回程从去程终点出发
        start = route_polyline(ROUTES[side], START[side])[-1]
    points = route_polyline(route, start)

    # 边界墙
    for (x0, y0, x1, y1) in BOUNDARY_WALLS:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               facecolor="#9aa0a6", edgecolor="#5f6368",
                               linewidth=0.8, zorder=2))
    # 内墙
    for (x0, y0, x1, y1), name in zip(walls, labels["wall_names"]):
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               facecolor="#5f6368", edgecolor="#202124",
                               linewidth=0.8, zorder=3))
        cx = 0.5 * (x0 + x1)
        cy = 0.5 * (y0 + y1)
        ax.text(cx, cy, name, ha="center", va="center", fontsize=11,
                color="white", zorder=5,
                bbox=dict(boxstyle="round,pad=0.15", fc="#5f6368", ec="none"))

    # 路线（红色折线 + 分段箭头）
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    ax.plot(xs, ys, color="#d93025", linewidth=2.2, zorder=4,
            label=labels["route"])
    for i in range(len(route)):
        (x0, y0), (x1, y1) = points[i], points[i + 1]
        ax.annotate("", xy=(x1, y1), xytext=(x0, y0),
                    arrowprops=dict(arrowstyle="-|>", color="#d93025",
                                    lw=2.2, mutation_scale=16),
                    zorder=4)
        mid = (0.5 * (x0 + x1), 0.5 * (y0 + y1))
        ax.plot(*mid, marker="o", markersize=6, color="white",
                markeredgecolor="#d93025", markeredgewidth=1.4, zorder=5)
        ax.text(mid[0], mid[1], str(i), ha="center", va="center",
                fontsize=8, color="#d93025", zorder=6,
                fontweight="bold")
        # 终点坐标
        ax.text(x1 + 0.06, y1 + 0.06,
                f"{x1:.2f},{y1:.2f}" if route[i][0] == "X"
                else f"{x1:.2f},{y1:.2f}",
                fontsize=8, color="#a71e21", zorder=6)

    # 机器人起始位姿
    rect = Rectangle((start[0] - 0.5 * ROBOT_W, start[1] - 0.5 * ROBOT_L),
                     ROBOT_W, ROBOT_L, facecolor="#34a853",
                     edgecolor="#0d652d", linewidth=1.4, zorder=6, alpha=0.9,
                     label=labels["robot"])
    ax.add_patch(rect)
    ax.annotate("", xy=(start[0], start[1] + 0.5 * ROBOT_L + 0.12),
                xytext=(start[0], start[1]),
                arrowprops=dict(arrowstyle="-|>", color="#0d652d", lw=2.0,
                                mutation_scale=18), zorder=7)
    ax.text(start[0], start[1] - 0.5 * ROBOT_L - 0.18, labels["start"],
            ha="center", va="top", fontsize=10, color="#0d652d", zorder=7,
            fontweight="bold")
    ax.text(start[0] + 0.16, start[1] + 0.5 * ROBOT_L + 0.22,
            f"({start[0]:.3f}, {start[1]:.3f})", ha="left", va="bottom",
            fontsize=9, color="#0d652d", zorder=7)

    # 注记
    if show_note:
        ax.text(0.02, 0.02, labels["note_normal" if side == "normal"
                else "note_mirrored"], transform=ax.transAxes,
                fontsize=9, color="#3c4043", va="bottom", ha="left",
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#dadce0"))

    ax.set_aspect("equal")
    ax.set_xlim(-0.35, FIELD_W + 0.35)
    ax.set_ylim(-0.55, FIELD_H + 0.55)
    ax.grid(True, linestyle=":", linewidth=0.5, color="#e0e0e0", zorder=0)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    title = labels["title_normal" if side == "normal"
                   else "title_mirrored"]
    if returning:
        title = ("回程路线（车头朝 -Y）· " if has_cjk_label(labels)
                 else "Return route (heading -Y) · ") + title
    ax.set_title(title + "   " + labels["field_size"],
                 fontsize=13, fontweight="bold")
    return [Rectangle((0, 0), 1, 1, facecolor="#9aa0a6", edgecolor="#5f6368",
                      label=labels["boundary"]),
            Rectangle((0, 0), 1, 1, facecolor="#5f6368", edgecolor="#202124",
                      label=labels["inner"]),
            Rectangle((0, 0), 1, 1, facecolor="#34a853", edgecolor="#0d652d",
                      label=labels["robot"])]


def has_cjk_label(labels):
    return "wall_names" in labels and labels["wall_names"][0] == "墙 1"


def main():
    parser = argparse.ArgumentParser(description="生成对抗场地地图 PNG")
    parser.add_argument("--side", choices=["normal", "mirrored", "both"],
                        default="both")
    parser.add_argument("--return", action="store_true", dest="returning",
                        help="画回程路线（车头朝 -Y 返回起点）")
    parser.add_argument("--outdir", default=".", help="输出目录（默认当前目录）")
    parser.add_argument("--dpi", type=int, default=150)
    args = parser.parse_args()

    font_name = find_cjk_font()
    if font_name:
        plt.rcParams["font.family"] = font_name
    plt.rcParams["axes.unicode_minus"] = False
    labels = make_labels(font_name is not None)
    if font_name:
        print(f"使用中文字体: {font_name}")
    else:
        print("未找到中文字体，标注使用英文（安装 Noto Sans CJK / 微软雅黑后自动中文）")

    os.makedirs(args.outdir, exist_ok=True)
    sides = ["normal", "mirrored"] if args.side == "both" else [args.side]
    for side in sides:
        fig, ax = plt.subplots(figsize=(7.2, 12.0))
        handles = draw_side(ax, side, labels, show_note=True,
                            returning=args.returning)
        ax.legend(handles=handles, loc="upper left", fontsize=9,
                  framealpha=0.9)
        fig.tight_layout()
        suffix = "_return" if args.returning else ""
        out_path = os.path.join(args.outdir, f"map_{side}{suffix}.png")
        fig.savefig(out_path, dpi=args.dpi)
        plt.close(fig)
        print(f"已生成: {out_path}")


if __name__ == "__main__":
    main()
