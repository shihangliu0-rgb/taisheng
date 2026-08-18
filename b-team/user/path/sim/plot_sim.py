#!/usr/bin/env python3
"""绘制全路线闭环仿真轨迹：真值 vs 真实融合里程计地图坐标。"""
import csv
import json
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
ROUTE_NORMAL = [(0.374, 0.3085), (0.374, 1.600), (2.480, 1.600),
                (2.480, 2.600), (0.360, 2.600), (0.360, 3.700),
                (0.500, 3.700)]
ROUTE_MIRRORED = [(2.626, 0.3085), (2.626, 1.600), (0.520, 1.600),
                  (0.520, 2.600), (2.640, 2.600), (2.640, 3.700),
                  (2.500, 3.700)]
ROBOT_W, ROBOT_L = 0.440, 0.617


def load(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append(row)
    return rows


def fget(row, key, default=0.0):
    try:
        return float(row[key])
    except (KeyError, ValueError, TypeError):
        return default


def plot(csv_path, mirrored, title, out_path):
    rows = load(csv_path)
    walls = WALLS_MIRRORED if mirrored else WALLS_NORMAL
    route = ROUTE_MIRRORED if mirrored else ROUTE_NORMAL
    fig, axes = plt.subplots(1, 2, figsize=(11, 9),
                             gridspec_kw={"width_ratios": [1.15, 1]})
    ax = axes[0]
    ax.add_patch(Rectangle((0, 0), 3, 6, fill=False, lw=1.2, ec="k"))
    for x0, y0, x1, y1 in walls:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               fc="#555555", ec="k", lw=0.5))
    rx = [p[0] for p in route]
    ry = [p[1] for p in route]
    ax.plot(rx, ry, color="#bbbbbb", lw=1.0, ls=":", label="Nominal route")
    fx = [fget(r, "true_x") for r in rows]
    fy = [fget(r, "true_y") for r in rows]
    ax.plot(fx, fy, color="tab:green", lw=1.8, label="True pose")
    # 锚定前 map 为 0；到点交接后融合停跑，这两段都不画进控制轨迹。
    live = [r for r in rows
            if int(fget(r, "imu_ok")) == 1 and fget(r, "map_x") != 0.0]
    if any(int(fget(r, "route_complete")) for r in live):
        live = [r for r in live if int(fget(r, "route_complete")) == 0] + [
            next(r for r in live if int(fget(r, "route_complete")) == 1)
        ]
    mx = [fget(r, "map_x") for r in live]
    my = [fget(r, "map_y") for r in live]
    if mx:
        ax.plot(mx, my, color="tab:blue", lw=0.9, ls="--", alpha=0.75,
                label="Fused map coord")
    x0, y0 = fx[0], fy[0]
    ax.add_patch(Rectangle((x0 - ROBOT_W / 2, y0 - ROBOT_L / 2),
                           ROBOT_W, ROBOT_L, fill=False, ec="tab:green",
                           ls=":", lw=1.0))
    xe, ye = fx[-1], fy[-1]
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

    ax2 = axes[1]
    err_rows = [r for r in rows
                if int(fget(r, "imu_ok")) == 1 and
                int(fget(r, "route_complete")) == 0]
    t = [fget(r, "t_ms") / 1000.0 for r in err_rows]
    err = [fget(r, "odom_err") * 1000.0 for r in err_rows]
    ax2.plot(t, err, color="tab:orange", lw=1.2, label="|map - true|")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Odometry error (mm)")
    ax2.set_title("Real fused odometry vs truth")
    ax2.grid(True, alpha=0.25)
    ax2.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print("saved", out_path)


def downsample(rows, step=8):
    out = []
    for i, row in enumerate(rows):
        if i % step == 0 or i == len(rows) - 1:
            out.append({
                "t": round(fget(row, "t_ms") / 1000.0, 3),
                "x": round(fget(row, "true_x"), 4),
                "y": round(fget(row, "true_y"), 4),
                "yaw": round(fget(row, "yaw_deg"), 2),
                "mx": round(fget(row, "map_x"), 4),
                "my": round(fget(row, "map_y"), 4),
                "seg": int(fget(row, "seg")),
                "auto": int(fget(row, "auto_state")),
                "front": int(fget(row, "front_cm")),
                "left": int(fget(row, "left_cm")),
                "err": round(fget(row, "odom_err") * 1000.0, 1),
            })
    return out


def write_html(out_dir, scenarios):
    html = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8"/>
<title>泰升路径闭环仿真</title>
<style>
body{font-family:system-ui,sans-serif;margin:16px;background:#111;color:#eee}
h1{font-size:20px;margin:0 0 8px}
.bar{display:flex;gap:8px;flex-wrap:wrap;margin:8px 0 12px}
button{background:#2a2a2a;color:#eee;border:1px solid #555;border-radius:6px;
padding:6px 10px;cursor:pointer}
button.active{background:#1b6;color:#111;font-weight:700}
#stage{display:flex;gap:16px;align-items:flex-start;flex-wrap:wrap}
canvas{background:#1a1a1a;border:1px solid #333;border-radius:8px}
#hud{font-family:ui-monospace,monospace;font-size:13px;line-height:1.55;
min-width:260px}
.ok{color:#6d6}.bad{color:#f66}
</style></head><body>
<h1>真实融合里程计 · 全路线闭环仿真</h1>
<p>绿框=真值机器人，蓝虚线=path.c 用地图坐标（来自真实 PathLineImu，不是真值回灌）。</p>
<div class="bar" id="tabs"></div>
<div class="bar">
<button id="play">播放 / 暂停</button>
<button id="reset">重播</button>
</div>
<div id="stage"><canvas id="cv" width="420" height="820"></canvas>
<div id="hud"></div></div>
<script>
const DATA = __DATA__;
const WALLS = {
  normal:[[0,0,3,0.049],[0,5.951,3,6],[0,0,0.049,6],[2.951,0,3,6],
         [1.05,1.07,3,1.12],[0,2.075,2,2.125],[1.05,3.075,3,3.125]],
  mirrored:[[0,0,3,0.049],[0,5.951,3,6],[0,1.07,0.049,6],[2.951,0,3,6],
            [0,1.07,1.95,1.12],[1,2.075,3,2.125],[0,3.075,1.95,3.125]]
};
const RW=0.440, RL=0.617;
const cv=document.getElementById('cv'), ctx=cv.getContext('2d');
const hud=document.getElementById('hud');
let key=Object.keys(DATA)[0], idx=0, playing=true, last=0;
function sx(x){return 30+x*120;}
function sy(y){return 790-y*120;}
function draw(){
  const sc=DATA[key], rows=sc.rows, r=rows[Math.min(idx,rows.length-1)];
  ctx.clearRect(0,0,cv.width,cv.height);
  ctx.strokeStyle='#888'; ctx.strokeRect(sx(0),sy(6),360,720);
  ctx.fillStyle='#555';
  for (const w of WALLS[sc.mirrored?'mirrored':'normal']){
    ctx.fillRect(sx(w[0]), sy(w[3]), (w[2]-w[0])*120, (w[3]-w[1])*120);
  }
  ctx.beginPath();
  rows.forEach((p,i)=>{const X=sx(p.x),Y=sy(p.y); i?ctx.lineTo(X,Y):ctx.moveTo(X,Y);});
  ctx.strokeStyle='#3c3'; ctx.lineWidth=2; ctx.stroke();
  ctx.beginPath();
  rows.forEach((p,i)=>{const X=sx(p.mx),Y=sy(p.my); i?ctx.lineTo(X,Y):ctx.moveTo(X,Y);});
  ctx.strokeStyle='#6af'; ctx.setLineDash([5,4]); ctx.lineWidth=1.2; ctx.stroke();
  ctx.setLineDash([]);
  ctx.save();
  ctx.translate(sx(r.x), sy(r.y));
  ctx.rotate(-r.yaw*Math.PI/180);
  ctx.strokeStyle='#8f8'; ctx.strokeRect(-RW*60, -RL*60, RW*120, RL*120);
  ctx.beginPath(); ctx.moveTo(0, RL*40); ctx.lineTo(0, -RL*55);
  ctx.stroke();
  ctx.restore();
  hud.innerHTML = `<b>${sc.title}</b><br>
t = ${r.t.toFixed(2)} s<br>
true = (${r.x.toFixed(3)}, ${r.y.toFixed(3)}) yaw=${r.yaw.toFixed(1)}°<br>
map  = (${r.mx.toFixed(3)}, ${r.my.toFixed(3)})<br>
seg=${r.seg} auto=${r.auto}<br>
DT35 front=${r.front} cm  left=${r.left} cm<br>
odom error = <span class="${r.err>30?'bad':'ok'}">${r.err.toFixed(1)} mm</span>`;
}
function tick(ts){
  if (playing && ts-last>40){
    last=ts; idx=Math.min(idx+1, DATA[key].rows.length-1);
    if (idx===DATA[key].rows.length-1) playing=false;
    draw();
  }
  requestAnimationFrame(tick);
}
const tabs=document.getElementById('tabs');
Object.keys(DATA).forEach((k,i)=>{
  const b=document.createElement('button');
  b.textContent=DATA[k].title;
  if(i===0) b.classList.add('active');
  b.onclick=()=>{key=k; idx=0; playing=true;
    [...tabs.children].forEach(x=>x.classList.remove('active'));
    b.classList.add('active'); draw();};
  tabs.appendChild(b);
});
document.getElementById('play').onclick=()=>playing=!playing;
document.getElementById('reset').onclick=()=>{idx=0; playing=true; draw();};
draw(); requestAnimationFrame(tick);
</script></body></html>
"""
    payload = {}
    for name, meta in scenarios.items():
        rows = load(meta["csv"])
        payload[name] = {
            "title": meta["title"],
            "mirrored": meta["mirrored"],
            "rows": downsample(rows),
        }
    page = html.replace("__DATA__", json.dumps(payload, ensure_ascii=False))
    path = out_dir / "view_sim.html"
    path.write_text(page, encoding="utf-8")
    print("saved", path)


if __name__ == "__main__":
    out = Path(sys.argv[1] if len(sys.argv) > 1 else "user/path/sim/results")
    scenarios = {
        "normal_manual": {
            "csv": out / "normal_manual.csv",
            "mirrored": False,
            "title": "常规侧 / 手动去程",
        },
        "normal_auto": {
            "csv": out / "normal_auto.csv",
            "mirrored": False,
            "title": "常规侧 / 全自动",
        },
        "mirrored_manual": {
            "csv": out / "mirrored_manual.csv",
            "mirrored": True,
            "title": "镜像侧 / 手动谨慎",
        },
        "mirrored_auto": {
            "csv": out / "mirrored_auto.csv",
            "mirrored": True,
            "title": "镜像侧 / 全自动",
        },
    }
    plot(out / "normal_manual.csv", False,
         "Normal / manual outbound (real fused odom)",
         out / "traj_normal_manual.png")
    plot(out / "normal_auto.csv", False,
         "Normal / fully autonomous (real fused odom)",
         out / "traj_normal_auto.png")
    plot(out / "mirrored_manual.csv", True,
         "Mirrored / manual outbound (real fused odom)",
         out / "traj_mirrored_manual.png")
    plot(out / "mirrored_auto.csv", True,
         "Mirrored / fully autonomous (real fused odom)",
         out / "traj_mirrored_auto.png")
    write_html(out, scenarios)
