# 人工路径辅助模块 AI 对接与差异审计

> 面向后续接手本工程的 AI / 开发者。阅读代码前先看本文，再看
> [`README.md`](README.md) 与
> [`../imu/README.md`](../imu/README.md)。
>
> 当前实现分支：`arena/01a00460-taisheng`
>
> 本文对比的 B-team 分支：`origin/arena/01a00390-taisheng`
>
> B-team 固定对比提交：`fd99b528aca9e29adf061ea5468e78e9dab101df`

## 1. 一句话结论

当前代码**不是 B-team 自动路径规划器的移植版**。它只引用 B-team
`user/path_planner/path_config.h` 中的场地墙体几何，实际运行方式是：

- 没有小电脑，不接收小电脑绝对位姿或自动路径指令；
- LoRa 遥控器始终是平移指令来源；
- 本机按键 1 切换当前地图分段的单轴直线约束；
- 正常控制期间 yaw 始终锁定为 `0°`；
- 左 DT35 与实测 yaw 确定一次性初始 X，初始 Y 固定为半车长；前/左
  DT35 分别保留对应方向的动态安全限制；
- 初始定点成功后，由本地 IMU / 四轮 VESC 融合里程计连续更新地图位置；
- 静态地图墙只做辅助净空限制，不生成自动轨迹。

不要把 B-team 的 `user/path_planner/`、`user/pc_link/`、FSM、自动纯追踪或
小电脑定位直接接入当前分支，除非用户明确改变“全程人工、没有小电脑”的需求。

---

## 2. 审计范围与结论

### 2.1 为什么不能直接比较两个分支的全仓文件

两个工作分支的共同祖先是空工程基线：

```text
c2cd5364b419b39d38083d44fed2a3624975c429
```

当前分支随后用提交
`d363ebea0283cc218248343466f3fcae916971f3`（“导入 b-team 抬升分支完整工程基线”）
导入完整下位机工程；B-team 对比分支则在自己的历史中继续加入
`user/path_planner/` 和 `user/pc_link/`。因此，从共同祖先直接做全仓 diff 会把两边
各自导入的完整工程都算成路径功能改动，结论没有意义。

本文对“本次及前序路径实现是否改了无关文件”的审计基准是：

```bash
git diff d363ebe..HEAD
```

### 2.2 累计修改结果

从导入基线 `d363ebe` 到编写本文前的功能提交 `a162168`：

- 共改动 30 个文件；
- 23 个文件位于新增模块 `user/path/`、`user/path/imu/`；
- 其余 7 个是既有工程的必要调用点或既有说明；
- 总计 `3926 insertions(+), 1 deletion(-)`；唯一统计为 deletion 的内容，是
  Keil XML 中原 `IncludePath` 整行被“原内容 + 两个新增目录”替换，不是删除
  既有 include 目录，也没有删除既有源码逻辑。

**审计结论：没有发现与本地里程计、人工路径、安全控制或工程编译接入无关的
业务文件修改；没有删除、替换既有底盘、通信、遥控、IMU 功能。**
既有文件中的运行时代码改动均为追加式调用，逐项如下。

| 既有文件 | 净改动 | 修改目的 | 审计判断 |
| --- | ---: | --- | --- |
| `Core/Src/freertos.c` | `+6/-0` | include 两个新模块；初始化 `Path`/`PathLineImu`；每 1 ms 先更新里程计再运行路径安全层 | 必要周期入口 |
| `MDK-ARM/b-up.uvprojx` | `+61/-1` | 追加 include path，并把新 `.c/.h` 加入 Keil group | 必要构建接入；未移除旧路径/源文件 |
| `remote control/遥控器输入与下位机对接说明.md` | `+16/-0` | 追加按键 1、yaw=0、激光和 LoRa 超时说明 | 与实际遥控行为同步的文档修改 |
| `user/com_link/computer_link.c` | `+2/-0` | 调用 `Path_ReplaceNonRemoteCommand()`，防止旧速度协议绕过人工安全层；动作协议保持原样 | 无小电脑模式的必要旁路保护 |
| `user/com_link/lora_link.c` | `+3/-0` | 提交遥控命令/六键状态；LoRa 超时时通知路径层 | 必要遥控入口和掉线清理 |
| `user/imu/imu.c` | `+11/-0` | 在原帧头、长度、帧尾校验后，把 `0x01` 加速度帧转发给融合里程计 | 必要传感器数据入口；原 `0x02/0x03` 解析保留 |
| `user/imu/imu_main.c` | `+4/-0` | 在原 IMU 配置序列中追加开启 `0x01` 加速度主动上报 | 必要传感器配置；原命令序列保留 |

前序二维融合里程计是独立模块，位于 `user/path/imu/`；本次人工路径、安全、
地图和定位的新功能均位于用户指定的 `user/path/`。

### 2.3 原“双激光初始定点”提交的历史审计

提交 `a16216899b73dd73de1e40fe0d7d599f3d95168e` 共改动 11 个文件：

- 10 个在 `user/path/`：定位源码、地图、总入口、README 和宿主测试；
- 唯一不在 `user/path/` 的文件是 `MDK-ARM/b-up.uvprojx`，只为加入
  `path_localization.c/.h`；
- 没有修改 Core、通信、遥控、IMU 或底盘源码。

上述是 `a162168` 的历史审计名称；当前版本已按实车启动位置纠正为“左光定 X、
半车长固定 Y”，前光不再参与初始定位，详见第 6 节。

---

## 3. 当前实现与 B-team 的核心区别

| 项目 | 当前实现：`user/path/` | B-team：`user/path_planner/` |
| --- | --- | --- |
| 目标 | 人工驾驶辅助、单轴约束、动态安全、静态墙保护 | 上电后自动建轨迹、自动跟踪并到达目标 |
| 指令来源 | LoRa 人工摇杆；安全层只限轴/限速/制动 | 自动控制器生成底盘速度；自动运行时拥有底盘并屏蔽人工指令 |
| 小电脑 | 不使用；不依赖绝对位姿、感知帧或自动路径指令 | `user/pc_link/` 通过 UART7 接收 `0x11` 绝对位姿，并发送 `0x20` 状态 |
| 定位 | 左 DT35 3 个新样本中值 + IMU 实测 yaw 确定 X，Y 固定为半车长；只锚定一次，以后使用本地融合里程计 | 小电脑绝对 XY/yaw 与 IMU 融合，含门限、延迟补偿和重捕获 |
| 路线 | 6 段 Manhattan 直线；只约束当前段 X 或 Y，运动仍由人控制 | 路点、曲线插值/纯追踪、速度剖面、轨迹验收 |
| yaw | 所有输出 `z=0`，原 IMU yaw hold 目标设为 `0°`；定点几何例外地使用当时实测 yaw | 自动控制器内部 yaw-lock，并参与自动轨迹跟踪/故障判断 |
| 状态管理 | 轻量布尔状态、段号、按键沿和诊断结构；没有复制 B-team FSM | 含自动运行状态、`fsm`、`fault_manager`、`logger`、恢复策略 |
| 激光 | 前光限制 `+Y`，左光限制 `-X`；动态制动距离；0 值按 5 cm 保守处理；离线不停机 | 以前激光急停/降速及期望墙距为主；配置含掉线停车、无回波策略和可选左侧微调 |
| 地图 | 复制 7 个墙矩形；膨胀后只用于射线净空和 6 段终点 | 同一墙表用于自动轨迹生成、验收、期望激光距离及可视化 |
| 控制周期 | `Path_Run1ms()` 在底盘任务每 1 ms 运行 | `PathRunner_Run()` 由通信任务调用，内部按 5 ms 控制周期分频 |
| 轮径参数 | 融合里程计按实车轮径 `0.152 m`、直驱比 `1.0` | B-team `path_config.h` 使用轮半径 `0.050 m` 和 `190.99 RPM/(m/s)` |
| 调试接口 | `Path_GetDiagnostics()`；当前未主动发串口状态帧 | `PathRunner_GetDebug()`、轨迹读取、CSV、日志和 PC 状态帧 |

### 3.1 当前只从 B-team 采用了什么

来源：

```text
origin/arena/01a00390-taisheng:
user/path_planner/path_config.h
```

采用的静态事实：

- 场地尺寸 `3.0 m × 6.0 m`；
- 机器人长 `0.617 m`、宽 `0.440 m`；
- 4 面边界墙与 3 段内墙：
  - 南：`(0,0)–(3,0.049)`；
  - 北：`(0,5.951)–(3,6)`；
  - 西：`(0,0)–(0.049,6)`；
  - 东：`(2.951,0)–(3,6)`；
  - 墙 1：`(1.05,1.07)–(3,1.12)`；
  - 墙 B：`(0,2.075)–(2,2.125)`；
  - 墙 C：`(1.05,3.075)–(3,3.125)`。

没有采用 B-team 的路点表、自动曲线、速度剖面、纯追踪、上位机协议、融合策略、
FSM、故障恢复、日志或底盘所有权逻辑。

### 3.2 特别不能沿用的 B-team 假设

1. 当前没有小电脑，不能等待 `PcLink_GetPosition()`，也不能以 PC 掉线作为停车
   状态机条件。
2. 当前 DT35 子板实际输出被钳在 `5–20 cm`；不能把 0 当作“无回波且自由”。
3. 用户明确要求激光离线不停机；离线时只跳过对应方向激光限制并记录诊断。
4. 当前轮径是 `152 mm`，不能套用 B-team 的 `50 mm` 轮半径换算。
5. 当前 yaw 正常控制固定为 0，不能直接执行需要连续转向的 B-team 曲线。
6. 当前只允许人工命令，不应恢复 `PathPlanner_OwnsChassis()` 式自动接管。

---

## 4. 当前数据流与任务时序

```text
LoRa 本机控制帧
  └─ user/com_link/lora_link.c
       └─ Path_SubmitRemoteCommand(vx, vy, z, payload[4]六键, tick)
            └─ 通信任务写入 seqlock 遥控邮箱

DM-IMU 0x01 加速度帧 ─┐
IMU yaw/gyro 公共数据 ─┼─ PathLineImu_Run1ms()
四轮 VESC actual_rpm ──┘       └─ 融合速度、位置、累计路程

前/左 DT35 + 融合里程计 + 遥控邮箱
  └─ Path_Run1ms()
       ├─ 更新 yaw=0 hold
       ├─ 只收集左光一次性初始定位样本（前光仅用于安全）
       ├─ 更新地图坐标和当前路线段
       ├─ 应用单轴约束
       ├─ 应用膨胀地图净空限制
       ├─ 应用前/左激光动态安全限制
       └─ Chassis_SetVelocity() / Chassis_StopAll()
```

### 4.1 FreeRTOS 实际调用顺序

`Core/Src/freertos.c` 当前接入顺序：

1. `MX_FREERTOS_Init()` 创建任务前调用 `Path_Init()`；
2. `StartChassisTask()` 中依次初始化原 IMU、原底盘、`PathLineImu_Init()`；
3. 每个 1 ms 周期：
   - 原 `Chassis_Run1ms()`；
   - `PathLineImu_Run1ms(HAL_GetTick())`；
   - `Path_Run1ms(HAL_GetTick())`。

不要把 `Path_Run1ms()` 放到融合里程计之前，否则该周期会使用旧的速度/位置信息。

### 4.2 并发约定

- LoRa/通信任务是遥控邮箱单写者；底盘 1 ms 任务读取。
- 邮箱用 sequence + memory barrier 避免跨任务读取半包数据。
- `Path_Run1ms()` 提交底盘前会短暂关中断并复查 sequence，避免新遥控帧被旧计算
  结果覆盖。
- DT35 中断更新的距离、时间戳、在线状态在短临界区中读取为一致快照。
- 后续新增字段时，不要绕过这些并发保护直接跨任务共享复合状态。

---

## 5. 坐标、单位与控制语义

### 5.1 坐标

- 车体/地图 `+X`：向右；
- 车体/地图 `+Y`：向前；
- 前 DT35 光束：车体 `+Y`；
- 左 DT35 光束：车体 `-X`；
- yaw `0°`：车头朝地图 `+Y`；
- 正常输出旋转分量始终为 `z=0`，姿态由既有 IMU yaw hold 闭环维持。

### 5.2 单位

- 地图位置、激光几何：m；
- DT35 链路原始距离：cm；
- IMU 加速度：`m/s²`；
- 里程计速度：`m/s`；
- IMU yaw：deg；
- VESC `actual_rpm`：机械 RPM。

路径安全层将底盘平移命令按以下比例估算为速度：

```text
command_to_mps = π × 0.152 / 60
```

该比例建立在实车 152 mm 轮径、1:1 直驱以及现有底盘平移混控的机械 RPM
语义上。修改轮径或传动比时，必须同步修改
`user/path/imu/path_line_imu.h` 并重跑两套宿主测试。

---

## 6. 一次性左激光 X 定位与固定初始 Y

### 6.1 成功条件

初始地图位置只会设置一次，设置前必须同时满足：

1. `ImuMain_GetData()` 有效，IMU 为 READY、在线、yaw 有效；
2. 融合里程计至少有 IMU 解或编码器解；
3. 左 DT35 收到 3 个不同 `last_rx_ms` 的在线新样本；
4. 左光按实测 yaw 投影后能命中西边界有效 Y 范围。

左光 3 点取中值。该采样器与运行期激光安全使用的“最近 3 点最小值”完全
分开，不能合并。前 DT35 的样本、在线状态和读数不属于成功条件；即使前光
启动时无样本或离线，也必须能在左侧第 3 个新样本后完成锚定。

### 6.2 几何

- 左安装偏移：中心左方 `0.175 m`；目标为西边界内侧面 `x=0.049 m`；
- 机器人长度：`0.6170 m`；初始中心 Y 固定为长度一半 `0.3085 m`。

设左光中值距离为 `d_l`，定点周期实测 yaw 为 `ψ`：

```text
x = 0.049 + (0.175 + d_l) × cos(ψ)
y = 0.6170 / 2 = 0.3085
```

`path_localization.c` 计算左光实际墙面交点，要求落在西墙有效 Y 范围，并拒绝
`cos(ψ) <= 0`、NaN/Inf 和地图外结果。它不再用前光反算 Y，也不再校验前光
与墙 B 的交点。既有函数参数和诊断结构中的前光初始字段只为接口兼容保留；
`front_initial_sample_count`、`front_initial_distance_m`、
`front_wall_hit_x_m` 均保持为 `0`。

启动位置前 DT35 无法扫到原定位目标，其 `20 cm` 可能是真实距离，也可能是
超过量程后的饱和值，因此绝不能据此锚定坐标。前 DT35 仍参与运行期 `+Y`
动态限速，不能因取消定位依赖而删除该安全功能。

### 6.3 锚定之后

成功时保存：

```text
map_origin_x = initial_x - fused_odom_x
map_origin_y = 0.3085 - fused_odom_y
```

之后：

```text
map_x = map_origin_x + fused_odom_x
map_y = map_origin_y + fused_odom_y
```

不再使用激光重定位，也不依赖任何外部绝对位姿。当前没有运行期重定位 API；
如需重新定点，应重新上电或在用户明确要求后设计受控 reset 流程。

初始定位完成前，动态激光方向保护和 yaw=0 仍运行，但依赖地图位姿的墙体净空、
分段终点和自动取消不可用。当前需求确认正常使用场景会收齐左光样本，因此没有
增加“样本未收齐即全局停车”的策略。

---

## 7. 人工单轴路线

本机控制帧 `payload[4] bit0`（按键 1）按下沿切换单轴模式；按住不重复触发。
肩键不承担使能、取消或选轴功能。

| 段号 | 允许轴 | 自动判定终点 |
| ---: | --- | --- |
| 0 | Y | `y >= 1.650 m` |
| 1 | X | `x >= 2.480 m` |
| 2 | Y | `y >= 2.600 m` |
| 3 | X | `x <= 0.360 m` |
| 4 | Y | `y >= 3.700 m` |
| 5 | X | `x >= 0.500 m` |

- 单轴模式只把非当前轴清零，不代替操作员决定正/反方向和速度。
- 越过终点后自动取消单轴模式、停车，并置 `neutral_rearm_required`。
- 摇杆 X/Y 都回到绝对值 `<=3` 后，才重新接受平移命令。
- LoRa 超过 200 ms 无帧时，原有停车逻辑仍运行，同时取消单轴模式。

路线常量在 `user/path/path_map.c` 的 `path_map_route[]`。修改终点时必须同步修改
测试，不要从 B-team 路点表直接复制曲线点。

---

## 8. 安全策略

### 8.1 激光方向性

- 前 DT35 只限制朝前的 `+Y`；负 Y 逃离不受它阻止；
- 左 DT35 只限制朝左的 `-X`；正 X 逃离不受它阻止；
- 右侧和后方没有 DT35，不能宣称有动态障碍激光保护。

### 8.2 动态距离

```text
d_required = d_base + v × 0.080 + v² / (2 × 2.0)
```

- 前 `d_base=0.12 m`，释放到 `0.14 m`；
- 左 `d_base=0.10 m`，释放到 `0.12 m`；
- `v` 取当前请求逼近速度和编码器实测逼近速度的较大者；
- 实测逼近速度超过当前距离允许值 `0.03 m/s` 以上会触发制动；
- 在线异常 0 会钳为 5 cm，不得解释为自由空间；
- 距离减小首帧生效，距离增大要等最近 3 点中的旧小值被覆盖；
- 任一路激光离线时不因离线停车，只跳过对应方向激光限制。

### 8.3 静态地图保护

- 边界中心净空：X 为半车宽 + `5 cm`，即 `0.2700 m`；Y 为半车长 +
  `2 cm`，即 `0.3285 m`；
- 3 段内墙按同样车体外形和余量膨胀；
- 只沿当前人工请求方向做射线净空，并根据反解制动距离缩放平移向量；
- 净空 `<=0.01 m` 时硬停。

静态墙保护依赖一次性锚定后的融合里程计，会随里程计漂移降低精度，不能替代
操作员观察或实际障碍传感器。

---

## 9. 文件职责与接手修改位置

| 文件/目录 | 职责 | 常见修改入口 |
| --- | --- | --- |
| `user/path/path.c` | 1 ms 总控、遥控邮箱、模式切换、常规/镜像侧识别与锚定、回程（掉头 180°+回程路线+激光方向反转+末段激光校正+归零）、yaw、激光滤波、地图/激光限制、底盘输出 | 运行策略、阈值和诊断 |
| `user/path/path.h` | 对外 API 与 `path_diagnostics_t` | 新增只读诊断/API |
| `user/path/path_localization.c/.h` | 左激光 + yaw 定 X、半车长固定 Y 的纯定位 | 左安装偏移、西边界目标面和几何校验 |
| `user/path/path_map.c/.h` | 常规/镜像两套墙表与 6 段路线、`PathMap_SetMirrored()` 侧别选择、边界/内墙膨胀、射线净空 | 地图、车体净空、分段终点、镜像几何 |
| `user/path/path_safety.c/.h` | 动态制动距离和命令缩放纯函数 | 制动公式（同时补单元测试） |
| `user/path/tests/` | 地图、定位、安全和运行时宿主测试 | 每次策略变更必须补测 |
| `user/path/imu/path_line_imu.c/.h` | IMU 加速度 + VESC 编码器二维融合里程计 | 轮径、滤波、权重、ZUPT |
| `user/path/imu/tests/` | 融合里程计宿主测试 | 机械参数和融合算法回归 |
| `Core/Src/freertos.c` | 初始化和 1 ms 调用入口 | 只在任务时序确需变化时修改 |
| `user/com_link/lora_link.c` | LoRa 遥控命令与超时入口 | 遥控协议字段变化时修改 |
| `user/com_link/computer_link.c` | 阻止旧非遥控速度绕过安全层 | 通信控制策略明确变化时修改 |
| `user/imu/imu.c` | 已校验 `0x01` 帧转发 | IMU 帧协议变化时修改 |
| `user/imu/imu_main.c` | 开启加速度主动上报 | IMU 配置协议变化时修改 |
| `MDK-ARM/b-up.uvprojx` | Keil include path 与源文件 group | 新增/删除编译单元时同步 |

### 9.1 对外 API

人工路径模块：

```c
void Path_Init(void);
void Path_SubmitRemoteCommand(int16_t *vx, int16_t *vy, int16_t *z,
                              uint8_t six_buttons, uint32_t now_ms);
void Path_NotifyRemoteOffline(uint32_t now_ms);
void Path_ReplaceNonRemoteCommand(int16_t *vx, int16_t *vy, int16_t *z);
void Path_Run1ms(uint32_t now_ms);
bool Path_GetDiagnostics(path_diagnostics_t *diagnostics);
```

地图模块镜像侧/回程选择（一般由 `path.c` 自动调用，也可手动强制）：

```c
void PathMap_SetMirrored(bool mirrored);
bool PathMap_IsMirrored(void);
void PathMap_SetReturnMode(bool returning);
bool PathMap_IsReturnMode(void);
const path_map_route_segment_t *PathMap_GetReturnRoute(uint8_t *count);
```

### 9.2 对抗镜像侧（场地互换）实现要点

- **语义**：左右镜像（`x' = 3 − x`）。镜像场地要求 3 段内墙物理摆到镜像
  位置（墙 1、墙 C 贴西墙，墙 B 贴东墙），机器人贴东墙起步，车头仍朝
  `+Y`，路线是常规路线的镜像（X 段方向取反、终点 `0.52 / 2.64 / 2.50`）。
- **自动识别**：DT35 子板超量程输出钳在 `20 cm`，饱和视为“没有目标”。
  未定点时左光采集只收 `< 20 cm` 样本；前光读数首次降到 `20 cm` 以下时
  判定一次：左光有目标 → 常规侧；左光无目标 → 镜像侧。
- **镜像锚定**：`PATH_MAP_MIRRORED_START_X_M = 2.626 m` 为假定贴东墙
  对称起点；Y 用前光安装偏移 `PATH_LOCALIZATION_FRONT_SENSOR_OFFSET_M`
  （`0.225 m`）反算镜像场地第一堵前墙（贴东墙的墙 B，南面
  `y = 2.075 m`）：`y = 2.075 − 0.225 − d_front`。不依赖 yaw 和左光。
- **左光门控**：镜像侧左光无目标时跳过左光限速（20 cm 饱和是假障碍），
  出现 `< 20 cm` 真实目标时限速恢复；常规侧行为不变。
- **已知边界**：常规侧摆位离西墙超过 `20 cm` 会误判为镜像侧；镜像侧
  X 精度取决于贴东墙摆放的一致性（无传感器可测）。

### 9.3 回程（掉头 180° 返回起点）

- **触发**：去程 `route_complete` 后按遥控六键 **bit1（按键 2）**进入
  回程；回程完成后再按退出、重新开始去程。切换都要求摇杆回中。
  **去程走完后不必立刻回程**：可继续前进完成任务，回程路线从按键时
  所在位置（墙间走廊内任意点）引导回起点。
- **掉头**：`Path_UpdateYawZeroLock` 把 yaw 目标从 `0°` 切到 `180°`，
  旋转指令由底盘任务 `ImuMain_CalcOmega` 注入（本层 z=0 不阻塞）。
  旋转未对齐（`|Δyaw| > 20°`）期间平移命令置 0；地图坐标始终实时跟随
  里程计（原地旋转不产生平移位移，不做坐标冻结）。
- **回程路线**：`PathMap_GetReturnRoute()`，常规侧
  `X-0.36 → Y-2.60 → X+2.48 → Y-1.65 → X-0.374 → Y-0.3085`；
  镜像侧为对应镜像。按键 1 照常逐段单轴。
- **激光方向**：回程（车头 -Y）前光保护 `-Y`（`vy<0`）、左光保护
  `+X`（`vx>0`），`Path_ApplyLaserLimits` 内部按车头归一化；
  回程左光 `20 cm` 饱和视为假目标跳过（去程常规侧贴西墙仍保守限速）。
- **漂移校正**：回程最后一段前光 `< 20 cm` 时
  `y = 0.049 + 0.225 + d_front` 实时校正 Y；镜像侧左光读到东墙时
  `x = 2.951 − 0.175 − d_left` 实时校正 X。前光钳到 `5 cm`（真实距离
  低于量程下限）= 贴南墙，物理兜底判定回到起点。
- **归零**：回到起点后地图坐标强制设为初始锚定位置（吸收全程漂移），
  `return_complete` 置位并停车。

融合里程计：

```c
void PathLineImu_Init(void);
void PathLineImu_OnAccelerationFrame(const uint8_t *frame,
                                     uint8_t length,
                                     uint32_t now_ms);
void PathLineImu_Run1ms(uint32_t now_ms);
bool PathLineImu_GetData(path_line_imu_data_t *data);
void PathLineImu_ResetPosition(void);
```

`Path_GetDiagnostics()` 当前是主要观测入口，但工程没有自动把它发到串口/小电脑。
实机调试可先用 Keil watch 读取；如后续需要遥测，应新增只读输出，不要引入自动
控制指令或改变底盘所有权。

---

## 10. 验证方法

### 10.1 宿主测试

在仓库根目录运行：

```bash
user/path/tests/run_host_tests.sh
user/path/imu/tests/run_host_tests.sh
git diff --check
```

路径测试覆盖：

- 实测 yaw 下的左光 X 几何、左墙交点和固定 `Y=0.3085 m`；
- 前光无样本/离线时，仅凭左光 3 个新时间戳样本中值完成锚定；
- 一次性锚定以及后续里程计偏移；
- 7 个墙矩形、正/负方向终点和膨胀墙净空；
- 动态制动公式与命令缩放；
- 按键沿、单轴限制、自动取消和摇杆回中重启；
- 激光方向性、0 值保守处理、离线不停机；
- yaw=0 和遥控超时。

里程计测试覆盖 X/Y 积分、四轮逆运动学、152 mm 轮径换算、yaw 坐标旋转、
编码器掉线权重和异常加速度拒收。

### 10.2 Keil 工程 XML 检查

```bash
python3 - <<'PY'
import xml.etree.ElementTree as ET
p = "MDK-ARM/b-up.uvprojx"
ET.parse(p)
print("Keil project XML: OK")
PY
```

还应确认工程中恰好包含以下新增编译单元：

```text
user/path/imu/path_line_imu.c
user/path/path.c
user/path/path_map.c
user/path/path_localization.c
user/path/path_safety.c
```

### 10.3 实机最小验收顺序

1. 架车/静止上电，等待 IMU READY 和加速度零偏完成；
2. 保持前 DT35 无样本或离线，确认左 DT35 在线且
   `left_initial_sample_count` 随 3 个新时间戳从 0 到 3，前计数恒为 0；
3. 确认 `initial_position_valid=true`，核对左光中值、实测 yaw 下的初始 X、
   固定 `Y=0.3085 m` 和左墙交点；
4. 确认 `yaw_zero_lock_ready=true`、`output_z=0`；
5. 恢复前 DT35 后低速验证 `+Y` 接近前障碍时受限、`-Y` 仍可退出；
6. 低速验证 `-X` 接近左障碍时受限、`+X` 仍可退出；
7. 模拟单路激光离线，确认只标记 offline、不因离线停车；
8. 按本机按键 1，逐段确认非当前轴清零、越过终点自动取消和回中再启；
9. 断开 LoRa 超过 200 ms，确认停车并取消单轴模式；
10. 逐段靠近地图墙前低速验证净空限制，首次实车测试必须保留人工急停条件。

---

## 11. 后续 AI 修改守则

1. 先确认需求仍是“没有小电脑、全程人工、yaw=0”；不明确就询问用户。
2. 新增人工路径功能优先放在 `user/path/`；不要复制 B-team planner。
3. 既有工程文件只做最小追加式接入，不删除、替换或改写原功能。
4. 修改地图时只把 B-team 当静态几何参考，并明确墙面是外边界、内侧面还是矩形。
5. 修改定位时保留“每路 3 个新样本中值”和“只锚定一次”，除非用户明确更改。
6. 修改动态安全时保留方向性逃离能力；激光离线策略不能擅自改成全局停车。
7. 不把 DT35 的 0 当作无回波自由；先以当前子板 `5–20 cm` 语义为准。
8. 不用肩键做路径使能、取消或选轴；当前仅按键 1 切换单轴模式。
9. 所有旋转输出继续为 0；初始定位几何仍使用 IMU 当时实测 yaw。
10. 每次修改至少运行两套宿主测试、`git diff --check` 和 Keil XML 检查；涉及
    硬件参数时还要做低速实机复核。
11. 提交前用以下命令重新审计范围，确保没有无关文件：

```bash
git status --short
git diff --name-status
git diff --stat
git diff --check
```

如果需要重新查看 B-team 固定版本，不要切换当前分支，可直接使用：

```bash
git show fd99b528aca9e29adf061ea5468e78e9dab101df:user/path_planner/path_config.h
git show fd99b528aca9e29adf061ea5468e78e9dab101df:user/path_planner/README.md
```
