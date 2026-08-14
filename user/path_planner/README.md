# path_planner 使用说明(简版)

跑曲线模块:上电后自动执行 标定 → 等位姿 → 建轨迹 → 跟踪 → 到达。全部参数在 `path_config.h`。

## 文件

| 文件 | 作用 |
|---|---|
| `path_config.h` | 全部参数(墙/路点/速度/阈值),调参只改它 |
| `path_main.h` | 接口声明 |
| `path_main.c` | 算法实现 |
| `map_check.py` | PC 画图核对场地:`python3 map_check.py` |

## 状态机(0x20 状态帧 state 字段)

```
0 INIT → 1 CALIB → 2 WAIT_START → 3 BUILD → 4 RUN → 5 ARRIVED
                   (陀螺零偏)(等位姿)(建轨迹) (跟踪)    任意故障 → 6 STOPPED
                                  ↑_____________________________|
                                  瞬态故障(位姿/IMU/激光/电机)恢复:
                                  RUN → STOPPED → 健康1s → WAIT_START → BUILD → RUN
                                  (最多自动重布防 3 次;确定性故障不恢复)
```

上电约 5~6s 后开始行驶;RUN 中 error=4 表示激光降速(正常)。

## 数据流

```
小电脑 0x11 位姿(UART7,50Hz) + IMU gyro(USART1) + 前/左激光 DT35(UART9,5-20cm)
    → 融合(15cm/20°门限 + 120ms 延迟补偿)
    → 查表 + 纯追踪 + 激光兜底 + yaw-lock
    → Chassis_SetVelocity(vx右, vy前, ω逆时针; RPM 单位) → FDCAN1 → 4×VESC
反向每 50ms 回小电脑 0x20 状态帧(在线判定)。
```

## 单位约定(改任何参数前先看)

| 量 | 单位 | 位置 |
|---|---|---|
| IMU `yaw_deg` / `gyro_z_deg_s` | deg、deg/s | imu_main 输出 |
| 上位机 `field_w`(0x11 帧) | **rad** | 融合输入 |
| 融合/控制器内部 yaw、ω | rad、rad/s | path 内部 |
| 速度 | m/s | 剖面/纯追踪 |
| 底盘 `Chassis_SetVelocity(vx,vy)` | **轮 RPM** | 输出,= m/s × 190.99 |
| 底盘 `z` | ω(rad/s) 控制量 | = ω × 9.549 |

防护:yaw 输入有量级门(|yaw|≤2π),把"度"当 rad 喂入会在起步被
拒绝(STOP_HEADING),不会带着错误角度乱跑。

## 接口(path_main.h)

| 函数 | 说明 |
|---|---|
| `PathRunner_Init()` / `PathRunner_Run()` | commTask 里 1ms 调用(内部 5ms 分频),已接好 |
| `PathRunner_GetDebug()` | 读调试信息(每 400ms 一刷) |
| `PathRunner_GetTrajectory()` | 读离线轨迹(CSV/上位机可视化) |
| `PathPlanner_OwnsChassis()` | RUN 期间 true,手动指令被屏蔽 |
| `PathSpeedProfile_DumpCsv()` | 串口导出剖面 CSV |

## 使用步骤

1. Keil 编译烧录(工程已配好,不用加文件);
2. 上电前:**车头朝 +y**;手推左转验证 gyro 符号(反了改 `PATH_GYRO_SIGN`);墙表与实测一致;
3. 改墙/路点后跑 `python3 map_check.py` 核对;
4. 首跑开 `PATH_DEBUG`(huart8 每 400ms 一行状态)。

## 常用参数(path_config.h)

| 宏 | 默认 | 说明 |
|---|---|---|
| `PATH_WALLS_TABLE` / `PATH_WAYPOINTS_TABLE` | — | 墙表 / 路点(起点由上电位姿覆盖) |
| `PATH_RPM_PER_M_S` / `PATH_Z_PER_RAD_S` | 190.99 / 9.549 | 轮径 0.05m 换算(标定改这里) |
| `PATH_V_MAX_MS` | 1.5 | 最大速度 |
| `PATH_ARRIVE_TOL_M` | 0.08 | 到达阈值(物理停点 ≤0.15) |
| `PATH_POSE_LATENCY_MS` | 120 | 位姿延迟补偿(实测标定) |
| `PATH_GYRO_SIGN` | 1.0 | gyro 符号反了改 -1 |

## 停机原因码(0x20 error 字段)

| 码 | 含义 | 处理 |
|---|---|---|
| 6 | 前激光 ≤12cm | 清障 |
| 7 | 位姿丢失 | 查 UART7/小电脑 |
| 8 | IMU 未就绪(10s 超时) | 查 IMU 接线 |
| 9 | 前激光无帧 | 查 UART9 |
| 10 | 轨迹验收失败 | 跑 map_check,查墙表 |
| 11 | 数值异常 NaN | 报修 |
| 12 | 电机离线 | 查 CAN/供电 |
| 13 | 起步朝向超 ±30° | 车摆正朝 +y |
| 14 | 30s 未到达 | 开调试看日志 |

## 常见现象 → 排查

| 现象 | 查 |
|---|---|
| 上电不动,状态停在 1 | IMU 是否 READY(10s 后应转 8) |
| 停在 2 | 小电脑没发 0x11 / 起步朝向超限 |
| 停在 3 → 10 | map_check 找穿墙点;墙表与真实不符 |
| 行驶中贴墙急停(6) | 激光安装方向 / 墙表 |
| 原地打转 | gyro 符号反了 |
| 到不了终点(14) | 开 PATH_DEBUG 看 400ms 日志 |

## 安全规则(4 条)

1. 故障**去抖**后才停:IMU 200ms / 电机 400ms / 激光 1500ms,单次抖动不误停;
2. 瞬态故障(位姿/IMU/激光/电机)停机后,健康 1s **自动重布防**,最多 3 次;
3. 所有停机走 VESC SET_BRAKE 锁存;ARRIVED/STOPPED 后遥控可直接接管;
4. 控制只用融合位姿;位姿缺 150ms 降速 0.3m/s、800ms 停机;
5. 轨迹进入 RUN 前强校验:首点=本次起点、末点=目标(±容差)、
   曲率/净距/间距全部达标,不通过 STOP_BUILD 不出车。

---

算法细节(融合/样条/速度剖面/验收)见 git 历史中的详细版说明或代码注释。
