# 激光测距安全限速模块（user/laser_safety）

只有**前方**和**左侧**两个激光测距（DT35），本模块用它们给底盘速度命令做最后一道
"预测式"限速，防止撞墙。模块独立成文件夹，只依赖 `chassis_main.h` 和
`dt35_pnp_link.h` 的现有接口，不改动这两者的实现。

## 1. 限速模型（刹车距离预测）

距离越近，允许速度越小；距离小于等于硬停距离时对应方向完全禁止：

```
margin_cm = 距离(cm) - 硬停距离(10cm)
margin <= 0        -> v_max = 0            （完全禁止该方向）
v_max >= 全速阈值  -> 不限制
其余               -> v_max = sqrt(2 * a * margin)
```

- `a` = 最大减速度，默认 **2 m/s²**（`LASER_SAFETY_BRAKE_ACCEL_MM_S2 = 2000` mm/s²）。
- 例：前方 15cm、突然下 1 m/s 前进命令 → 剩余 5cm，允许
  `sqrt(2*2000*50mm) ≈ 447 mm/s`，实际只能动 0.45 m/s 左右；
  距离 10cm 及以内时前进速度被压到 **0**，完全停止、不准向前。
- 距离远（模型算出的上限 ≥ `LASER_SAFETY_FULL_SPEED_MM_S = 2000` mm/s）时不做任何限制。

## 2. 方向约束

| 传感器 | 约束的运动 | 放行的运动 |
|---|---|---|
| 前方 DT35 | 向前速度 vy > 0 | 后退 vy < 0、左右平移、旋转 |
| 左侧 DT35 | 向左平移 vx < 0 | 向右平移 vx > 0、前后、旋转 |

z（旋转）不受限制。

## 3. 传感器离线策略

`dt35_link[x].online` 为 0（500ms 无数据）时**不限制**对应方向，只把
`front_offline / left_offline` 标志置位，方便调试观察。

## 4. 两层防护

1. **命令入口钳制**：所有底盘速度命令源（遥控 `lora_link.c`、上位机
   `computer_link.c`、自动底盘 `auto_chassis.c`、路径跟踪 `path_main.c`）都已改为
   调用 `LaserSafety_RequestVelocity()`，命令下发瞬间即按当前距离限速。
2. **周期纠偏**：`LaserSafety_Run1ms()` 在底盘 1ms 任务里每毫秒检查一次当前生效
   目标（`chassis_target_vx/vy/z`）。距离持续变小时不断压低目标，机器人边靠近边
   减速；到 10cm 时目标被压到 0。纠偏重发使用 100ms 短超时
   （`LASER_SAFETY_ENFORCE_TIMEOUT_MS`），命令源离线后命令到期即自动停止，
   不会无限续命。

## 5. 已接入的位置

- `Core/Src/freertos.c`：`Chassis_Run1ms()` 之后调用 `LaserSafety_Run1ms()`；
- `user/com_link/lora_link.c` / `computer_link.c`、`user/chassis_vesc/auto_chassis.c`、
  `user/path/path_main.c`：`Chassis_RequestVelocity(...)` →
  `LaserSafety_RequestVelocity(...)`，参数不变；
- `MDK-ARM/b-up.uvprojx`：新增 `LASER_SAFETY` 组和 `../user/laser_safety` 头文件路径。

## 6. 调试与调参

- `LaserSafety_GetStatus(&clamp)` 可读出最近一次钳制结果（vx/vy/vz、四个限速/阻塞
  标志、两个离线标志），方便在上位机或调试器里观察。
- 觉得刹得太保守（离墙很远就限速）→ 调大 `LASER_SAFETY_BRAKE_ACCEL_MM_S2`；
  反之调小。注意该值不能超过底盘实际能做到的减速度，否则模型"预测刹得住"会失真。
- 硬停距离改 `LASER_SAFETY_STOP_DISTANCE_CM`（默认 10cm）。
