# 二维 IMU / VESC 融合里程计

本目录是独立于 `user/imu` 的里程计模块。它只通过原模块的公开数据接口读取
姿态和四轮 VESC 状态，并由原 IMU 解析层转发 `0x01` 加速度帧。

## 坐标与单位

- 车体系 `+X`：机器人向右；车体系 `+Y`：机器人向前。
- DM-IMU `0x01` 帧的传感器 `+X/+Y` 分别直接映射到车体系 `+X/+Y`。
- 加速度为 `m/s²`，速度为 `m/s`，位置为 `m`，角度为度。
- 世界系使用 `ImuMain` 的右手系 yaw 将车体系数据旋转得到。
- 轮径 152 mm（半径 0.076 m），传动比 1:1；`actual_rpm` 按机械 RPM 直接换算，
  不再除以电机极对数。

## 处理链

1. 缓存并解析 DM-IMU `0x01` 帧中的 X/Y/Z float。
2. 对 X、Y 分别做 200 Hz / 25 Hz 二阶 Butterworth 低通。
3. 静止窗口完成初始水平零偏，并在静止时缓慢跟踪零偏。
4. 使用加速度方差、Z 轴角速度和四轮速度共同判断静止，执行二维 ZUPT。
5. 用 yaw 把车体系水平加速度转至世界系，梯形积分得到二维 IMU 速度、位置和累计路程。
6. 通过与底盘混控矩阵严格互逆的四轮机械 RPM 解算车体系 X/Y 速度；新鲜反馈继续积分编码器位置和累计路程。
7. 根据四轮反馈年龄自适应调整编码器权重，并对融合速度作梯形位置和累计路程积分。

`imu_distance_m`、`encoder_distance_m` 和 `fused_distance_m` 都是速度模长
`sqrt(vx²+vy²)` 对时间的梯形积分，表示累计路程而非当前位置的位移模长。
因此机器人原路返回后 X/Y 位置可以回到零附近，而累计路程不会减少。

## 与原底盘解算的机械参数核对

原“抬升分支”的 `chassis_main.c` 输出四轮机械 RPM，平移混控为：

```text
LF = vx + vy + rotation       RF = vx - vy + rotation
LR = vx - vy - rotation       RR = vx + vy - rotation
```

本模块严格采用其逆解：

```text
vx = (LF + RF + LR + RR) / 4
vy = (LF - RF - LR + RR) / 4
```

原底盘的 `CHASSIS_ROTATION_SCALE = 3.5 + 3.30` 只把旋转命令换算成
四轮差速量；该项在上述平移逆解中严格相消，不是轮径或米制轮距，不能再次乘入
平移路程。原底盘代码本身未定义轮径，也没有 RPM 到米制速度的换算。

实车轮径固定为 `0.152 m`、半径 `0.076 m`、直驱比 `1.0`，所以每轮：

```text
wheel_mps = actual_rpm * (pi * 0.152) / 60
```

`vesc_motor.c` 已使用 21 极对数把 VESC eRPM 转为 `actual_rpm`，本模块直接使用
该机械 RPM，不重复除以 21。定量校验为：四轮同为 60 RPM 时，底盘沿 +X
每秒走一圈周长，即 `pi * 0.152 = 0.477522 m`。

编码器权重参数沿用参考实现的策略：60 ms 内目标权重 0.5，60~1000 ms
线性衰减，连续 3 次超时判为掉线，连续 5 次新鲜反馈恢复，权重按 1.0/s
爬升或下降。反馈年龄取四个轮子中最旧的一帧，避免任一轮失效时仍把整组反馈
误判为新鲜。

## 接口

- `PathLineImu_Init()`：初始化。
- `PathLineImu_OnAccelerationFrame()`：由 `user/imu/imu.c` 转发加速度帧。
- `PathLineImu_Run1ms()`：由底盘 FreeRTOS 任务每 1 ms 调用。
- `PathLineImu_GetData()`：读取 IMU、编码器、融合里程计与诊断量。
- `PathLineImu_ResetPosition()`：清零三套位置和累计路程，保留速度、零偏和权重。

上电后必须保持机器人静止，直到 `accel_bias_ready` 置位；按照 200 Hz 加速度
输出，静止窗口最少需要约 0.5 秒。`imu_solution_valid` 表示姿态、加速度帧与
水平零偏均已就绪，`encoder_solution_valid` 表示四轮反馈已通过掉线/恢复确认。

## 主机算法测试

仓库不包含 Keil 命令行编译器时，可先在带 C11 编译器的主机运行纯算法测试：

```sh
./user/path/imu/tests/run_host_tests.sh
```

测试覆盖 X/Y 两轴零偏和积分、四轮逆运动学、机械 RPM 换算、60 RPM 一秒
一圈的路程比例、往返时路程与位移的区别、yaw 坐标旋转、编码器权重掉线衰减，
以及异常加速度拒收。
