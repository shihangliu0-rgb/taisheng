# path_planner —— 离线速度剖面 + 在线跟踪(STM32 C 实现)

原 Python 方案(speed_profile_project,10 个模块)的 C 版本,直接跑在抬升分支的
STM32H723 工程里,数据全部来自仓库已有外设模块,没有仿真层。

## 1. 模块对应关系

| C 模块 | 原 Python 模块 | 说明 |
| --- | --- | --- |
| `path_config.h` | field.yaml | 全部参数集中成宏(墙体/路点/速度剖面/控制器增益) |
| `path_geometry.c/.h` | geometry.py | AABB 墙、膨胀、距离场、Slab 射线投射、坐标变换 |
| `path_spline.c/.h` | bspline_smoother.py | 三次 B 样条(de Boor)、弧长、曲率、离墙外推、最小转弯半径整形 |
| `path_speed_profile.c/.h` | speed_profile.py | 曲率限速 + 前/后向扫描、期望激光查表、CSV 串口输出 |
| `path_fusion.c/.h` | imu_fusion.py | 陀螺积分 + 上位机互补融合(30cm/20° 门限、3 点中值) |
| `path_pure_pursuit.c/.h` | pure_pursuit.py | 自适应前视纯追踪 |
| `path_yaw_lock.c/.h` | yaw_lock.py | 死区 ±1°、分段增益、速度自适应限幅 |
| `path_runner.c/.h` | profile_runner.py | 总控:查表 + 纯追踪 + 激光兜底 + 横向微调 + 航向锁 |

## 2. 与仓库已有模块的对接(数据流)

```
小电脑 0x11 位置帧 --UART7--> user/pc_link     --PcLink_GetPosition()--> path_fusion
IMU(USART1)        --------> user/imu         --ImuMain_GetData()------> path_fusion(gyro/yaw)
DT35/PNP(UART9)    --------> user/com_link    --dt35_link[].distance_cm-> path_runner(前/左激光)
path_runner        --Chassis_SetVelocity()---> user/chassis_vesc(4x VESC, FDCAN)
```

- 前激光 = `dt35_link[SENSOR_LINK_F_INDEX].distance_cm`(DT35_F, 地址 0x40)
- 左激光 = `dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm`(DT35_L, 地址 0x41)
- 上位机 `field_w` 为 **yaw_rad**(小电脑侧已修复,直接与 yaw-lock 协同)。

## 3. 坐标系(重要,勿改)

- 世界系:原点左下角,+x 右,+y 前;用户 yaw:0 朝 +y,逆时针为正(同 field.yaml)
- 车体系:+x 前,+y 左;变换用 `R(yaw + pi/2)`(path_geometry.c 有注释)
- 底盘系(chassis_main.c 实际约定):**x=向右、y=向前、z=逆时针**;
  变换直接 `R(yaw)`,输出 `Command` 与 `Chassis_SetVelocity` 一一对应,
  无需再翻转符号
- `Chassis_SetVelocity(vx, vy, z)` 单位:vx/vy 为轮子 RPM,z 为角速度单位
  (内部乘 CHASSIS_ROTATION_SCALE=6.8 转成轮子 RPM 差),换算系数见
  path_config.h 第 1 节

## 4. 已做的工程改动(最小侵入)

1. `Core/Src/freertos.c`:StartCommTask 里接入
   `PcLink_Init()` / `PcLink_Run()` / `PathRunner_Init()` /
   `PathRunner_Run()`(USER CODE 区,规划器最后执行,每周期覆盖手动
   computer_link 指令);
2. `Core/Src/main.c`:串口回调已挂 `PcLink_RxCplt()` / `PcLink_Error()`
   (小电脑数据入口);
3. `MDK-ARM/b-up.uvprojx`:新增 `Application/User/path_planner` 分组,
   pc_link 加入 com_link 分组,IncludePath 增加 `../user/path_planner`
   与 `../user/pc_link`;
4. `user/pc_link/pc_link.c/.h`:增加 `PcLink_GetStats()` 好帧/CRC 计数、
   `PcLink_GetPositionSeq()` 新帧序号(供融合去重);
5. `path_runner` 初始化时调用 `ImuMain_EnableYawHold(false)`:
   关闭 IMU 模块自带航向保持,避免底盘 `ImuMain_CalcOmega` 覆盖本模块的
   z 指令(本模块用自己的 yaw-lock);运行时回传 `PcLink_SetStatus()`
   给小电脑状态帧。

## 5. 未知 / 待实测确认项(标定后再上车)

1. **轮径/轴距/轮距已从仓库底盘解算找到并写入**(path_config.h 第 1 节):
   轮半径 0.050m(底盘分支 odom_fusion.h)、轴距 0.35/轮距 0.33(抬升分支
   CHASSIS_ROTATION_SCALE=3.5+3.30),推导闭合:
   `PATH_RPM_PER_M_S=190.99`、`PATH_Z_PER_RAD_S=9.549`。
   装车后建议用"直行 1m / 原地转 90°"各验证一次(实测误差大再微调);
2. **左激光安装方向**:默认按 field.yaml 车体正左(挂 0,0.175,朝 +y);
   若实车 DT35_L 朝左后或安装偏移不同,改 `PATH_LASER_LEFT_*`;
3. **横向微调符号**:按底盘系推导为 `-Kp*err`(`PATH_LAT_TRIM_SIGN=-1`);
   若实车反而朝左墙贴,改成 `+1.0f`;左墙 <10cm 有强制向右修正保护;
4. **上电朝向已确认与 field.yaml 一致**(yaw=0 朝 +y),`PATH_YAW_TARGET_RAD=0`
   直接可用;
5. **路点表补点**:在 `(1.0,2.6)` 与 `(0.5,3.7)` 之间补了拐点 `(0.5,2.6)`
   (见 path_config.h 注释):原路点直连弦线穿过 wall_C(wall_C 注释写明
   "必须走到 x=0.5 才能上翻"),补点后路径沿墙 C 西侧缺口 x<0.7 通过;
6. **墙体几何**:按 field.yaml 反推,实测墙位不同只改 `PATH_WALLS_TABLE`;
7. **前激光兜底对横向贴墙场景的影响**:yaw 锁定朝 +y,东西向平移段前激光
   会照到正前方的隔墙(wall_B/wall_C 距激光约 0.2-0.25m),按公式
   `sqrt(2*a_brake*(d-0.12))` 会把该段限到约 0.5-0.7 m/s,全程预计
   6-7 秒(主机仿真实测值见测试输出);要提速需调整 a_brake/安全距离或
   兜底策略,请与算法组确认;
8. **规划器与手动上位机并发写底盘**:同任务内规划器后执行,每 5ms 覆盖
   一次,手动指令会被立即覆盖;比赛时请停发手动速度帧;
9. **CSV 落盘**:MCU 无文件系统,运行时剖面在 RAM 中只读;可用
   `PathSpeedProfile_DumpCsv(&huartX)` 从串口导出人工检查(主机侧存成
   speed_profile.csv / expected_laser.csv)。
## 6. 关键设计说明(与 field.yaml / 原 Python 方案的差异)

1. **矩形膨胀代替外接圆**:yaw 锁定(±2°)下机器人始终轴对齐,路径推离用
   半宽/半长 + 余量(0.30 / 0.3885)的矩形膨胀。外接圆(0.459)会把
   wall_C 西侧缺口(x<0.7)压成 x<0.24,比车宽 0.44 还窄,物理不可行;
2. **最小转弯半径 0.65m**:两个关键拐角(D 角、wall_C 西侧拐角)的墙角在
   弯道内侧,内切圆弧离墙角只有 0.2~0.34m,小于车角半径 0.379m 会擦墙。
   曲率整形把转弯点向弯道外侧推,形成"先远离墙再转弯"的外侧绕行弧
   (前激光装在前部中央,覆盖不到车头前角,这是真实盲区,只能靠路径保证);
3. **曲率自适应前视**:`Ld = min(Ld_min + Ld_k*v, 0.22/sqrt(|κ|+0.05))`,
   急弯缩短前视抑制抄近道(纯追踪抄近道是两次擦墙事故的根因);
4. **激光停车保留横向脱困**:前激光 <12cm 时只强制纵向 vx=0,保留目标方向
   的横向分量平移(全向轮可侧移绕开挡墙),否则贴墙后永久死锁;
5. **上位机帧去重融合**:pc_link 保存最近一帧、控制周期 5ms 远快于 50Hz
   帧率,必须按 `PcLink_GetPositionSeq()` 判断"新帧"才融合一次,否则中值
   滤波窗口会被同一个跳变值填满、跳变抑制失效(仿真复现过);
6. **融合门限顺序**:先对原始样本做 30cm 门限(超限整帧拒绝,不污染中值
   窗口),再用 3 点中值吸收 20cm 单帧跳变(满足"20cm 阶跃被拒绝"验收),
   yaw 20° 门限只拒绝 yaw 分量。

## 7. 仿真验证(主机侧,代码见 /tmp 测试桩,不进仓库)

用运动学仿真(真值 + 噪声 IMU/上位机/激光 + 20cm/40cm/23° 跳变注入)
跑了 20 次:0 碰撞、全部到达,用时 13.80s(含 1s 上电标定,实际行驶约
12.8s)。关键指标:最大 |yaw| 0.00°、最大融合误差约 5~6cm(50Hz 采样
滞后 + 中值滤波固有,瞬时值)、最大 v_used 1.5 m/s、跳变拒绝 xy=2 /
yaw=1、CRC 错误 0。可视化:仓库外的 `path_planner_sim_result.png`。

⚠️ 用时 13.8s 明显超过原 Python 验收的 3~5s,原因是 yaw 锁定时车头
始终朝 +y,东西向平移段的前激光正对隔墙(wall_B/wall_C 距激光仅
0.2~0.25m),按兜底公式 `sqrt(2*a_brake*(d-0.12))` 被限到 0.5~0.7 m/s
(约占全程 4m)。要在该几何下提速,需与算法组确认:a_brake 提高、兜底
只对"运动方向前方"生效、或调整安全距离——都不改物理也能提,但属于
策略取舍,不是代码缺陷。

## 8. 调试输出

`path_config.h` 打开 `PATH_DEBUG` 并定义 `PATH_DEBUG_UART_HANDLE`(建议用
空闲的 huart8,或临时把 huart2 借出),每 400ms 输出一行:
`t, state, reason, x, y, yaw, cmd(vx,vy,w), v_ref, v_used, laserF, laserL, crc`。

## 9. 安全行为汇总

| 条件 | 行为 |
| --- | --- |
| 上位机位姿丢失 > 500ms | Chassis_StopAll,STOP_UPPER_LOST |
| IMU 离线 | Chassis_StopAll,STOP_IMU_LOST |
| 前激光 < 12cm | vx/vy 置 0,仅保留航向锁 |
| 前激光离线(可配置) | Chassis_StopAll,STOP_LASER_LOST |
| 全程 > 15s | Chassis_StopAll,STOP_TIMEOUT |
| 到达终点 0.15m 内 | Chassis_StopAll,ARRIVED |
