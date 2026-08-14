/**
 ******************************************************************************
 * @file    path_config.h
 * @brief   路径规划与控制集中配置(原 Python 方案 speed_profile_project 的
 *          field.yaml 全部内容 + 控制器可调参数,统一收敛到本文件)
 *
 * 每个宏后都注明其对应的 field.yaml 段,改参数只改本文件。
 ******************************************************************************
 */
#ifndef PATH_CONFIG_H
#define PATH_CONFIG_H

#define PATH_PI                     3.14159265f   /* pi(float) */

/* ==================================================================
 * 0. 总开关 / 控制周期 / 调试
 * ================================================================== */
#define PATH_ENABLE                   1U     /* 1 = 启用路径规划与控制 */
#define PATH_CONTROL_PERIOD_MS        5U     /* 控制周期,默认 200 Hz */
#define PATH_MAX_RUN_MS               20000U /* 全程超时保护,超时主动停 */

/* 调试打印:默认关闭。打开后每 400ms 向 PATH_DEBUG_UART_HANDLE 指定的
 * 空闲串口输出一行状态(阻塞发送,约 10ms@115200,只建议调试时打开)。 */
#define PATH_DEBUG                    0U
/* #define PATH_DEBUG_UART_HANDLE    huart8 */
#define PATH_DEBUG_PERIOD_MS          400U

/* ==================================================================
 * 1. 底盘单位换算(对接 Chassis_SetVelocity)
 * ==================================================================
 * chassis_main.c 的底盘坐标系: X 向右(平移)、Y 向前、Z 逆时针;
 * vx/vy 单位是轮子 RPM,z 是角速度控制量(内部乘
 * CHASSIS_ROTATION_SCALE = 3.5+3.30 = 6.8 转成轮子 RPM 差)。
 *
 * 参数来源(仓库内底盘解算,非估算):
 *   - 轮半径 0.050 m     : bteam/底盘分支 chassis/app/chassis/odom_fusion.h
 *                          ODOM_WHEEL_RADIUS_M = 0.050f
 *   - 轴距 0.35 / 轮距 0.33 : 抬升分支 chassis_main.c 的
 *                          CHASSIS_ROTATION_SCALE(3.5f + 3.30f)
 *
 * 推导(与两边代码闭合):
 *   PATH_RPM_PER_M_S = 60/(2*pi*r) = 190.99 rpm per m/s
 *   麦轮旋转项:轮线速度 = w * (轴距/2 + 轮距/2) = w * 0.34
 *   所需轮 RPM = w * 0.34 * 190.99 = 64.94 * w
 *   代码 rotation_rpm = z * 6.8 -> z = 64.94/6.8 * w = 9.549 * w
 *   即 PATH_Z_PER_RAD_S = 60/(2*pi) = 9.549(与 6.8、0.34、0.05 三值闭合)
 */
#define PATH_WHEEL_RADIUS_M           0.050f
#define PATH_RPM_PER_M_S              190.99f
#define PATH_CHASSIS_ROT_ARM_M        0.34f   /* 轴距/2 + 轮距/2 = 0.175+0.165 */
#define PATH_Z_PER_RAD_S              9.549f  /* = 60/(2*pi),z 控制量 per rad/s */

/* ==================================================================
 * 2. 场地几何(field.yaml: field / walls)
 * ================================================================== */
/* 注意(P2-3):若场地 D 拐角处有圆形障碍(审计假设 R0.28@(2.55,1.95)),
 * 实测几何后在此追加一堵轴对齐近似墙并 +1 PATH_WALL_COUNT,例如:
 *   {2.27f, 1.67f, 2.83f, 2.23f},   // D 圆外接矩形近似
 * 当前墙表未建模该障碍(仿真 S9 靠激光兜底侥幸通过,不可依赖)。 */
#define PATH_FIELD_W_M                11.0f
#define PATH_FIELD_H_M                6.0f
#define PATH_WALL_THICKNESS_M         0.049f
#define PATH_WALL_COUNT               6U

/* 4 面外墙 + 2 面内部隔墙(wall_B / wall_C,wall_A 在 yaml 中已注释掉)。
 * 实测墙位不同时只改这张表。 */
#define PATH_WALLS_TABLE \
    {0.000f, 0.000f, 11.000f, 0.049f},   /* south 下墙 */      \
    {0.000f, 5.951f, 11.000f, 6.000f},   /* north 上墙 */      \
    {0.000f, 0.000f, 0.049f,  6.000f},   /* west  左墙 */      \
    {10.951f, 0.000f, 11.000f, 6.000f},  /* east  右墙 */      \
    {0.000f, 2.075f, 2.300f, 2.125f},    /* wall_B_mid_sep */  \
    {0.700f, 3.075f, 3.000f, 3.125f}     /* wall_C_upper_sep */

/* ==================================================================
 * 3. 机器人尺寸与膨胀(field.yaml: robot)
 * ================================================================== */
#define PATH_ROBOT_LENGTH_M           0.617f
#define PATH_ROBOT_WIDTH_M            0.44f
#define PATH_SAFETY_MARGIN_M          0.08f
/* 路径膨胀:yaw 锁定(±2°)下机器人始终轴对齐,用矩形半宽/半长 + 安全余量
 * 作为膨胀量,比 field.yaml 的外接圆(半对角线 0.379+0.08=0.459)更精确:
 * 圆模型会把 wall_C 西侧缺口(x<0.7)压成 x<0.24,比车宽 0.44 还窄,
 * 导致绕墙 C 的通道物理不可行。矩形膨胀才是该姿态下的真实占位。 */
#define PATH_INFLATE_DX_M             (0.5f * PATH_ROBOT_WIDTH_M + PATH_SAFETY_MARGIN_M)   /* 0.30 */
#define PATH_INFLATE_DY_M             (0.5f * PATH_ROBOT_LENGTH_M + PATH_SAFETY_MARGIN_M)  /* 0.3885 */
/* 硬膨胀(轨迹验收用):机器人真实外廓 + 1.5cm。墙C西侧缺口等窄通道
 * 无法在"软膨胀+8cm余量"下绕行,验收以硬膨胀为准(软膨胀只用于整形);
 * 线段切角允许 2mm,采样直连弦与真实曲线的偏差即此量级 */
#define PATH_HARD_MARGIN_M            0.015f
#define PATH_SEGMENT_CUT_EPS_M        0.002f

/* ==================================================================
 * 4. 固定路线 waypoints(field.yaml: route,起点由上电位姿覆盖)
 * ==================================================================
 * 注意: 与 field.yaml 的差异:在 (1.0,2.6) 与 (0.5,3.7) 之间补了拐点
 *    (0.5, 2.6)。原因:wall_C(x∈[0.7,3.0], y≈3.1)挡住了 F->G 的直连弦线
 *    (弦线在 y=3.075 处 x≈0.78,落在墙内),且 wall_C 注释写明
 *    "让机器人必须走到 x=0.5 才能上翻" —— 即原路点表漏掉了这个拐点。
 *    先向西走到 x=0.5,再向北穿过 wall_C 西侧缺口(x<0.7)。
 */
#define PATH_WAYPOINT_COUNT           12U
#define PATH_WAYPOINTS_TABLE \
    {0.50f, 1.00f},   /* 名义起点占位:上电后由小电脑实测位姿整体覆盖 */ \
    {1.00f, 1.65f},   /* 通道1 左入口 */               \
    {2.00f, 1.65f},   /* 通道1 中段 */                 \
    {2.45f, 1.65f},   /* D角圆弧起点(R=0.65 fillet) */ \
    {2.70f, 1.70f},   /* D角圆弧点1 */                 \
    {2.91f, 1.84f},   /* D角圆弧点2 */                 \
    {3.05f, 2.10f},   /* D角圆弧点3 */                 \
    {3.05f, 2.45f},   /* D角圆弧终点 */                \
    {1.00f, 2.65f},   /* 通道2 左段(2.65:远离墙B) */   \
    {0.55f, 2.50f},   /* 缺口转弯过渡点(下探,给纯追踪切角留余量) */ \
    {0.34f, 2.45f},   /* 墙C西侧缺口入口:走 x=0.34(左侧余 7cm,
                          右侧余 14cm,把切角余量留宽一倍);直角弯在
                          y<2.75 完成,否则扫到 wall_C 西南角(0.7,3.075) */ \
    {0.34f, 3.70f}    /* 终点:缺口列直行北上,距目标(0.5,3.7)
                          0.16m(略超 0.15 容差,靠到达判定在途中触发) */

#define PATH_GOAL_X_M                 0.50f
#define PATH_GOAL_Y_M                 3.70f
#define PATH_ARRIVE_TOL_M             0.15f
/* 上电后等待上位机位姿的时间,超时用 yaml 起点 */
/* 起点完全由小电脑首帧有效位姿给出(不是表内占位点):
 * WAIT_START 收到首帧"数值合法、在场内、|yaw|<=30 度"的位置帧后,
 * 用实测位姿整体覆盖 waypoints[0],不设与占位点的距离门限。
 * 表内 (0.5,1.0) 仅为名义起点注释,正常流程必然被覆盖;
 * 若实测起点离名义路线太远导致轨迹被墙拦断,BUILD 验收会拒绝并
 * STOP_BUILD(不会盲跑)。 */
/* 起步朝向硬约束(P0-1):本任务物理前提是上电 yaw=0 朝 +y。
 * WAIT_START 收到首帧有效位姿后检查 |field_w|,超过该值直接
 * STOP_HEADING(比"yaw 门限锁死后全速撞墙"安全) */
#define PATH_START_YAW_LIMIT_DEG      30.0f
/* CALIB 总超时(P1-5):IMU 未插/静默时不能永远卡在 CALIB */
#define PATH_CALIB_TIMEOUT_MS         10000U

/* ==================================================================
 * 5. B 样条(field.yaml: bspline)
 * ================================================================== */
#define PATH_SPLINE_DEGREE            3U
#define PATH_SPLINE_SAMPLES           300U
/* 把落入膨胀墙的采样点沿距离场梯度外推 */
#define PATH_PUSH_STEP_M              0.02f
#define PATH_PUSH_MAX_ITERS           60U
/* 推离 + 拉普拉斯平滑交替迭代轮数 */
#define PATH_PUSH_SMOOTH_ROUNDS       4U
/* 弯道半径:拐角 D 由路点表内 R=0.65 fillet 采样点保证(见路点注释)。
 * 墙C缺口(x<0.7 的 5cm 窄通道)几何上无法做 0.65m 半径,由曲率限速
 * (v^2*|k| <= a_lat)+ 曲率自适应前视保证跟踪精度。 */
#define PATH_SAMPLE_STEP_MAX_M        0.15f   /* 最大采样点间距 */
#define PATH_MIN_CLEARANCE_M          0.02f   /* 中心路径到硬膨胀墙的最小净距 */
#define PATH_KAPPA_HARD_MAX           50.0f   /* 验收硬上限(R=0.02):缺口窄 S 弯的
                                                 固有尖峰,由速度剖面强制
                                                 v<=sqrt(a_lat/|k|) 低速通过 */
#define PATH_LAT_ACC_TOL              1.20f   /* 横向加速度超限容差 */
#define PATH_BUILD_MAX_ATTEMPTS       3U      /* 整形+验收的最大尝试次数 */
#define PATH_REQUIRE_MOTORS           1U      /* 任一电机离线 -> 停车 */

/* ==================================================================
 * 6. 速度剖面(field.yaml: speed_profile)
 * ================================================================== */
#define PATH_V_MAX_MS                 1.5f
#define PATH_V_START_MS               0.0f
#define PATH_V_GOAL_MS                0.30f
#define PATH_V_MIN_MS                 0.20f
#define PATH_A_LAT_MAX                1.5f
#define PATH_A_LON_ACCEL              1.2f
#define PATH_A_LON_BRAKE              1.8f
#define PATH_KAPPA_MIN                0.01f   /* 曲率下限,防止除零 */

/* ==================================================================
 * 7. IMU + 上位机位姿互补融合(原 imu_fusion.py)
 * ================================================================== */
#define PATH_FUSION_XY_GATE_M         0.15f   /* 单帧 xy 跳变门限(>15cm 拒绝:
                                                 20cm 阶跃必拒;机器人最快 3cm/帧,
                                                 正常运动远低于该门限) */
#define PATH_FUSION_YAW_GATE_DEG      20.0f   /* yaw 与预测差 >20° 拒绝 */
#define PATH_FUSION_YAW_GAIN          0.15f   /* yaw 低通拉回增益 */
#define PATH_FUSION_UPPER_TIMEOUT_MS  500U    /* 链路丢失判定(CRC 有效帧刷新) */
#define PATH_FUSION_CALIB_SAMPLES     200U    /* 静止标定采样帧数(约1s) */
/* 数据可用性(通过 15cm/20 度门限才刷新):可用数据年龄超过
 * DEGRADE_MS 限速到 DEGRADE_V_MS,超过 DATA_STOP_MS 判定定位不可用停机 */
#define PATH_UPPER_DEGRADE_MS         100U   /* 5 帧缺失即降速(P1-1:盲开窗口 <=100ms) */
#define PATH_UPPER_DEGRADE_V_MS       0.30f
#define PATH_UPPER_DATA_STOP_MS       800U
#define PATH_GYRO_SIGN                1.0f    /* IMU z 轴与 yaw 反向时改 -1 */

/* ==================================================================
 * 8. 纯追踪(原 pure_pursuit.py)
 * ================================================================== */
#define PATH_LD_MIN_M                 0.15f
#define PATH_LD_K_S                   0.12f
/* 曲率自适应前视上限:急弯处缩短前视距离,防止纯追踪抄近道切墙 */
#define PATH_LD_KAPPA_MAX_M           0.15f
#define PATH_SEARCH_WINDOW            150U    /* 前向最近点搜索窗口 */
#define PATH_SEARCH_BACK_WINDOW       10U     /* 允许回退窗口(防过冲卡死) */

/* ==================================================================
 * 9. 航向锁(原 yaw_lock.py)
 * ================================================================== */
#define PATH_YAW_TARGET_RAD           0.0f    /* 锁定目标:车头朝 world +y */
#define PATH_YAW_DEADZONE_DEG         1.0f
#define PATH_YAW_KP_SMALL             1.5f
#define PATH_YAW_KP_LARGE             3.0f
#define PATH_YAW_KP_THRES_DEG         5.0f
#define PATH_W_BASE_RAD_S             3.0f
#define PATH_W_SLOPE                  1.2f
#define PATH_W_MIN_RAD_S              0.3f

/* ==================================================================
 * 10. 激光兜底 / 横向微调(field.yaml: sensors + 原 profile_runner.py)
 * ================================================================== */
#define PATH_LASER_STOP_DIST_M        0.12f   /* 前激光 <12cm 强制 vx=0(停车,不横移) */
#define PATH_LASER_MAX_RANGE_M        0.20f   /* 真实 DT35 量程 5-20cm(固件钳位) */
/* 无回波行为(P1-3):若实测 DT35 无回波上报 0cm,置 1 把 0 视为超程无障碍;
 * 若实测无回波上报 20cm(钳位)则本开关无效。以台架实测为准。 */
#define PATH_LASER_NO_ECHO_FREE       1U
#define PATH_LASER_TIMEOUT_MS         500U    /* 与 dt35_pnp_link 的离线判据一致 */
#define PATH_STOP_ON_LASER_LOSS       1U      /* 前激光离线 -> 停车 */
/* 横向微调:err = laser_left - expected_left[i],叠加在底盘 x(向右)上 */
#define PATH_LAT_TRIM_KP              1.5f
#define PATH_LAT_TRIM_MAX_MS          0.30f
/* 注意: 符号待实测:底盘系 x=向右,离左墙太近(err<0)应向右修正 -> 取负号。
 *    若实车反而朝墙贴,把这里改成 +1.0f。 */
#define PATH_LAT_TRIM_SIGN            (-1.0f)
#define PATH_LAT_SAFE_M               0.10f   /* 左激光 <10cm 强制向右修正 */

/* 激光挂载(车体系: +x 前, +y 左) */
#define PATH_LASER_FRONT_X_M          0.225f
#define PATH_LASER_FRONT_Y_M          0.0f
#define PATH_LASER_LEFT_X_M           0.0f
#define PATH_LASER_LEFT_Y_M           0.175f

/* ==================================================================
 * 11. 指令 slew-rate 限幅(原关键设计原则)
 * ================================================================== */
#define PATH_SLEW_XY_ACCEL_MS2        2.0f
#define PATH_SLEW_W_ACCEL_RADS2       4.0f

#endif /* PATH_CONFIG_H */
