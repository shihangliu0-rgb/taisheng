/* path_config.h - 场地与路径参数(改参数只改这里) */
#ifndef PATH_CONFIG_H
#define PATH_CONFIG_H

#define PATH_PI                     3.14159265f   /* pi(float) */

#define PATH_ENABLE                   1U     /* 1 = 启用路径规划与控制 */
#define PATH_CONTROL_PERIOD_MS        5U     /* 控制周期,默认 200 Hz */
#define PATH_MAX_RUN_MS               30000U /* 全程超时(位姿<10Hz 时限速,留足时间) */

/* 调试打印:默认关闭 */
#define PATH_DEBUG                    0U
/* #define PATH_DEBUG_UART_HANDLE huart8 */
#define PATH_DEBUG_PERIOD_MS          400U

#define PATH_WHEEL_RADIUS_M           0.050f
#define PATH_RPM_PER_M_S              190.99f
#define PATH_CHASSIS_ROT_ARM_M        0.34f
#define PATH_Z_PER_RAD_S              9.549f

/* 按用户实测信息更新(2026-08 场地 */
#define PATH_FIELD_W_M                3.0f    /* 真实场地:东墙在 x=3.0(用户提供) */
#define PATH_FIELD_H_M                6.0f
/* 位姿合法性范围(融合前校验) */
#define PATH_POSE_X_MIN_M             (-0.5f)
#define PATH_POSE_X_MAX_M             (PATH_FIELD_W_M + 0.5f)
#define PATH_POSE_Y_MIN_M             (-0.5f)
#define PATH_POSE_Y_MAX_M             (PATH_FIELD_H_M + 0.5f)
#define PATH_WALL_THICKNESS_M         0.049f
#define PATH_WALL_COUNT               7U

/* 4 外墙 + 3 内墙 */
#define PATH_WALLS_TABLE \
    {0.000f, 0.000f, 3.000f, 0.049f},   /* south 下墙(真实场地宽 3m) */ \
    {0.000f, 5.951f, 3.000f, 6.000f},   /* north 上墙 */ \
    {0.000f, 0.000f, 0.049f, 6.000f},   /* west  左墙 */ \
    {2.951f, 0.000f, 3.000f, 6.000f},   /* east  东墙(用户:x=3m 就是东墙) */ \
    {1.05f, 1.07f, 3.00f, 1.12f},       /* 墙1(南) */ \
    {0.000f, 2.075f, 2.000f, 2.125f},   /* 墙B(东端x=2.0,反推) */ \
    {1.05f, 3.075f, 3.00f, 3.125f},       /* 墙C(x 范围同墙1:1.05~3.00,用户实测) */

#define PATH_ROBOT_LENGTH_M           0.617f
#define PATH_ROBOT_WIDTH_M            0.44f
/* 软膨胀余量(仅用于整形):0.04m */
#define PATH_SAFETY_MARGIN_M          0.04f
/* 路径膨胀:yaw 锁定(±2°)下机器人始终轴对齐 */
#define PATH_INFLATE_DX_M             (0.5f * PATH_ROBOT_WIDTH_M + PATH_SAFETY_MARGIN_M)   /* 0.30 */
#define PATH_INFLATE_DY_M             (0.5f * PATH_ROBOT_LENGTH_M + PATH_SAFETY_MARGIN_M)  /* 0.3885 */
/* 硬膨胀(轨迹验收用):机器人真实外廓 + 1.5cm */
#define PATH_HARD_MARGIN_M            0.015f
#define PATH_SEGMENT_CUT_EPS_M        0.002f

#define PATH_WAYPOINT_COUNT           96U    /* 路由工作容量(长路线加密后 >64) */
/* 路由直线段加密步长:长腿+少点的弦长参数化会让样条在拐角折叠
 * (kappa 数百,仿真复现);加密后节点均匀,拐角自然圆滑 */
#define PATH_ROUTE_STEP_M             0.15f
#define PATH_WAYPOINTS_TABLE \
    {0.50f, 1.00f},   /* 名义起点占位:上电后由小电脑实测位姿整体覆盖 */ \
    {0.50f, 1.65f},   /* 先北上到通道1 高度:墙1 西端 x=1.05 */ \
    {1.00f, 1.65f},   /* 通道1 左入口 */ \
    {2.00f, 1.65f},   /* D角圆弧起点(用户给定;-90 度) */ \
    {2.12f, 1.67f},   /* -75 度 */ \
    {2.24f, 1.71f},   /* -60 度 */ \
    {2.34f, 1.79f},   /* -45 度 */ \
    {2.41f, 1.89f},   /* -30 度 */ \
    {2.46f, 2.00f},   /* -15 度 */ \
    {2.48f, 2.13f},   /* 0度(顶点) */ \
    {2.46f, 2.25f},   /* +15 度 */ \
    {2.41f, 2.36f},   /* +30 度 */ \
    {2.34f, 2.46f},   /* +45 度 */ \
    {2.24f, 2.54f},   /* +60 度 */ \
    {2.12f, 2.58f},   /* +75 度 */ \
    {2.00f, 2.60f},   /* D角圆弧终点 */ \
    {1.00f, 2.60f},   /* 通道2 左段 */ \
    {0.375f, 2.60f},  /* 墙C缺口入口 */ \
    {0.36f, 2.66f},   /* 拐角过渡点:强制样条贴西侧绕 90 度角 */ \
    {0.36f, 3.70f},   /* 缺口列直行北上 */ \
    {0.50f, 3.70f}     /* 目标点 */
    /* D 角圆弧:半圆 R=0.475 */

#define PATH_GOAL_X_M                 0.50f
#define PATH_GOAL_Y_M                 3.70f
#define PATH_ARRIVE_TOL_M             0.08f   /* 融合判停阈值;物理停点=阈值+制动(~2cm)+滞后(~3cm) <= 0.15 规格 */
/* 起点=小电脑实测位姿 */
/* 起步朝向 ±30° */
#define PATH_START_YAW_LIMIT_DEG      30.0f
/* CALIB 总超时 */
#define PATH_CALIB_TIMEOUT_MS         10000U
/* 故障去抖(P0-3):连续故障累计超窗口才停机,单次抖动不误停 */
#define PATH_FAULT_IMU_MS             200U
#define PATH_FAULT_MOTOR_MS           400U
#define PATH_FAULT_LASER_MS           1500U   /* 下位板 I2C 重试 1000ms */
/* 瞬态故障停机后,输入连续健康达此时长重新布防(限次数) */
#define PATH_RECOVER_MS               1000U
#define PATH_RECOVER_MAX_TIMES        3U

#define PATH_SPLINE_DEGREE            3U
#define PATH_SPLINE_SAMPLES           300U
/* 把落入膨胀墙的采样点沿距离场梯度外推 */
#define PATH_PUSH_STEP_M              0.02f
#define PATH_PUSH_MAX_ITERS           60U
/* 推离 + 拉普拉斯平滑交替迭代轮数 */
#define PATH_PUSH_SMOOTH_ROUNDS       0U
/* 弯道半径:不做人为整形 */
#define PATH_SAMPLE_STEP_MAX_M        0.15f   /* 最大采样点间距 */
#define PATH_MIN_CLEARANCE_M          0.02f   /* 中心路径到硬膨胀墙的最小净距 */
#define PATH_KAPPA_HARD_MAX           50.0f   /* κ硬上限 */
#define PATH_LAT_ACC_TOL              1.20f   /* 横向加速度超限容差 */
#define PATH_BUILD_MAX_ATTEMPTS       3U      /* 整形+验收的最大尝试次数 */
#define PATH_REQUIRE_MOTORS           1U      /* 任一电机离线 -> 停车 */

#define PATH_V_MAX_MS                 1.5f
#define PATH_V_START_MS               0.0f
#define PATH_V_GOAL_MS                0.30f
#define PATH_V_MIN_MS                 0.20f
#define PATH_A_LAT_MAX                1.5f
#define PATH_A_LON_ACCEL              1.2f
#define PATH_A_LON_BRAKE              1.8f
/* 剖面反向扫描使用的"跟踪刹车斜率" */
#define PATH_PROFILE_BRAKE_MS2        1.6f
/* 曲率限速的弧长前视窗 */
#define PATH_CURV_LOOKAHEAD_M         0.8f
#define PATH_KAPPA_MIN                0.01f   /* 曲率下限,防止除零 */

#define PATH_FUSION_XY_GATE_M         0.15f   /* 单帧 xy 跳变门限(>15cm 拒绝 */
#define PATH_FUSION_YAW_GATE_DEG      20.0f   /* yaw 与预测差 >20° 拒绝 */
#define PATH_FUSION_YAW_GAIN          0.15f   /* yaw 低通拉回增益 */
#define PATH_FUSION_UPPER_TIMEOUT_MS  500U    /* 链路丢失判定(CRC 有效帧刷新) */
#define PATH_FUSION_CALIB_SAMPLES     200U    /* 静止标定采样帧数(约1s) */
/* 数据可用性 */
#define PATH_UPPER_DEGRADE_MS         150U   /* 数据年龄超此即降速 */
/* 重新捕获 */
#define PATH_FUSION_REACQ_MS          300U
/* 位姿延迟补偿(P0-2):按此延迟用指令速度前推测量值(实测标定填真值) */
#define PATH_POSE_LATENCY_MS          120U   /* 实测标定后填真值 */
/* yaw 重捕获(P0-1):连续一致偏出门限达此帧数,强制整帧覆盖 yaw */
#define PATH_FUSION_YAW_REACQ_N       15U
#define PATH_UPPER_DEGRADE_V_MS       0.30f
#define PATH_UPPER_DATA_STOP_MS       800U
#define PATH_GYRO_SIGN                1.0f    /* IMU z 轴与 yaw 反向时改 -1 */

#define PATH_LD_MIN_M                 0.08f
#define PATH_LD_K_S                   0.06f
/* 曲率自适应前视上限:急弯处缩短前视距离 */
#define PATH_LD_KAPPA_MAX_M           0.08f
/* 前视下限:高曲率拐角处前视若被压到点间距以下,纯追踪目标会在
 * 相邻点间振荡,车原地打转(仿真复现);下限 6cm > 2x点间距 3cm */
#define PATH_LD_CAP_MIN_M              0.06f
#define PATH_SEARCH_WINDOW            150U    /* 前向最近点搜索窗口 */
#define PATH_SEARCH_BACK_WINDOW       40U     /* 回退窗口(过冲后找回最近点) */

#define PATH_YAW_TARGET_RAD           0.0f    /* 锁定目标:车头朝 world +y */
#define PATH_YAW_DEADZONE_DEG         1.0f
#define PATH_YAW_KP_SMALL             1.5f
#define PATH_YAW_KP_LARGE             3.0f
#define PATH_YAW_KP_THRES_DEG         5.0f
#define PATH_W_BASE_RAD_S             3.0f
#define PATH_W_SLOPE                  1.2f
#define PATH_W_MIN_RAD_S              0.3f

#define PATH_LASER_STOP_DIST_M        0.12f   /* 前激光 <12cm 强制停车 */
#define PATH_LASER_MAX_RANGE_M        0.20f   /* 真实 DT35 量程 5-20cm(固件钳位) */
/* 期望墙门控 */
#define PATH_LASER_EXPECTED_MARGIN_M  0.04f
#define PATH_LASER_RECOVERY_V_MS      0.25f
/* 期望墙表射程(须大于激光量程,否则期望值全被钳到 0.20 门控恒真) */
#define PATH_EXPECTED_FRONT_RANGE_M   0.5f
#define PATH_EXPECTED_SIDE_RANGE_M    0.5f
#define PATH_LASER_LATERAL_DIR_MAX    0.60f   /* 目标方向车体纵向分量阈值 */
/* 无回波=0 视为超程 */
#define PATH_LASER_NO_ECHO_FREE       1U
#define PATH_LASER_TIMEOUT_MS         500U    /* 与 dt35_pnp_link 的离线判据一致 */
#define PATH_STOP_ON_LASER_LOSS       1U      /* 前激光离线 -> 停车 */

#define PATH_LAT_TRIM_ENABLE          0U      /* 左激光微调默认关(量程门控失真) */
#define PATH_LAT_TRIM_KP              1.5f
#define PATH_LAT_TRIM_MAX_MS          0.30f
/* 横向微调符号 */
#define PATH_LAT_TRIM_SIGN            (-1.0f)
#define PATH_LAT_SAFE_M               0.10f   /* 左激光 <10cm 强制向右修正 */

/* 激光挂载(车体系: +x 前, +y 左) */
#define PATH_LASER_FRONT_X_M          0.225f
#define PATH_LASER_FRONT_Y_M          0.0f
#define PATH_LASER_LEFT_X_M           0.0f
#define PATH_LASER_LEFT_Y_M           0.175f

#define PATH_SLEW_XY_ACCEL_MS2        2.0f
#define PATH_SLEW_W_ACCEL_RADS2       4.0f

#endif /* PATH_CONFIG_H */
