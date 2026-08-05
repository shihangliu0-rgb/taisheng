# Robocon 控制组嵌入式控制系统 —— 串口 485 IMU 驱动与卡尔曼滤波姿态/位置双重积分惯导模块 (V1.0)

本项目基于 **Robocon 控制组代码规范 V1.0** 制定，针对 **STM32F405RGTx + HAL 库 + 串口 485 高精度 IMU 传感器**，采用了精简实用的单 `app` 目录分层方案。

> **特别声明：** 
> 1. 本次代码重构**未对主片时钟（保持默认的 `HSI` 16MHz 内部 RC 时钟 -> 168MHz PLL）、底层 HAL 外设配置、引脚、DMA 及中断寄存器做任何变动**。
> 2. 按照需求，**已将之前所有的码盘编码器（`Encoder/`）、运动学解算（`Kinematics/`）、底盘里程计（`Chassis/`）以及冗余的其他业务分层全部彻底删除**，只保留最核心的 IMU 及自定义应用层代码。
> 3. **核心算法已与底盘硬件通信完全解耦**：所有姿态解算、滤波、ZUPT/ZARU 零速度更新与双重积分算法已单独放置在 **`Core/app/imu_algo.h/.c`** 中。

---

## 一、 核心架构与目录组织 (`Core/app/`)

为了摆脱过度的多级子文件夹分层，全系统所有**自写的应用、硬件驱动与进阶解算算法代码**全部统一放置于 **`Core/app/`** 文件夹中：

```text
Core/
├── app/
│   ├── my_main.c        # 用户自定义主程序入口 (初始化、循环调度与串口调试日志接口)
│   ├── my_main.h        # 主应用头文件及 LOG 宏定义
│   ├── imu.c            # IMU 串口6 DMA接收硬件层驱动、定时自标定与通信序列调度
│   ├── imu.h            # IMU 数据结构体、外部 API 及全部算法核心调参宏专区
│   ├── imu_algo.c       # 【独立算法】完整处理链与11层进阶惯导解算实现
│   └── imu_algo.h       # 【算法头文件】二状态卡尔曼、Butterworth、ZUPT与梯形双重积分声明
├── Inc/                 # STM32CubeMX 生成的基础头文件 (main.h, usart.h 等)
└── Src/                 # STM32CubeMX 生成的基础源文件 (main.c, usart.c 等)
```

---

## 二、 全系统解算链路与算法执行流程图

系统针对 **偏航角姿态 (`Yaw`)** 与 **线性运动双重积分 (`Pos / Vel / Acc`)** 建立了完整的实时惯导数据流与互补校正网络：

```text
===================================================================================================
                                      [IMU 串口 6 DMA 高频报文接收]
===================================================================================================
     │                                     │                                      │
     │ (0x02 角速度帧)                     │ (0x01 线性加速度帧)                  │ (0x03 偏航角帧)
     ▼                                     ▼                                      ▼
[原始 Z轴角速度 gyro_raw]         [原始线性加速度 acc_raw_x, acc_raw_y]           [绝对偏航角 yaw_raw]
     │                                     │                                      │
     │                                     │ (第 4 层) 异常绝对量速检查            │
     ▼                                     ▼                                      │
异常阶跃速度检验 (>2000°/s剔除)     (第 4 层) 二阶 Butterworth 低通滤波           │
     │                                (20~30Hz截止频率，滤除高频电机震动)         │
     ▼                                     │                                      │
角速度自适应一阶滤波                        ▼                                      │
(静止 alpha=0.02, 运动 alpha=0.3)  (第 7 层) 100 帧滑动窗口振动判断                │
     │                                (平均值 mean 与 标准差 std_acc)             │
     ▼                                     │                                      │
100 帧滑动窗口静止检测                     │                                      │
     │                             ┌───────┴───────────────┐                      │
     ▼                             ▼                       ▼                      │
(第 6 层) ZARU 零角速度更新      [判断为车体驻停]          [判断为车体运动]            │
指数平滑收敛陀螺仪零漂 bias       (std_acc < 0.08)          (std_acc >= 0.08)         │
     │                             │                       │                      │
     │                             │                       │ (第 3 层) 姿态转换    │
     ▼                             │                       ▼                      │
角速度去零偏梯形积分               │                 去除在线加速度零偏 acc_bias  │
     │                             │                       │                      │
     ▼                             ▼                       ▼                      │
(微秒级时间片 dt)              (第 5 层) ZUPT 零速度更新  依据解算出的【Yaw角】将机体   │
基于 Cortex-M4 DWT 周期计数器  直接硬性限位 vx=0, vy=0    加速度正交变换至世界坐标系     │
     │                             │                 [acc_world_x, acc_world_y]   │
     ▼                             │                       │                      │
二状态 [yaw, bias]^T 卡尔曼        │                       ▼                      │
协方差矩阵状态预测                  │                 (第 11 层) 差速直行底盘       │
     │                             │                 地面侧滑阻尼运动学模型约束      │
     │                             │                       │                      │
     │                             │                       ▼                      │
     │                             │                 (第 8 层) 第一次梯形速度积分   │
     │                             │                 0.5*(a_now+a_last)*dt        │
     │                             │                       │                      │
     │                             │                       ▼                      │
     │                             │                 (第 8 层) 第二次梯形位置积分   │
     │                             │                 0.5*(v_now+v_last)*dt        │
     │                             │                       │                      │
     │                             │                       ▼                      │
     │                             ▼                 当前位置与线速度状态          │
     │                       (第 2 层) 加速度零偏           [pos_x, pos_y]            │
     │                       acc_bias 在线平滑收敛         [vel_x, vel_y]             │
     │                             │                       │                      │
     │                             └──────────┬────────────┘                      │
     │                                        ▼                                   │
     │                              (第 9 层 & 第 10 层)                          │
     │                              三状态 [pos, vel, acc_bias]^T                 │
     │                              卡尔曼误差协方差矩阵 P 预测                    │
     │                                        │                                   │
     │                                        ▼                                   │
     │                              当驻停触发 ZUPT 时执行                         │
     │                              z_vel = 0.0 闭环测量更正与漂移修正              │
     │                                        │                                   │
     │                                        ▼                                   │
     │                             =======================                        │
     │                             【实时三轴正交运动输出】                       │
     │                              Pos X/Y, Vel X/Y, AccW                        │
     │                             =======================                        │
     │                                                                            │
     └────────────────────────────────┬───────────────────────────────────────────┘
                                      ▼
                            【卡尔曼绝对偏航测量闭环校正】
                             融合角度与首帧相对零位修正
                             同步校准 Yaw 角度与陀螺仪 Bias
                                      │
                                      ▼
                            =======================
                            【实时最优偏航姿态角输出】
                             Yaw_deg (-180 ~ 180°)
                            =======================
===================================================================================================
```

---

## 三、 十大惯导算法技术解析与工程提示

1. **第 1 层：IMU 数据处理流（IMU Processing Chain）**  
   按照真正的惯性导航标准建立了严格的前后依赖处理：**线性加速度采集 $\to$ 二阶低通除噪 $\to$ 零偏跟踪去漂移 $\to$ 二维坐标系姿态正交变换 $\to$ 重力/偏置消除 $\to$ 世界线性加速度 $\to$ 速度积分 $\to$ 位置积分**。
2. **第 2 层：加速度零偏在线估计（Acc Bias Online Tracking）**  
   不同环境温度下，加速度计静止时不严格等于 `0`。我们在车体停止的 ZUPT 期间，将余重偏置在线跟踪：
   $$ a_{bias} = (1 - \alpha) \cdot a_{bias} + \alpha \cdot a_{filt} $$
3. **第 3 层：姿态空间变换与去偏（Rotation Compensation）**  
   使用滤波得到的实时 **`Yaw_deg`** 将去零漂后的机体加速度投影至现场正交大地系：
   $$ A_X = a_{body,x} \cos\psi - a_{body,y} \sin\psi, \quad A_Y = a_{body,x} \sin\psi + a_{body,y} \cos\psi $$
4. **第 4 层：二阶 Butterworth 低通滤波（Biquad LPF）**  
   为抑制差速驱动轮强劲的高频谐波振动，采用 20~30Hz 截止频率的双二阶 Butterworth 差分滤波器，具有 40dB/dec 的急剧高频衰减，彻底消除积分高频抖动。
5. **第 5 层：零速度更新 ZUPT（Zero Velocity Update）**  
   **不用任何外部编码器、纯靠 IMU 实现驻停检测与自修复**！在识别静止后：
   - 将线速度硬性归零 `vx = 0.0, vy = 0.0`，从根本上直接掐死停止滑行积漂；
   - 联动 3 状态卡尔曼执行观测值为 `z = 0.0` 的误差更正。
6. **第 6 层：零角速度更新 ZARU（Zero Angular Rate Update）**  
   同 ZUPT 原理，静止窗内判定当前角速度均为芯片底噪和漂移，平滑收敛更新陀螺仪零位。
7. **第 7 层：滑动窗口振动判断（Sliding Window std_acc）**  
   采用最近 100 帧数据的滑动均值与标准差 $\sigma$，杜绝仅凭单帧瞬时数据把被碰撞或抖动误判为静止。
8. **第 8 层：非线性梯形双重积分（Trapezoidal Double Integration）**  
   速度使用 `0.5 * (a_now + a_last) * dt`；位置使用 `0.5 * (v_now + v_last) * dt`。
9. **第 9 层 & 第 10 层：卡尔曼与 ESKF 误差协方差体系（3-State KF & ESKF Note）**  
   在 $X$ 与 $Y$ 双向建立了独立的三状态 $[pos, vel, a_{bias}]^T$ 协方差估计器。
   > **工程提示（不可观测性指正）**：由于二阶积分是 $t^2$ 放大，纯 IMU 没有外部绝对参考观测（如码盘、UWB或视觉）时，KF 只具去噪和 ZUPT 期间重置作用；机器人在长时间长距离持续运动中的慢漂移是惯性导航的物理规律，无法仅靠 IMU 单芯片清除。
10. **第 11 层：地面机器人非全向侧滑约束（Non-Holonomic Constraint）**  
    针对普通差速轮与直行车体，默认给机体侧向速度极强阻尼；如果底盘使用麦克纳姆轮或全向轮横向移动，请在 `imu.h` 中将 `IMU_ALGO_NONHOLONOMIC_CONSTRAINT` 改为 `0U`。

---

## 四、 调参专区与参数释义 (`Core/app/imu.h`)

所有卡尔曼过程方差、滤波器系数、ZUPT 门槛均已作为全大写宏集中放置在 **`Core/app/imu.h`**：

```c
/* ================= 姿态偏航角与陀螺仪调参专区 ================= */
#define IMU_ALGO_Q_YAW                  0.001f    // 偏航角卡尔曼过程方差
#define IMU_ALGO_Q_BIAS                 0.00001f  // 零漂自标定跟踪方差
#define IMU_ALGO_R_YAW                  3.0f      // 角度传感器噪声信度
#define IMU_ALGO_GYRO_ABS_LIMIT         2000.0f   // 异常速度范围限值 (°/s)
#define IMU_ALGO_GYRO_STEP_LIMIT        500.0f    // 单帧阶跃飞线过滤阈值
#define IMU_ALGO_GYRO_MOTION_THRESHOLD  5.0f      // 静止与快转自适应分界
#define IMU_ALGO_GYRO_ALPHA_STATIC      0.02f     // 静止去噪强滤波 alpha
#define IMU_ALGO_GYRO_ALPHA_MOTION      0.30f     // 快速响应弱滤波 alpha
#define IMU_ALGO_STAT_MEAN_THRESHOLD    0.20f     // 静止在线跟踪平均值上限
#define IMU_ALGO_STAT_STD_THRESHOLD     0.05f     // 静止在线跟踪标准差上限
#define IMU_ALGO_BIAS_DYNAMIC_ALPHA     0.01f     // 角速度零偏防震荡指数

/* ================== 线性加速度、ZUPT与位置双重积分调参专区 ================== */
#define IMU_ALGO_ACC_LPF_B0             0.097631f // 默认二阶 Butterworth 25Hz系数
#define IMU_ALGO_ACC_LPF_B1             0.195262f
#define IMU_ALGO_ACC_LPF_B2             0.097631f
#define IMU_ALGO_ACC_LPF_A1             -0.942809f
#define IMU_ALGO_ACC_LPF_A2             0.333333f
#define IMU_ALGO_ACC_ABS_LIMIT          100.0f    // 线性加速度超限保护 (m/s²)
#define IMU_ALGO_ZUPT_GYRO_THRESHOLD    0.25f     // ZUPT 触动角速度上限 (°/s)
#define IMU_ALGO_ZUPT_ACC_STD_THRESHOLD 0.08f     // ZUPT 加速度 100帧平稳标准差门槛
#define IMU_ALGO_ACC_BIAS_ALPHA         0.001f    // 加速度零漂估计跟踪平滑系数
#define IMU_ALGO_Q_POS                  0.0001f   // 三状态卡尔曼-位置信度
#define IMU_ALGO_Q_VEL                  0.005f    // 三状态卡尔曼-速度信度
#define IMU_ALGO_Q_ACC_BIAS             0.00002f  // 三状态卡尔曼-零飘漂移速度
#define IMU_ALGO_R_ZUPT                 0.01f     // ZUPT v=0 测量反弹力度
#define IMU_ALGO_NONHOLONOMIC_CONSTRAINT 1U       // 地面侧滑阻尼开关
```

---

## 五、 对外接口与如何调用 (`Core/app/imu.h`)

你可以在 `main.c` 或 `my_main.c` 中直接调用以下函数获取全部去偏和修正好的物理量：

```c
#include "imu.h"
#include "my_main.h"

void ReadImuCompleteState(void)
{
    /* 1. 偏航姿态角与陀螺仪角速度 */
    float yaw   = Imu_GetYaw();        // 最优偏航角 (-180~180°)
    float gyroZ = Imu_GetGyroZ();      // 去除零偏与滤波后 Z 轴角速度
    float biasZ = Imu_GetBias();       // 在线跟踪出的 Z 轴角速度零偏

    /* 2. 前向与横向位置、速度与加速度 (大地正交系) */
    float posX   = Imu_GetPosX();      // 世界坐标 X 轴二次积分累计距离 (m)
    float posY   = Imu_GetPosY();      // 世界坐标 Y 轴二次积分累计距离 (m)
    float velX   = Imu_GetVelX();      // 梯形积分+ZUPT纠正的 X 轴速度 (m/s)
    float velY   = Imu_GetVelY();      // 梯形积分+ZUPT纠正的 Y 轴速度 (m/s)
    float accW_X = Imu_GetAccWorldX(); // 姿态空间旋转去偏后的 X 轴世界加速度 (m/s²)
    float accW_Y = Imu_GetAccWorldY(); // 姿态空间旋转去偏后的 Y 轴世界加速度 (m/s²)
    uint8_t zupt = Imu_IsZuptActive(); // 当前车体是否驻停触发 ZUPT 零速度锁定 (1:驻停, 0:运动)

    /* 3. 在开机或者定位点校零时，一键清空位置坐标与速度 */
    // Imu_ResetPosition(); 
}
```
