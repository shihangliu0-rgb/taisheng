#include "chassis_main.h"

#include "fdcan.h"
#include "imu_main.h"

#include <stddef.h>

#define ARRAY_SIZE(array) ((uint8_t)(sizeof(array) / sizeof((array)[0])))

/* 底盘 VESC 硬件参数。 */
#define CHASSIS_MOTOR_LF_ID       87U
#define CHASSIS_MOTOR_RF_ID       43U
#define CHASSIS_MOTOR_LR_ID       67U
#define CHASSIS_MOTOR_RR_ID       59U
#define CHASSIS_MOTOR_POLE_PAIRS  21U
#define CHASSIS_MAX_RPM           4000
#define CHASSIS_MIN_RPM           0
#define CHASSIS_BRAKE_CURRENT_A   10.0f

/* 底盘控制周期和斜坡时间。 */
#define CHASSIS_COMMAND_PERIOD_MS  10U
#define CHASSIS_START_RAMP_MS      300U
#define CHASSIS_STOP_RAMP_MS       200U
#define CHASSIS_RX_DRAIN_MAX       32U

#define CHASSIS_ROTATION_SCALE     (3.5f + 3.30f)

typedef enum
{
    CHASSIS_MOTION_STOPPED,
    CHASSIS_MOTION_STARTING,
    CHASSIS_MOTION_RUNNING,
    CHASSIS_MOTION_STOPPING,
    CHASSIS_MOTION_EMERGENCY_STOP
} chassis_motion_state_t;

static const vesc_motor_config_t chassis_motor_cfg[] =
{
    {
        .id = CHASSIS_MOTOR_LF_ID,
        .pole_pairs = CHASSIS_MOTOR_POLE_PAIRS,
        .min_rpm = CHASSIS_MIN_RPM,
        .max_rpm = CHASSIS_MAX_RPM,
        .brake_current_a = CHASSIS_BRAKE_CURRENT_A
    },
    {
        .id = CHASSIS_MOTOR_RF_ID,
        .pole_pairs = CHASSIS_MOTOR_POLE_PAIRS,
        .min_rpm = CHASSIS_MIN_RPM,
        .max_rpm = CHASSIS_MAX_RPM,
        .brake_current_a = CHASSIS_BRAKE_CURRENT_A
    },
    {
        .id = CHASSIS_MOTOR_LR_ID,
        .pole_pairs = CHASSIS_MOTOR_POLE_PAIRS,
        .min_rpm = CHASSIS_MIN_RPM,
        .max_rpm = CHASSIS_MAX_RPM,
        .brake_current_a = CHASSIS_BRAKE_CURRENT_A
    },
    {
        .id = CHASSIS_MOTOR_RR_ID,
        .pole_pairs = CHASSIS_MOTOR_POLE_PAIRS,
        .min_rpm = CHASSIS_MIN_RPM,
        .max_rpm = CHASSIS_MAX_RPM,
        .brake_current_a = CHASSIS_BRAKE_CURRENT_A
    }
};

static vesc_can_t chassis_vesc_bus;
static vesc_motor_t chassis_motors[ARRAY_SIZE(chassis_motor_cfg)];
static uint32_t chassis_last_tx_ms;
static uint32_t chassis_ramp_begin_ms;
static int32_t chassis_stop_start_rpm[CHASSIS_WHEEL_COUNT];
static bool chassis_ready;
static volatile bool chassis_emergency_stop;

volatile int16_t chassis_target_vx = 0;
volatile int16_t chassis_target_vy = 0;
volatile int16_t chassis_target_z = 0;
chassis_motion_state_t chassis_motion_state = CHASSIS_MOTION_STOPPED;

/**
 * @brief 处理底盘 VESC 反馈并更新在线状态
 * @param now_ms 当前系统时间(ms)
 * @retval None
 */
static void Chassis_ProcessFeedback(uint32_t now_ms)
{
    vesc_can_msg_t msg;
    uint8_t drain_count = 0U;
    uint8_t i;

    VescCan_Recover(&chassis_vesc_bus);
    while (drain_count < CHASSIS_RX_DRAIN_MAX)
    {
        HAL_StatusTypeDef status = VescCan_Read(&chassis_vesc_bus, &msg);

        if (status == HAL_BUSY)
        {
            break;
        }
        drain_count++;
        if (status != HAL_OK)
        {
            continue;
        }

        for (i = 0U; i < ARRAY_SIZE(chassis_motors); i++)
        {
            if (VescMotor_Parse(&chassis_motors[i], &msg, now_ms))
            {
                break;
            }
        }
    }

    for (i = 0U; i < ARRAY_SIZE(chassis_motors); i++)
    {
        VescMotor_Update(&chassis_motors[i], now_ms);
    }
}

/**
 * @brief 发送四个车轮的当前目标转速
 * @retval None
 */
static void Chassis_SendWheelRpm(void)
{
    uint8_t i;

    for (i = 0U; i < ARRAY_SIZE(chassis_motors); i++)
    {
        (void)VescMotor_SendRpm(&chassis_motors[i]);
    }
}

/**
 * @brief 整组设置四个车轮的目标转速
 * @param target_rpm 四个车轮的目标转速(RPM)
 * @retval HAL 状态
 */
static HAL_StatusTypeDef Chassis_SetWheelRpm(
    const int32_t target_rpm[CHASSIS_WHEEL_COUNT])
{
    uint8_t i;

    for (i = 0U; i < ARRAY_SIZE(chassis_motors); i++)
    {
        if (VescMotor_SetRpm(&chassis_motors[i], target_rpm[i]) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

/**
 * @brief 按统一比例缩放一个转速，正负值采用对称四舍五入
 * @param value 原始转速(RPM)
 * @param numerator 比例分子
 * @param denominator 比例分母
 * @retval 缩放后的转速(RPM)
 */
static int32_t Chassis_ScaleRpm(int32_t value, uint32_t numerator,
                                uint32_t denominator)
{
    int64_t scaled;

    if (numerator >= denominator)
    {
        return value;
    }

    scaled = (int64_t)value * (int64_t)numerator;
    if (scaled >= 0)
    {
        scaled += (int64_t)denominator / 2;
    }
    else
    {
        scaled -= (int64_t)denominator / 2;
    }
    return (int32_t)(scaled / (int64_t)denominator);
}

/**
 * @brief 四轮同比例限幅，保持全向轮混合控制方向不变
 * @param wheel_rpm 四个车轮的目标转速(RPM)
 * @retval None
 */
static void Chassis_LimitWheelRpm(
    int32_t wheel_rpm[CHASSIS_WHEEL_COUNT])
{
    int64_t scaled;
    uint32_t max_magnitude = 0U;
    uint8_t i;

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        int64_t value = wheel_rpm[i];
        uint32_t magnitude = (uint32_t)((value < 0) ? -value : value);

        if (magnitude > max_magnitude)
        {
            max_magnitude = magnitude;
        }
    }

    if (max_magnitude <= (uint32_t)CHASSIS_MAX_RPM)
    {
        return;
    }

    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        scaled = (int64_t)wheel_rpm[i] * (int64_t)CHASSIS_MAX_RPM;
        if (scaled >= 0)
        {
            scaled += (int64_t)max_magnitude / 2;
        }
        else
        {
            scaled -= (int64_t)max_magnitude / 2;
        }
        wheel_rpm[i] = (int32_t)(scaled / (int64_t)max_magnitude);
    }
}

/**
 * @brief 计算四轮混控并施加统一的启动比例
 * @param vx X 方向目标速度
 * @param vy Y 方向目标速度
 * @param z Z 轴目标旋转速度
 * @param ramp_ms 启动斜坡已经运行的时间(ms)
 * @retval HAL 状态
 */
static HAL_StatusTypeDef Chassis_ApplyMotion(int16_t vx, int16_t vy,
                                             int16_t z,
                                             uint32_t ramp_ms)
{
    int32_t rotation;
    int32_t wheel_rpm[CHASSIS_WHEEL_COUNT];
    uint8_t i;

    // 机器人坐标系：X 向右、Y 向前、Z 逆时针为正。
    rotation = (int32_t)((float)z * CHASSIS_ROTATION_SCALE);
    wheel_rpm[CHASSIS_WHEEL_LF] = vx + vy + rotation;
    wheel_rpm[CHASSIS_WHEEL_RF] = vx - vy + rotation;
    wheel_rpm[CHASSIS_WHEEL_LR] = vx - vy - rotation;
    wheel_rpm[CHASSIS_WHEEL_RR] = vx + vy - rotation;

    Chassis_LimitWheelRpm(wheel_rpm);
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
    {
        wheel_rpm[i] = Chassis_ScaleRpm(wheel_rpm[i], ramp_ms,
                                        CHASSIS_START_RAMP_MS);
    }
    return Chassis_SetWheelRpm(wheel_rpm);
}

/**
 * @brief 初始化 FDCAN1 和四台底盘 VESC
 * @retval HAL 状态
 */
HAL_StatusTypeDef Chassis_Init(void)
{
    uint8_t i;

    if (chassis_ready)
    {
        return HAL_OK;
    }
    if (VescCan_Init(&chassis_vesc_bus, &hfdcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < ARRAY_SIZE(chassis_motors); i++)
    {
        if (VescMotor_Init(&chassis_motors[i], &chassis_vesc_bus,
                           &chassis_motor_cfg[i]) != HAL_OK)
        {
            VescCan_Stop(&chassis_vesc_bus);
            return HAL_ERROR;
        }
    }

    chassis_motion_state = CHASSIS_MOTION_STOPPED;
    chassis_last_tx_ms = HAL_GetTick();
    chassis_ready = true;
    Chassis_SendWheelRpm();
    return HAL_OK;
}

/**
 * @brief 更新底盘三轴目标速度
 * @param vx X 方向目标速度，正方向向右
 * @param vy Y 方向目标速度，正方向向前
 * @param z Z 轴目标旋转速度，正方向为逆时针
 * @retval HAL 状态
 */
HAL_StatusTypeDef Chassis_SetVelocity(int16_t vx, int16_t vy, int16_t z)
{
    uint32_t primask;

    if (!chassis_ready)
    {
        return HAL_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    chassis_target_vx = vx;
    chassis_target_vy = vy;
    chassis_target_z = z;
    if ((vx != 0) || (vy != 0) || (z != 0))
    {
        chassis_emergency_stop = false;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return HAL_OK;
}

/**
 * @brief 执行底盘反馈、状态机、运动学和 CAN 发送
 * @retval None
 */
void Chassis_Run1ms(void)
{
    int32_t wheel_rpm[CHASSIS_WHEEL_COUNT];
    uint32_t now_ms;
    uint32_t elapsed_ms;
    uint32_t remaining_ms;
    uint32_t primask;
    int16_t vx;
    int16_t vy;
    int16_t z;
    bool emergency_stop;
    bool motion_requested;
    bool send_now = false;
    uint8_t i;

    if (!chassis_ready)
    {
        return;
    }

    now_ms = HAL_GetTick();
    Chassis_ProcessFeedback(now_ms);

    primask = __get_PRIMASK();
    __disable_irq();
    vx = chassis_target_vx;
    vy = chassis_target_vy;
    z = chassis_target_z;
    emergency_stop = chassis_emergency_stop;
    if (primask == 0U)
    {
        __enable_irq();
    }
    motion_requested = (vx != 0) || (vy != 0) || (z != 0);

    if (emergency_stop)
    {
        if (chassis_motion_state != CHASSIS_MOTION_EMERGENCY_STOP)
        {
            for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
            {
                wheel_rpm[i] = 0;
            }
            (void)Chassis_SetWheelRpm(wheel_rpm);
            chassis_motion_state = CHASSIS_MOTION_EMERGENCY_STOP;
            send_now = true;
        }
    }
    else
    {
        switch (chassis_motion_state)
        {
        case CHASSIS_MOTION_STOPPED:
            if (motion_requested)
            {
                chassis_ramp_begin_ms = now_ms;
                chassis_motion_state = CHASSIS_MOTION_STARTING;
            }
            else
            {
                z = ImuMain_CalcOmega(0, 0, 0);
                (void)Chassis_ApplyMotion(0, 0, z,
                                          CHASSIS_START_RAMP_MS);
            }
            break;

        case CHASSIS_MOTION_STARTING:
            if (!motion_requested)
            {
                chassis_ramp_begin_ms = now_ms;
                for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
                {
                    chassis_stop_start_rpm[i] =
                        chassis_motors[i].status.target_rpm;
                }
                chassis_motion_state = CHASSIS_MOTION_STOPPING;
                break;
            }

            elapsed_ms = now_ms - chassis_ramp_begin_ms;
            z = ImuMain_CalcOmega(vx, vy, z);
            if (elapsed_ms >= CHASSIS_START_RAMP_MS)
            {
                elapsed_ms = CHASSIS_START_RAMP_MS;
                chassis_motion_state = CHASSIS_MOTION_RUNNING;
            }
            (void)Chassis_ApplyMotion(vx, vy, z, elapsed_ms);
            break;

        case CHASSIS_MOTION_RUNNING:
            if (!motion_requested)
            {
                chassis_ramp_begin_ms = now_ms;
                for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
                {
                    chassis_stop_start_rpm[i] =
                        chassis_motors[i].status.target_rpm;
                }
                chassis_motion_state = CHASSIS_MOTION_STOPPING;
            }
            else
            {
                z = ImuMain_CalcOmega(vx, vy, z);
                (void)Chassis_ApplyMotion(vx, vy, z,
                                          CHASSIS_START_RAMP_MS);
            }
            break;

        case CHASSIS_MOTION_STOPPING:
            if (motion_requested)
            {
                chassis_ramp_begin_ms = now_ms;
                chassis_motion_state = CHASSIS_MOTION_STARTING;
                z = ImuMain_CalcOmega(vx, vy, z);
                (void)Chassis_ApplyMotion(vx, vy, z, 0U);
                break;
            }

            elapsed_ms = now_ms - chassis_ramp_begin_ms;
            if (elapsed_ms >= CHASSIS_STOP_RAMP_MS)
            {
                remaining_ms = 0U;
                chassis_motion_state = CHASSIS_MOTION_STOPPED;
            }
            else
            {
                remaining_ms = CHASSIS_STOP_RAMP_MS - elapsed_ms;
            }
            for (i = 0U; i < CHASSIS_WHEEL_COUNT; i++)
            {
                wheel_rpm[i] = Chassis_ScaleRpm(
                    chassis_stop_start_rpm[i], remaining_ms,
                    CHASSIS_STOP_RAMP_MS);
            }
            (void)Chassis_SetWheelRpm(wheel_rpm);
            break;

        case CHASSIS_MOTION_EMERGENCY_STOP:
            if (motion_requested)
            {
                chassis_ramp_begin_ms = now_ms;
                chassis_motion_state = CHASSIS_MOTION_STARTING;
            }
            else
            {
                chassis_motion_state = CHASSIS_MOTION_STOPPED;
            }
            break;

        default:
            chassis_emergency_stop = true;
            chassis_motion_state = CHASSIS_MOTION_EMERGENCY_STOP;
            break;
        }
    }

    if (send_now ||
        ((now_ms - chassis_last_tx_ms) >= CHASSIS_COMMAND_PERIOD_MS))
    {
        chassis_last_tx_ms = now_ms;
        Chassis_SendWheelRpm();
    }
}

/**
 * @brief 锁存底盘急停，底盘任务最迟 1ms 内整组清零
 * @retval None
 */
void Chassis_StopAll(void)
{
    uint32_t primask;

    if (!chassis_ready)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    chassis_target_vx = 0;
    chassis_target_vy = 0;
    chassis_target_z = 0;
    chassis_emergency_stop = true;
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/**
 * @brief 获取指定底盘车轮的 VESC 状态
 * @param wheel 车轮编号
 * @param status VESC 状态输出
 * @retval true 获取成功，false 参数无效或底盘未初始化
 */
bool Chassis_GetStatus(chassis_wheel_t wheel,
                       vesc_motor_status_t *status)
{
    if (!chassis_ready || (wheel >= CHASSIS_WHEEL_COUNT))
    {
        return false;
    }
    return VescMotor_GetStatus(&chassis_motors[wheel], status);
}
