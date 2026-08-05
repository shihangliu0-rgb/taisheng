#include "chassis_main.h"

#include "fdcan.h"
#include "imu_main.h"

#include <stddef.h>

#define ARRAY_SIZE(array) ((uint8_t)(sizeof(array) / sizeof((array)[0])))

#define CHASSIS_MOTOR_LF_ID       87U
#define CHASSIS_MOTOR_RF_ID       43U
#define CHASSIS_MOTOR_LR_ID       67U
#define CHASSIS_MOTOR_RR_ID       59U
#define CHASSIS_MOTOR_POLE_PAIRS  21U
#define CHASSIS_MAX_RPM           4000
#define CHASSIS_MIN_RPM           0
#define CHASSIS_BRAKE_CURRENT_A   18.0f
#define CHASSIS_HALF_WIDTH_SCALE  4.31f
#define CHASSIS_HALF_LENGTH_SCALE 5.30f
#define CHASSIS_ROTATION_SCALE    (CHASSIS_HALF_WIDTH_SCALE + \
                                   CHASSIS_HALF_LENGTH_SCALE)
#define CHASSIS_COMMAND_PERIOD_MS 10U
#define CHASSIS_RX_DRAIN_MAX      32U

static const vesc_motor_config_t motor_config[] = {
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

static vesc_can_t vesc_bus;
static vesc_motor_t motors[ARRAY_SIZE(motor_config)];
static uint32_t last_command_ms;
static bool chassis_ready;

volatile int16_t chassis_target_vx=0;
volatile int16_t chassis_target_vy=0;
volatile int16_t chassis_target_z=0;

static void process_feedback(uint32_t now_ms)
{
    vesc_can_msg_t msg;
    uint8_t drain_count = 0U;
    uint8_t i;

    VescCan_Recover(&vesc_bus);
    while (drain_count < CHASSIS_RX_DRAIN_MAX)
    {
        HAL_StatusTypeDef status = VescCan_Read(&vesc_bus, &msg);

        if (status == HAL_BUSY)
        {
            break;
        }
        drain_count++;
        if (status != HAL_OK)
        {
            continue;
        }

        for (i = 0U; i < ARRAY_SIZE(motors); i++)
        {
            if (VescMotor_Parse(&motors[i], &msg, now_ms))
            {
                break;
            }
        }
    }

    for (i = 0U; i < ARRAY_SIZE(motors); i++)
    {
        VescMotor_Update(&motors[i], now_ms);
    }
}

static void send_targets(void)
{
    uint8_t i;

    for (i = 0U; i < ARRAY_SIZE(motors); i++)
    {
        (void)VescMotor_SendRpm(&motors[i]);
    }
}

static HAL_StatusTypeDef update_targets(int16_t vx, int16_t vy, int16_t z)
{
    int32_t rotation;
    int32_t target_rpm[CHASSIS_WHEEL_COUNT];
    uint8_t i;

    // 机器人坐标系：X 向右、Y 向前、Z 逆时针为正
    rotation = (int32_t)((float)z * CHASSIS_ROTATION_SCALE);
    target_rpm[CHASSIS_WHEEL_LF] =  vx + vy - rotation;
    target_rpm[CHASSIS_WHEEL_RF] =  vx - vy - rotation;
    target_rpm[CHASSIS_WHEEL_LR] =  vx - vy + rotation;
    target_rpm[CHASSIS_WHEEL_RR] =  vx + vy + rotation;

    for (i = 0U; i < ARRAY_SIZE(motors); i++)
    {
        if (VescMotor_SetRpm(&motors[i], target_rpm[i]) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef Chassis_Init(void)
{
    uint8_t i;

    if (chassis_ready)
    {
        return HAL_OK;
    }
    if (VescCan_Init(&vesc_bus, &hfdcan1) != HAL_OK)
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < ARRAY_SIZE(motors); i++)
    {
        if (VescMotor_Init(&motors[i], &vesc_bus,
                           &motor_config[i]) != HAL_OK)
        {
            VescCan_Stop(&vesc_bus);
            return HAL_ERROR;
        }
    }

    chassis_ready = true;
    last_command_ms = HAL_GetTick();
    send_targets();
    return HAL_OK;
}

HAL_StatusTypeDef Chassis_SetVelocity(int16_t vx, int16_t vy, int16_t z)
{
    if (!chassis_ready)
    {
        return HAL_ERROR;
    }

    chassis_target_vx = vx;
    chassis_target_vy = vy;
    chassis_target_z = z;
    return update_targets(vx, vy, z);
}

void Chassis_Run1ms(void)
{
    uint32_t now_ms;
    int16_t vx;
    int16_t vy;
    int16_t z;

    if (!chassis_ready)
    {
        return;
    }

    now_ms = HAL_GetTick();
    process_feedback(now_ms);
    vx = chassis_target_vx;
    vy = chassis_target_vy;
    z = ImuMain_CalcOmega(vx, vy, chassis_target_z);
    (void)update_targets(vx, vy, z);
    if ((now_ms - last_command_ms) >= CHASSIS_COMMAND_PERIOD_MS)
    {
        last_command_ms = now_ms;
        send_targets();
    }
}

void Chassis_StopAll(void)
{
    uint8_t i;

    if (!chassis_ready)
    {
        return;
    }

    chassis_target_vx = 0;
    chassis_target_vy = 0;
    chassis_target_z = 0;
    for (i = 0U; i < ARRAY_SIZE(motors); i++)
    {
        (void)VescMotor_SetRpm(&motors[i], 0);
    }
    send_targets();
}

bool Chassis_GetStatus(chassis_wheel_t wheel,
                       vesc_motor_status_t *status)
{
    if (!chassis_ready || (wheel >= CHASSIS_WHEEL_COUNT))
    {
        return false;
    }

    return VescMotor_GetStatus(&motors[wheel], status);
}
