#include "up_main.h"

#include "fdcan_task.h"

#include <float.h>
#include <stddef.h>

#define ARRAY_SIZE(array) ((uint8_t)(sizeof(array) / sizeof((array)[0])))

#define RS_HOST_ID           0xFDU
#define RS_MOTOR_L_ID        39U
#define RS_MOTOR_F_ID        40U
#define DM_MOTOR_L_ID        0x05U
#define DM_MOTOR_F_ID        0x07U
#define DM_MOTOR_L_RX_ID     0x15U
#define DM_MOTOR_F_RX_ID     0x17U
#define M2006_MOTOR_L_ID     1U
#define M2006_MOTOR_F_ID     2U

#define RS_HOME_ANGLE_DEG    0.0f
#define RS_HOME_SPEED_RAD_S  9.5f
#define DM_HOME_ANGLE_DEG    0.0f
#define DM_HOME_SPEED_RAD_S  0.2f
#define DM_HOME_TORQUE_NM    0.0f
#define DM_HOME_KP           70.0f
#define DM_HOME_KD           1.25f
#define M2006_HOME_ANGLE_DEG 0.0f
#define M2006_PID_KP         45.0f
#define M2006_PID_KI         0.0f
#define M2006_PID_KD         5.0f
#define MOTOR_MOVE_TIME_MS   2000U // 四台电机到达目标角度的总时间
#define RAD_TO_DEG           57.2957795f
#define DM_MAX_ANGLE_DEG     (DM_J4310_P_MAX * RAD_TO_DEG)

#if (MOTOR_MOVE_TIME_MS == 0U)
#error "MOTOR_MOVE_TIME_MS must be greater than zero"
#endif

typedef struct
{
    float rs_l_deg;
    float rs_f_deg;
    float dm_l_deg;
    float dm_f_deg;
} motor_angles_t;

typedef struct
{
    motor_angles_t start;
    motor_angles_t target;
    motor_angles_t current;
    uint32_t start_ms;
    bool running;
} motor_curve_t;

// RS、DM 和 M2006 的接线 ID 与默认控制参数集中在本文件
static const rs_app_motor_config_t rs_motor_config[] = {
    {
        .id = RS_MOTOR_L_ID,
        .period_ms = RS_APP_CONTROL_PERIOD_MS,
        .command = {
            .mode = RS_CSP,
            .data.csp = {
                .angle_deg = RS_HOME_ANGLE_DEG,
                .max_speed_rad_s = RS_HOME_SPEED_RAD_S
            }
        }
    },
    {
        .id = RS_MOTOR_F_ID,
        .period_ms = RS_APP_CONTROL_PERIOD_MS,
        .command = {
            .mode = RS_CSP,
            .data.csp = {
                .angle_deg = RS_HOME_ANGLE_DEG,
                .max_speed_rad_s = RS_HOME_SPEED_RAD_S
            }
        }
    }
};

static const dm_app_motor_config_t dm_motor_config[] = {
    {
        .tx_id = DM_MOTOR_L_ID,
        .master_id = DM_MOTOR_L_RX_ID,
        .feedback_id = DM_MOTOR_L_ID,
        .command = {
            .angle_deg = DM_HOME_ANGLE_DEG,
            .speed_rad_s = DM_HOME_SPEED_RAD_S,
            .torque_nm = DM_HOME_TORQUE_NM,
            .kp = DM_HOME_KP,
            .kd = DM_HOME_KD
        }
    },
    {
        .tx_id = DM_MOTOR_F_ID,
        .master_id = DM_MOTOR_F_RX_ID,
        .feedback_id = DM_MOTOR_F_ID,
        .command = {
            .angle_deg = DM_HOME_ANGLE_DEG,
            .speed_rad_s = DM_HOME_SPEED_RAD_S,
            .torque_nm = DM_HOME_TORQUE_NM,
            .kp = DM_HOME_KP,
            .kd = DM_HOME_KD
        }
    }
};

static const m2006_config_t m2006_motor_config[] = {
    {
        .id = M2006_MOTOR_L_ID,
        .target_position_deg = M2006_HOME_ANGLE_DEG,
        .pid = {
            .kp = M2006_PID_KP,
            .ki = M2006_PID_KI,
            .kd = M2006_PID_KD
        }
    },
    {
        .id = M2006_MOTOR_F_ID,
        .target_position_deg = M2006_HOME_ANGLE_DEG,
        .pid = {
            .kp = M2006_PID_KP,
            .ki = M2006_PID_KI,
            .kd = M2006_PID_KD
        }
    }
};

// FDCAN3 同时接收 DM 和 M2006 的标准帧反馈。
static const uint16_t std_rx_ids[] = {
    DM_MOTOR_L_RX_ID,
    DM_MOTOR_F_RX_ID,
    C610_FEEDBACK_ID(M2006_MOTOR_L_ID),
    C610_FEEDBACK_ID(M2006_MOTOR_F_ID)
};

static const fdcan_config_t fdcan_config = {
    .rs_host_id = RS_HOST_ID,
    .std_rx_ids = std_rx_ids,
    .std_rx_id_count = ARRAY_SIZE(std_rx_ids)
};

static rs_app_t rs_app;
static dm_app_t dm_app;
static c610_bus_t c610_bus;
static m2006_motor_t m2006_motors[ARRAY_SIZE(m2006_motor_config)];
static motor_curve_t motor_curve;
// 防止电机离线期间每 1 ms 重复发起重启。
static bool rs_initialization_requested[ARRAY_SIZE(rs_motor_config)];
static bool dm_initialization_requested[ARRAY_SIZE(dm_motor_config)];
static bool app_ready;               // 所有总线和电机对象均初始化完成
static bool auto_initialize_enabled; // StopAll 后禁止离线电机自动重启
static bool motors_initialized;      // 区分首次使能和掉线后的重启

static bool motor_angles_valid(const motor_angles_t *angles)
{
    if ((angles == NULL) ||
        (angles->rs_l_deg != angles->rs_l_deg) ||
        (angles->rs_f_deg != angles->rs_f_deg) ||
        (angles->dm_l_deg != angles->dm_l_deg) ||
        (angles->dm_f_deg != angles->dm_f_deg))
    {
        return false;
    }

    return (angles->rs_l_deg <= FLT_MAX) &&
           (angles->rs_l_deg >= -FLT_MAX) &&
           (angles->rs_f_deg <= FLT_MAX) &&
           (angles->rs_f_deg >= -FLT_MAX) &&
           (angles->dm_l_deg <= DM_MAX_ANGLE_DEG) &&
           (angles->dm_l_deg >= -DM_MAX_ANGLE_DEG) &&
           (angles->dm_f_deg <= DM_MAX_ANGLE_DEG) &&
           (angles->dm_f_deg >= -DM_MAX_ANGLE_DEG);
}

static float curve_value(float start, float target, float progress)
{
    float progress_2 = progress * progress;
    float progress_3 = progress_2 * progress;
    float blend = progress_3 *
                  (10.0f + progress * (-15.0f + 6.0f * progress));

    return start + (target - start) * blend;
}

static HAL_StatusTypeDef set_motor_angles(const motor_angles_t *angles)
{
    rs_command_t rs_command = rs_motor_config[0].command;
    dm_app_mit_command_t dm_command = dm_motor_config[0].command;

    rs_command.data.csp.angle_deg = angles->rs_l_deg;
    if (RsApp_SetCmd(&rs_app, RS_MOTOR_L_ID, &rs_command) != HAL_OK)
    {
        return HAL_ERROR;
    }
    rs_command = rs_motor_config[1].command;
    rs_command.data.csp.angle_deg = angles->rs_f_deg;
    if (RsApp_SetCmd(&rs_app, RS_MOTOR_F_ID, &rs_command) != HAL_OK)
    {
        return HAL_ERROR;
    }

    dm_command.angle_deg = angles->dm_l_deg;
    if (DmApp_SetMitCmd(&dm_app, DM_MOTOR_L_ID, &dm_command) != DM_OK)
    {
        return HAL_ERROR;
    }
    dm_command = dm_motor_config[1].command;
    dm_command.angle_deg = angles->dm_f_deg;
    if (DmApp_SetMitCmd(&dm_app, DM_MOTOR_F_ID, &dm_command) != DM_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static void update_motor_curve(uint32_t now_ms)
{
    uint32_t elapsed_ms;
    float progress;

    if (!motor_curve.running)
    {
        return;
    }

    elapsed_ms = now_ms - motor_curve.start_ms;
    if (elapsed_ms >= MOTOR_MOVE_TIME_MS)
    {
        progress = 1.0f;
        motor_curve.running = false;
    }
    else
    {
        progress = (float)elapsed_ms / (float)MOTOR_MOVE_TIME_MS;
    }

    motor_curve.current.rs_l_deg = curve_value(
        motor_curve.start.rs_l_deg, motor_curve.target.rs_l_deg, progress);
    motor_curve.current.rs_f_deg = curve_value(
        motor_curve.start.rs_f_deg, motor_curve.target.rs_f_deg, progress);
    motor_curve.current.dm_l_deg = curve_value(
        motor_curve.start.dm_l_deg, motor_curve.target.dm_l_deg, progress);
    motor_curve.current.dm_f_deg = curve_value(
        motor_curve.start.dm_f_deg, motor_curve.target.dm_f_deg, progress);

    (void)set_motor_angles(&motor_curve.current);
}

static HAL_StatusTypeDef initialize_rs_motor(uint8_t index, bool restart)
{
    const rs_app_motor_config_t *config = &rs_motor_config[index];
    rs_command_t command = config->command;
    HAL_StatusTypeDef status;

    command.data.csp.angle_deg = (index == 0U) ? motor_curve.current.rs_l_deg
                                               : motor_curve.current.rs_f_deg;
    if (RsApp_SetCmd(&rs_app, config->id, &command) != HAL_OK)
    {
        return HAL_ERROR;
    }
    status = restart ? RsApp_Restart(&rs_app, config->id)
                     : RsApp_Enable(&rs_app, config->id, true);
    if (status != HAL_OK)
    {
        return HAL_ERROR;
    }

    rs_initialization_requested[index] = true;
    return HAL_OK;
}

static HAL_StatusTypeDef initialize_dm_motor(uint8_t index, bool restart)
{
    const dm_app_motor_config_t *config = &dm_motor_config[index];
    dm_app_mit_command_t command = config->command;
    dm_result_t result;

    command.angle_deg = (index == 0U) ? motor_curve.current.dm_l_deg
                                      : motor_curve.current.dm_f_deg;
    if (DmApp_SetMitCmd(&dm_app, config->tx_id, &command) != DM_OK)
    {
        return HAL_ERROR;
    }
    result = restart ? DmApp_Restart(&dm_app, config->tx_id)
                     : DmApp_Enable(&dm_app, config->tx_id, true);
    if (result != DM_OK)
    {
        return HAL_ERROR;
    }

    dm_initialization_requested[index] = true;
    return HAL_OK;
}

static void initialize_offline_motors(uint32_t now_ms)
{
    uint8_t i;

    if (!auto_initialize_enabled)
    {
        return;
    }

    // 恢复在线后清除锁存，允许下一次掉线重新触发一次。
    for (i = 0U; i < ARRAY_SIZE(rs_motor_config); i++)
    {
        rs_app_status_t status;

        if (!RsApp_GetStatus(&rs_app, rs_motor_config[i].id, now_ms, &status))
        {
            continue;
        }
        if (status.online)
        {
            rs_initialization_requested[i] = false;
        }
        else if (!rs_initialization_requested[i] &&
                 (!status.has_feedback || (status.feedback.fault == 0U)))
        {
            (void)initialize_rs_motor(i, true);
        }
    }

    for (i = 0U; i < ARRAY_SIZE(dm_motor_config); i++)
    {
        dm_app_status_t status;

        if (!DmApp_GetStatus(&dm_app, dm_motor_config[i].tx_id, now_ms,
                             &status))
        {
            continue;
        }
        if (status.online)
        {
            dm_initialization_requested[i] = false;
        }
        else if (!dm_initialization_requested[i] &&
                 (!status.has_feedback ||
                  (status.feedback.fault == DM_FAULT_NONE)))
        {
            (void)initialize_dm_motor(i, true);
        }
    }
}

HAL_StatusTypeDef Up_Init(void)
{
    if (app_ready)
    {
        return HAL_OK;
    }

    if (Fdcan_Init(&fdcan_config) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (RsApp_Init(&rs_app, Fdcan_GetRsBus(), rs_motor_config,
                   ARRAY_SIZE(rs_motor_config)) != HAL_OK)
    {
        Fdcan_Stop();
        return HAL_ERROR;
    }
    if (DmApp_Init(&dm_app, Fdcan_GetStdBus(), dm_motor_config,
                   ARRAY_SIZE(dm_motor_config)) != DM_OK)
    {
        Fdcan_Stop();
        return HAL_ERROR;
    }
    if (C610_Init(&c610_bus, Fdcan_GetStdBus(), m2006_motors,
                  m2006_motor_config,
                  ARRAY_SIZE(m2006_motor_config)) != HAL_OK)
    {
        Fdcan_Stop();
        return HAL_ERROR;
    }
    app_ready = true;
    if (Up_HomeMotors() != HAL_OK)
    {
        app_ready = false;
        Fdcan_Stop();
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Up_HomeMotors(void)
{
    bool restart = motors_initialized;
    uint8_t i;

    if (!app_ready)
    {
        return HAL_ERROR;
    }

    auto_initialize_enabled = true;
    for (i = 0U; i < ARRAY_SIZE(rs_motor_config); i++)
    {
        if (initialize_rs_motor(i, restart) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    for (i = 0U; i < ARRAY_SIZE(dm_motor_config); i++)
    {
        if (initialize_dm_motor(i, restart) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    motors_initialized = true;

    if (restart)
    {
        return Up_SetMotorPos(RS_HOME_ANGLE_DEG, RS_HOME_ANGLE_DEG,
                              DM_HOME_ANGLE_DEG, DM_HOME_ANGLE_DEG);
    }

    return HAL_OK;
}

HAL_StatusTypeDef Up_SetMotorPos(float rs_l_deg, float rs_f_deg,
                                 float dm_l_deg, float dm_f_deg)
{
    motor_angles_t target = {
        .rs_l_deg = rs_l_deg,
        .rs_f_deg = rs_f_deg,
        .dm_l_deg = dm_l_deg,
        .dm_f_deg = dm_f_deg
    };

    if (!app_ready || !motor_angles_valid(&target))
    {
        return HAL_ERROR;
    }

    motor_curve.start = motor_curve.current;
    motor_curve.target = target;
    motor_curve.start_ms = HAL_GetTick();
    motor_curve.running = true;
    return HAL_OK;
}

HAL_StatusTypeDef Up_SetM2006Pos(uint8_t id, float position_deg)
{
    if (!app_ready)
    {
        return HAL_ERROR;
    }

    return C610_SetPos(&c610_bus, id, position_deg);
}

void Up_Run1ms(void)
{
    uint32_t now_ms;

    if (!app_ready)
    {
        return;
    }

    now_ms = HAL_GetTick();
    // 固定顺序：处理反馈，再恢复状态，最后按各自周期发送控制帧。
    Fdcan_Run1ms();
    initialize_offline_motors(now_ms);
    update_motor_curve(now_ms);
    RsApp_Run(&rs_app, now_ms);
    DmApp_Run(&dm_app, now_ms);
    C610_Run(&c610_bus, now_ms);
}

void Up_StopAll(void)
{
    if (!app_ready)
    {
        return;
    }

    auto_initialize_enabled = false;
    motor_curve.running = false;
    RsApp_StopAll(&rs_app);
    DmApp_StopAll(&dm_app);
    C610_StopAll(&c610_bus);
}

bool Up_GetRsStatus(uint8_t id, rs_app_status_t *status)
{
    return app_ready && RsApp_GetStatus(&rs_app, id, HAL_GetTick(), status);
}

bool Up_GetDmStatus(uint16_t id, dm_app_status_t *status)
{
    return app_ready && DmApp_GetStatus(&dm_app, id, HAL_GetTick(), status);
}

bool Up_GetM2006Status(uint8_t id, m2006_status_t *status)
{
    return app_ready && C610_GetStatus(&c610_bus, id, status);
}
