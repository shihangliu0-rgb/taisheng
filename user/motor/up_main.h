#ifndef UP_MAIN_H
#define UP_MAIN_H

#include "rs_app.h"

#include <stdbool.h>
#include <stdint.h>

#define UP_SECOND_ZERO_OFFSET_DEG 62.0f

typedef struct
{
    float rs_l_deg;
    float rs_r_deg;
} up_motor_angles_t;

typedef enum
{
    UP_STATE_INIT,
    UP_STATE_FIRST_ZERO,
    UP_STATE_MOVE_SECOND_ZERO,
    UP_STATE_SECOND_ZERO,
    UP_STATE_READY,
    UP_STATE_STOPPED,
    UP_STATE_ERROR
} up_state_t;

typedef enum
{
    UP_ZERO_DISABLE_RS,
    UP_ZERO_WAIT_DISABLED,
    UP_ZERO_CLEAR_RS_FAULT,
    UP_ZERO_SET_RS,
    UP_ZERO_ENABLE_RS,
    UP_ZERO_WAIT_READY
} up_zero_step_t;

extern up_motor_angles_t up_target_angles;
extern up_motor_angles_t up_command_angles;
extern HAL_StatusTypeDef up_last_result;
extern bool up_curve_running;
extern up_state_t up_state;
extern up_zero_step_t up_zero_step;

HAL_StatusTypeDef Up_Init(void);
HAL_StatusTypeDef Up_HomeMotors(void);
bool Up_IsReady(void);
HAL_StatusTypeDef Up_SetRsPos(float left_deg, float right_deg);
bool Up_GetMotorAngles(up_motor_angles_t *angles);
bool Up_MotorMoveDone(void);
bool Up_RsMoveDone(void);
HAL_StatusTypeDef Up_RebaseRsPosition(void);
HAL_StatusTypeDef Up_GetPositionRebaseStatus(void);
void Up_Run1ms(void);
void Up_StopAll(void);
bool Up_GetRsStatus(uint8_t id, rs_app_status_t *status);

#endif
