#include "action_api.h"

#include "up_main.h"

#define SECOND_ZERO_COMPENSATION_DEG 17.0f
#define RS_L_SUPPORT_DEG             179.0f
#define RS_R_SUPPORT_DEG             177.0f
#define RS_L_LIFT_TRAVEL_DEG         (RS_L_SUPPORT_DEG - UP_SECOND_ZERO_OFFSET_DEG + SECOND_ZERO_COMPENSATION_DEG)
#define RS_R_LIFT_TRAVEL_DEG         (RS_R_SUPPORT_DEG - UP_SECOND_ZERO_OFFSET_DEG + SECOND_ZERO_COMPENSATION_DEG)
#define RS_L_WRAP_TRAVEL_DEG         (360.0f - RS_L_SUPPORT_DEG)
#define RS_R_WRAP_TRAVEL_DEG         (360.0f - RS_R_SUPPORT_DEG)
#define RS_FLAT_TARGET_DEG           90.0f
#define ALIGN_TRAVEL_DEG             45.0f
#define ALIGN_TOLERANCE_DEG          3.0f
#define ACTION_TIMEOUT_MS            5000U

typedef enum { LEG_INITIAL, LEG_LIFTED, LEG_FLAT } leg_state_t;
typedef enum {
    ACTION_IDLE,
    ACTION_WAIT_LOWER,
    ACTION_WAIT_LIFT,
    ACTION_WAIT_FRONT_WRAP,
    ACTION_WAIT_FRONT_ZERO,
    ACTION_WAIT_FRONT_FLAT,
    ACTION_WAIT_FRONT_DOWN,
    ACTION_WAIT_ALIGN
} action_step_t;

static action_step_t action_step;
static leg_state_t front_state;
static bool motors_ready_latched;
static bool align_target_active;
static bool align_move_target_active;
static bool align_at_zero = true;
static bool align_switch_initialized;
static uint8_t align_switch_last;
static bool lift_target_active;
static bool lift_switch_initialized;
static uint8_t lift_switch_last;
static bool lift_switch_request_pending;
static bool remote_buttons_initialized;
static uint8_t remote_buttons_last;
static uint32_t action_due_ms;

volatile action_cmd_t action_pending = ACTION_CMD_NONE;
volatile uint8_t action_pnp_f_trigger;
volatile uint8_t action_pnp_b_trigger;
HAL_StatusTypeDef action_last_result = HAL_OK;

static bool time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return (int32_t)(now_ms - due_ms) >= 0;
}

static void set_action_step(action_step_t step)
{
    action_step = step;
    action_due_ms = HAL_GetTick() + ACTION_TIMEOUT_MS;
}

static void complete_action(void)
{
    action_last_result = HAL_OK;
    action_step = ACTION_IDLE;
}

static void fail_action(HAL_StatusTypeDef status)
{
    action_last_result = status;
    action_step = ACTION_IDLE;
    Up_StopAll();
}

static bool align_targets_at_zero(void)
{
    return (up_target_angles.rs_l_deg >= -ALIGN_TOLERANCE_DEG) &&
           (up_target_angles.rs_l_deg <= ALIGN_TOLERANCE_DEG) &&
           (up_target_angles.rs_r_deg >= -ALIGN_TOLERANCE_DEG) &&
           (up_target_angles.rs_r_deg <= ALIGN_TOLERANCE_DEG);
}

static bool action_allowed(action_cmd_t action)
{
    switch (action)
    {
    case ACTION_CMD_LOWER:
        return front_state == LEG_LIFTED;
    case ACTION_CMD_LIFT:
        return (front_state == LEG_INITIAL) &&
               !align_target_active && align_at_zero;
    case ACTION_CMD_FRONT_FLAT:
        return front_state == LEG_LIFTED;
    case ACTION_CMD_FRONT_DOWN:
        return front_state == LEG_FLAT;
    case ACTION_CMD_ALIGN:
        return (front_state == LEG_INITIAL) &&
               (!align_target_active || align_targets_at_zero());
    default:
        return false;
    }
}

static void start_action(action_cmd_t action)
{
    HAL_StatusTypeDef result = HAL_ERROR;

    switch (action)
    {
    case ACTION_CMD_LOWER:
        result = Up_SetRsPos(up_target_angles.rs_l_deg - RS_L_LIFT_TRAVEL_DEG,
                             up_target_angles.rs_r_deg - RS_R_LIFT_TRAVEL_DEG);
        if (result == HAL_OK) set_action_step(ACTION_WAIT_LOWER);
        break;
    case ACTION_CMD_LIFT:
        result = Up_SetRsPos(up_target_angles.rs_l_deg + RS_L_LIFT_TRAVEL_DEG,
                             up_target_angles.rs_r_deg + RS_R_LIFT_TRAVEL_DEG);
        if (result == HAL_OK) set_action_step(ACTION_WAIT_LIFT);
        break;
    case ACTION_CMD_ALIGN:
        align_move_target_active = align_target_active;
        align_at_zero = !align_move_target_active;
        result = Up_SetRsPos(align_move_target_active ? ALIGN_TRAVEL_DEG : 0.0f,
                             align_move_target_active ? ALIGN_TRAVEL_DEG : 0.0f);
        if (result == HAL_OK) set_action_step(ACTION_WAIT_ALIGN);
        break;
    case ACTION_CMD_FRONT_FLAT:
        result = Up_SetRsPos(up_target_angles.rs_l_deg + RS_L_WRAP_TRAVEL_DEG,
                             up_target_angles.rs_r_deg + RS_R_WRAP_TRAVEL_DEG);
        if (result == HAL_OK) set_action_step(ACTION_WAIT_FRONT_WRAP);
        break;
    case ACTION_CMD_FRONT_DOWN:
        result = Up_SetRsPos(RS_L_SUPPORT_DEG, RS_R_SUPPORT_DEG);
        if (result == HAL_OK) set_action_step(ACTION_WAIT_FRONT_DOWN);
        break;
    default:
        break;
    }
    action_last_result = result;
}

static void run_action_step(void)
{
    HAL_StatusTypeDef rebase_status;
    uint32_t now_ms = HAL_GetTick();

    switch (action_step)
    {
    case ACTION_WAIT_LOWER:
        if (Up_RsMoveDone()) { front_state = LEG_INITIAL; complete_action(); }
        break;
    case ACTION_WAIT_LIFT:
        if (Up_RsMoveDone()) { front_state = LEG_LIFTED; complete_action(); }
        break;
    case ACTION_WAIT_FRONT_WRAP:
        if (Up_RsMoveDone())
        {
            if (Up_RebaseRsPosition() == HAL_OK) set_action_step(ACTION_WAIT_FRONT_ZERO);
            else fail_action(HAL_ERROR);
        }
        break;
    case ACTION_WAIT_FRONT_ZERO:
        rebase_status = Up_GetPositionRebaseStatus();
        if (rebase_status == HAL_OK)
        {
            if (Up_SetRsPos(RS_FLAT_TARGET_DEG, RS_FLAT_TARGET_DEG) == HAL_OK)
                set_action_step(ACTION_WAIT_FRONT_FLAT);
            else fail_action(HAL_ERROR);
        }
        else if (rebase_status != HAL_BUSY) fail_action(rebase_status);
        break;
    case ACTION_WAIT_FRONT_FLAT:
        if (Up_RsMoveDone()) { front_state = LEG_FLAT; complete_action(); }
        break;
    case ACTION_WAIT_FRONT_DOWN:
        if (Up_RsMoveDone()) { front_state = LEG_LIFTED; complete_action(); }
        break;
    case ACTION_WAIT_ALIGN:
        if (Up_RsMoveDone()) { align_at_zero = !align_move_target_active; complete_action(); }
        break;
    default:
        break;
    }
    if ((action_step != ACTION_IDLE) && time_reached(now_ms, action_due_ms))
        fail_action(HAL_TIMEOUT);
}

static action_cmd_t take_pending_action(void)
{
    action_cmd_t action;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    action = action_pending;
    action_pending = ACTION_CMD_NONE;
    if (primask == 0U) __enable_irq();
    return action;
}

void Action_UpdatePnp(uint8_t trigger_f, uint8_t trigger_b)
{
    action_pnp_f_trigger = (trigger_f != 0U) ? 1U : 0U;
    action_pnp_b_trigger = (trigger_b != 0U) ? 1U : 0U;
}

void Action_UpdateAlignSwitch(uint8_t switch_state, uint8_t online)
{
    switch_state = (switch_state != 0U) ? 1U : 0U;
    if (online == 0U) { align_switch_initialized = false; return; }
    if (!align_switch_initialized)
    {
        align_switch_last = switch_state;
        align_switch_initialized = true;
        return;
    }
    if (switch_state != align_switch_last)
    {
        align_switch_last = switch_state;
        (void)Action_Request(ACTION_CMD_ALIGN);
    }
}

void Action_UpdateLiftSwitch(uint8_t switch_state, uint8_t online)
{
    switch_state = (switch_state != 0U) ? 1U : 0U;
    if (online == 0U) { lift_switch_initialized = false; return; }
    if (!lift_switch_initialized)
    {
        lift_switch_last = switch_state;
        lift_switch_initialized = true;
        return;
    }
    if (switch_state != lift_switch_last)
    {
        lift_switch_last = switch_state;
        lift_target_active = !lift_target_active;
        lift_switch_request_pending = true;
        (void)Action_Request(lift_target_active ? ACTION_CMD_LIFT : ACTION_CMD_LOWER);
    }
}

void Action_UpdateRemoteButtons(uint8_t buttons, uint8_t online)
{
    static const action_cmd_t button_actions[6] = {
        ACTION_CMD_FRONT_FLAT, ACTION_CMD_FRONT_DOWN,
        ACTION_CMD_NONE, ACTION_CMD_NONE, ACTION_CMD_NONE, ACTION_CMD_NONE
    };
    uint8_t pressed;
    uint8_t i;

    buttons &= 0x3FU;
    if (online == 0U) { remote_buttons_initialized = false; return; }
    if (!remote_buttons_initialized)
    {
        remote_buttons_last = buttons;
        remote_buttons_initialized = true;
        return;
    }
    pressed = (uint8_t)(buttons & (uint8_t)~remote_buttons_last);
    remote_buttons_last = buttons;
    for (i = 0U; i < 6U; i++)
    {
        if (((pressed & (uint8_t)(1U << i)) != 0U) &&
            (button_actions[i] != ACTION_CMD_NONE))
        {
            (void)Action_Request(button_actions[i]);
            break;
        }
    }
}

HAL_StatusTypeDef Action_Request(action_cmd_t action)
{
    if ((action > ACTION_CMD_MAX) ||
        ((action != ACTION_CMD_LOWER) && (action != ACTION_CMD_LIFT) &&
         (action != ACTION_CMD_FRONT_FLAT) &&
         (action != ACTION_CMD_FRONT_DOWN) &&
         (action != ACTION_CMD_ALIGN)))
    {
        return HAL_ERROR;
    }
    if (action == ACTION_CMD_ALIGN) align_target_active = !align_target_active;
    action_pending = action;
    return HAL_OK;
}

void Action_Run1ms(void)
{
    action_cmd_t action;

    if (!Up_IsReady())
    {
        motors_ready_latched = false;
    }
    else if (!motors_ready_latched)
    {
        front_state = LEG_INITIAL;
        align_at_zero = true;
        motors_ready_latched = true;
    }
    if (action_step != ACTION_IDLE)
    {
        run_action_step();
        return;
    }
    action = take_pending_action();
    if (action == ACTION_CMD_NONE) return;
    if (!Up_IsReady() || !action_allowed(action))
    {
        if (action == ACTION_CMD_ALIGN) align_target_active = !align_target_active;
        if (lift_switch_request_pending &&
            ((action == ACTION_CMD_LIFT) || (action == ACTION_CMD_LOWER)))
        {
            lift_target_active = !lift_target_active;
            lift_switch_request_pending = false;
        }
        action_last_result = HAL_ERROR;
        return;
    }
    if (lift_switch_request_pending &&
        ((action == ACTION_CMD_LIFT) || (action == ACTION_CMD_LOWER)))
        lift_switch_request_pending = false;
    start_action(action);
}
