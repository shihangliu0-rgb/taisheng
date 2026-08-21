#include "laser_safety.h"

#include "dt35_pnp_link.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>

/* DT35 距离单位 cm，底盘命令单位 mm/s，转换系数 10 mm/cm。 */
#define LASER_SAFETY_CM_TO_MM 10.0f

static laser_safety_clamp_t laser_last_clamp;

/*
 * 按刹车距离模型计算允许的最大速度(mm/s)：
 *   margin = 距离 - 硬停距离；
 *   margin <= 0            -> 0（完全禁止该方向）；
 *   v_max >= 全速阈值      -> INT32_MAX（不限制）；
 *   其余                   -> sqrt(2 * a * margin)。
 */
static int32_t LaserSafety_MaxSpeedMmS(uint16_t distance_cm)
{
    float margin_cm = (float)distance_cm - LASER_SAFETY_STOP_DISTANCE_CM;
    float max_speed_mm_s;

    if (margin_cm <= 0.0f)
    {
        return 0;
    }
    max_speed_mm_s = sqrtf(2.0f * LASER_SAFETY_BRAKE_ACCEL_MM_S2 *
                           margin_cm * LASER_SAFETY_CM_TO_MM);
    if (max_speed_mm_s >= (float)LASER_SAFETY_FULL_SPEED_MM_S)
    {
        return INT32_MAX;
    }
    return (int32_t)max_speed_mm_s;
}

void LaserSafety_ClampCommand(int16_t vx, int16_t vy, int16_t z,
                              laser_safety_clamp_t *out)
{
    uint16_t front_cm;
    uint16_t left_cm;
    int32_t max_speed;

    if (out == NULL)
    {
        return;
    }

    out->vx = vx;
    out->vy = vy;
    out->z = z;
    out->front_limited = false;
    out->front_blocked = false;
    out->left_limited = false;
    out->left_blocked = false;

    front_cm = dt35_link[SENSOR_LINK_F_INDEX].distance_cm;
    left_cm = dt35_link[SENSOR_LINK_L_B_INDEX].distance_cm;
    out->front_offline = (dt35_link[SENSOR_LINK_F_INDEX].online == 0U);
    out->left_offline = (dt35_link[SENSOR_LINK_L_B_INDEX].online == 0U);

    /* 前方传感器只约束向前速度(vy > 0)，后退放行。 */
    if (!out->front_offline)
    {
        max_speed = LaserSafety_MaxSpeedMmS(front_cm);
        if ((vy > 0) && ((int32_t)vy > max_speed))
        {
            out->vy = (max_speed > 0) ? (int16_t)max_speed : 0;
            out->front_limited = true;
            out->front_blocked = (max_speed == 0);
        }
    }

    /* 左侧传感器只约束向左平移(vx < 0)，向右平移放行。 */
    if (!out->left_offline)
    {
        max_speed = LaserSafety_MaxSpeedMmS(left_cm);
        if ((vx < 0) && (-(int32_t)vx > max_speed))
        {
            out->vx = (max_speed > 0) ? (int16_t)(-max_speed) : 0;
            out->left_limited = true;
            out->left_blocked = (max_speed == 0);
        }
    }
}

HAL_StatusTypeDef LaserSafety_RequestVelocity(chassis_cmd_source_t source,
                                              int16_t vx, int16_t vy,
                                              int16_t z, uint32_t timeout_ms)
{
    laser_safety_clamp_t out;

    LaserSafety_ClampCommand(vx, vy, z, &out);
    return Chassis_RequestVelocity(source, out.vx, out.vy, out.z,
                                   timeout_ms);
}

HAL_StatusTypeDef LaserSafety_SetVelocity(int16_t vx, int16_t vy, int16_t z)
{
    laser_safety_clamp_t out;

    LaserSafety_ClampCommand(vx, vy, z, &out);
    return Chassis_SetVelocity(out.vx, out.vy, out.z);
}

void LaserSafety_Run1ms(void)
{
    laser_safety_clamp_t out;
    chassis_cmd_source_t source;

    LaserSafety_ClampCommand(chassis_target_vx, chassis_target_vy,
                             chassis_target_z, &out);
    laser_last_clamp = out;

    /* 当前生效目标已经满足限速要求，不需要干预。 */
    if ((out.vx == chassis_target_vx) && (out.vy == chassis_target_vy))
    {
        return;
    }
    /* 没有活动命令源时不主动创造命令。 */
    if (!Chassis_GetActiveSource(&source))
    {
        return;
    }

    /*
     * 距离比命令下发时更近(或命令未经包装直接写入)，用限速值在活动命令源上
     * 重发一条短超时命令。命令源自身的新帧会以当前距离重新限速；
     * 若命令源离线，这条短超时命令到期后底盘自动停止。
     */
    (void)Chassis_RequestVelocity(source, out.vx, out.vy, out.z,
                                  LASER_SAFETY_ENFORCE_TIMEOUT_MS);
}

bool LaserSafety_GetStatus(laser_safety_clamp_t *status)
{
    if (status == NULL)
    {
        return false;
    }
    *status = laser_last_clamp;
    return true;
}
