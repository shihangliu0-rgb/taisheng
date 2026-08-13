/**
 ******************************************************************************
 * @file    path_yaw_lock.c
 * @brief   航向锁定实现
 ******************************************************************************
 */
#include "path_yaw_lock.h"

#include "path_config.h"
#include "path_geometry.h"   /* PathWrapAngle */

#include <math.h>

#define DEG2RAD (float)(PATH_PI / 180.0)
#define RAD2DEG (float)(180.0 / PATH_PI)

float PathYawLock_Step(float yaw_rad, float speed_ms)
{
    float err_deg = PathWrapAngle(PATH_YAW_TARGET_RAD - yaw_rad) * RAD2DEG;
    float kp;
    float w_max;
    float w;

    /* 死区 ±1°:高速直线时避免抖动 */
    if (fabsf(err_deg) <= PATH_YAW_DEADZONE_DEG)
    {
        return 0.0f;
    }

    kp = (fabsf(err_deg) <= PATH_YAW_KP_THRES_DEG) ?
         PATH_YAW_KP_SMALL : PATH_YAW_KP_LARGE;
    w = kp * err_deg * DEG2RAD;

    /* 速度自适应限幅 */
    w_max = PATH_W_BASE_RAD_S - PATH_W_SLOPE * speed_ms;
    if (w_max < PATH_W_MIN_RAD_S)
    {
        w_max = PATH_W_MIN_RAD_S;
    }
    if (w > w_max)
    {
        w = w_max;
    }
    if (w < -w_max)
    {
        w = -w_max;
    }

    return w;
}
