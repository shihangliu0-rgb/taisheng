/**
 ******************************************************************************
 * @file    path_pure_pursuit.c
 * @brief   纯追踪实现
 ******************************************************************************
 */
#include "path_pure_pursuit.h"

#include "path_config.h"

#include <math.h>
#include <stddef.h>

void PathPurePursuit_Find(const path_point_t *points, uint16_t count,
                          float x, float y, float v_ref, float kappa,
                          uint16_t i_near, uint16_t *i_target,
                          float *tx, float *ty)
{
    float lookahead = PATH_LD_MIN_M + PATH_LD_K_S * v_ref;
    float kappa_cap = PATH_LD_KAPPA_MAX_M /
                      sqrtf(fabsf(kappa) + 0.05f);
    uint16_t i;

    /* 急弯处缩短前视距离,抑制抄近道 */
    if (lookahead > kappa_cap)
    {
        lookahead = kappa_cap;
    }

    if ((points == NULL) || (i_target == NULL))
    {
        return;
    }
    if (i_near >= count)
    {
        i_near = (uint16_t)(count - 1U);
    }

    /* 从最近点起向后(轨迹方向)找第一个距离 >= Ld 的点 */
    for (i = i_near; i < count; i++)
    {
        float dx = points[i].x_m - x;
        float dy = points[i].y_m - y;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist >= lookahead)
        {
            *i_target = i;
            if (tx != NULL) { *tx = points[i].x_m; }
            if (ty != NULL) { *ty = points[i].y_m; }
            return;
        }
    }

    /* 剩余段不足一个前视距离:直接取终点 */
    *i_target = (uint16_t)(count - 1U);
    if (tx != NULL) { *tx = points[count - 1U].x_m; }
    if (ty != NULL) { *ty = points[count - 1U].y_m; }
}
