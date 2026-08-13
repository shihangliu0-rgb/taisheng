/**
 ******************************************************************************
 * @file    path_fusion.c
 * @brief   IMU + 上位机位姿互补融合实现
 ******************************************************************************
 */
#include "path_fusion.h"

#include "path_config.h"
#include "path_geometry.h"   /* PathWrapAngle */

#include <math.h>
#include <string.h>

#define DEG2RAD (float)(PATH_PI / 180.0)
#define RAD2DEG (float)(180.0 / PATH_PI)

typedef struct
{
    float x;
    float y;
    float yaw;                 /* 用户约定 yaw:0 朝 +y,CCW 为正 */
    float zero_offset_deg_s;   /* 静止标定得到的附加陀螺零偏 */
    bool have_upper;
    uint32_t last_upper_ms;
    uint16_t calib_count;
    float calib_sum;

    /* xy 中值滤波(单帧跳变剔除,配合 30cm 门限使用) */
    float median_x[PATH_FUSION_MEDIAN_WIN];
    float median_y[PATH_FUSION_MEDIAN_WIN];
    uint8_t median_i;
    uint8_t median_fill;

    uint32_t xy_rejects;
    uint32_t yaw_rejects;
    uint32_t upper_frames;
} path_fusion_t;

static path_fusion_t fusion;

static float median3(const float *buf, uint8_t n)
{
    float a;
    float b;
    float c;
    float tmp;

    if (n == 0U)
    {
        return 0.0f;
    }
    a = buf[0];
    b = (n > 1U) ? buf[1] : a;
    c = (n > 2U) ? buf[2] : b;

    /* 3 数排序取中值 */
    if (a > b) { tmp = a; a = b; b = tmp; }
    if (b > c) { tmp = b; b = c; c = tmp; }
    if (a > b) { tmp = a; a = b; b = tmp; }
    return b;
}

void PathFusion_Init(void)
{
    (void)memset(&fusion, 0, sizeof(fusion));
}

bool PathFusion_CalibrateSample(float gyro_z_deg_s)
{
    if (fusion.calib_count < PATH_FUSION_CALIB_SAMPLES)
    {
        fusion.calib_sum += gyro_z_deg_s;
        fusion.calib_count++;
    }

    if (fusion.calib_count >= PATH_FUSION_CALIB_SAMPLES)
    {
        /* 附加零偏 = -均值(抵消 IMU 内置零偏修正后的残余误差) */
        fusion.zero_offset_deg_s =
            -(fusion.calib_sum / (float)fusion.calib_count);
        return true;
    }

    return false;
}

void PathFusion_Predict(float gyro_z_deg_s, float dt_s)
{
    float rate_deg_s = (gyro_z_deg_s + fusion.zero_offset_deg_s) *
                       PATH_GYRO_SIGN;

    fusion.yaw += rate_deg_s * DEG2RAD * dt_s;
    fusion.yaw = PathWrapAngle(fusion.yaw);
}

bool PathFusion_UpdateUpper(float x_m, float y_m, float yaw_rad,
                            uint32_t now_ms)
{
    float med_x;
    float med_y;
    float dx;
    float dy;
    float yaw_err_rad;

    fusion.upper_frames++;
    fusion.last_upper_ms = now_ms;

    /* xy 跳变门限(>30cm 整帧拒绝):用原始样本与融合值比较,
     * 通过后才进入中值窗口 —— 离群帧不会污染窗口,不会造成冻结。 */
    if (fusion.have_upper)
    {
        dx = x_m - fusion.x;
        dy = y_m - fusion.y;
        if (sqrtf(dx * dx + dy * dy) > PATH_FUSION_XY_GATE_M)
        {
            fusion.xy_rejects++;
            return false;
        }
    }

    /* 中值滤波:吸收 20cm 量级的单帧跳变(未超 30cm 门限的离群帧) */
    fusion.median_x[fusion.median_i] = x_m;
    fusion.median_y[fusion.median_i] = y_m;
    fusion.median_i = (uint8_t)((fusion.median_i + 1U) %
                                PATH_FUSION_MEDIAN_WIN);
    if (fusion.median_fill < PATH_FUSION_MEDIAN_WIN)
    {
        fusion.median_fill++;
    }
    med_x = median3(fusion.median_x, fusion.median_fill);
    med_y = median3(fusion.median_y, fusion.median_fill);

    /* yaw 门限(与积分预测差 >20° 拒绝 yaw 分量,xy 照常融合) */
    yaw_err_rad = fabsf(PathWrapAngle(yaw_rad - fusion.yaw));
    if (fusion.have_upper &&
        (yaw_err_rad > PATH_FUSION_YAW_GATE_DEG * DEG2RAD))
    {
        fusion.yaw_rejects++;
    }
    else
    {
        /* yaw 低通拉回 */
        fusion.yaw += PATH_FUSION_YAW_GAIN *
                      PathWrapAngle(yaw_rad - fusion.yaw);
    }

    /* xy 直接覆盖 */
    fusion.x = med_x;
    fusion.y = med_y;
    fusion.have_upper = true;

    return true;
}

bool PathFusion_IsUpperLost(uint32_t now_ms)
{
    if (!fusion.have_upper)
    {
        /* 尚未收到过任何上位机数据:在等待起点阶段不算丢失 */
        return false;
    }
    return (uint32_t)(now_ms - fusion.last_upper_ms) >
           PATH_FUSION_UPPER_TIMEOUT_MS;
}

void PathFusion_Get(float *x_m, float *y_m, float *yaw_rad)
{
    if (x_m != NULL)
    {
        *x_m = fusion.x;
    }
    if (y_m != NULL)
    {
        *y_m = fusion.y;
    }
    if (yaw_rad != NULL)
    {
        *yaw_rad = fusion.yaw;
    }
}

void PathFusion_GetStats(uint32_t *xy_rejects, uint32_t *yaw_rejects,
                         uint32_t *upper_frames)
{
    if (xy_rejects != NULL)
    {
        *xy_rejects = fusion.xy_rejects;
    }
    if (yaw_rejects != NULL)
    {
        *yaw_rejects = fusion.yaw_rejects;
    }
    if (upper_frames != NULL)
    {
        *upper_frames = fusion.upper_frames;
    }
}
