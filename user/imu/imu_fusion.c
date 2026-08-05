/**
 * @file    imu_fusion.c
 * @brief   编码器(VESC CAN 轮速) 与 IMU 惯导速度的延迟自适应加权融合
 * @see     imu_fusion.h 设计说明
 */

#include "imu_fusion.h"

#include "imu.h"            /* Imu_GetVelX/Y, Imu_GetYaw */
#include "chassis_main.h"   /* Chassis_GetStatus, CHASSIS_WHEEL_* */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define FUSION_TWO_PI_OVER_60   (0.10471975511959795f)   /* 2π/60 */
#define FUSION_DEG2RAD          (0.017453292519943295f)
#define FUSION_DT_MAX_S         (0.02f)                  /* 单步最大 dt，防 hiatus 跳变 */
#define FUSION_AGE_NODATA       (0xFFFFFFFFu)            /* 无编码器数据时的年龄哨兵 */

/* e-rpm -> 轮缘线速度 m/s */
static float erpm_to_mps(int32_t erpm)
{
    float mech_rpm = (float)erpm / (float)FUSION_POLE_PAIRS / FUSION_GEAR_RATIO;
    return mech_rpm * FUSION_TWO_PI_OVER_60 * FUSION_WHEEL_RADIUS_M;
}

/* ---------------- 状态 ---------------- */
static float    w_enc;            /* 当前编码器权重 [0, FUSION_W_MAX] */
static bool     offline;          /* 编码器是否掉线(停用) */
static uint32_t offline_cnt;      /* 连续 age>1s 计数 */
static uint32_t reconnect_cnt;    /* 掉线后连续新鲜计数 */
static uint32_t last_update_ms;
static uint32_t enc_age_ms;       /* 编码器数据年龄(诊断) */

static float fused_vx, fused_vy;          /* 融合后世界系速度 m/s */
static float fused_posx, fused_posy;      /* 融合后世界系位置 m */
static float enc_world_vx, enc_world_vy;  /* 最近一次编码器世界系速度 m/s */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

void ImuFusion_Reset(void)
{
    w_enc = 0.0f;
    offline = false;
    offline_cnt = 0U;
    reconnect_cnt = 0U;
    last_update_ms = 0U;
    enc_age_ms = FUSION_AGE_NODATA;
    fused_vx = 0.0f; fused_vy = 0.0f;
    fused_posx = 0.0f; fused_posy = 0.0f;
    enc_world_vx = 0.0f; enc_world_vy = 0.0f;
}

void ImuFusion_ResetPosition(void)
{
    fused_posx = 0.0f;
    fused_posy = 0.0f;
}

/* 读 4 轮反馈，算车体速度与世界系速度；返回是否所有轮在线 */
static bool read_encoder(uint32_t now_ms)
{
    vesc_motor_status_t s_lf, s_rf, s_lr, s_rr;
    float v_lf, v_rf, v_lr, v_rr;
    float body_vx, body_vy;
    float yaw_rad, c, s;

    if (!Chassis_GetStatus(CHASSIS_WHEEL_LF, &s_lf) ||
        !Chassis_GetStatus(CHASSIS_WHEEL_RF, &s_rf) ||
        !Chassis_GetStatus(CHASSIS_WHEEL_LR, &s_lr) ||
        !Chassis_GetStatus(CHASSIS_WHEEL_RR, &s_rr))
    {
        return false;   /* 底盘未就绪 */
    }

    /* 任一轮掉线则逆运动学不完整，视为编码器不可用 */
    if (!s_lf.online || !s_rf.online || !s_lr.online || !s_rr.online)
    {
        return false;
    }

    /* 最新接收时间戳 -> 数据年龄 */
    {
        uint32_t freshest = s_lf.last_rx_ms;
        if ((int32_t)(s_rf.last_rx_ms - freshest) > 0) { freshest = s_rf.last_rx_ms; }
        if ((int32_t)(s_lr.last_rx_ms - freshest) > 0) { freshest = s_lr.last_rx_ms; }
        if ((int32_t)(s_rr.last_rx_ms - freshest) > 0) { freshest = s_rr.last_rx_ms; }
        enc_age_ms = (uint32_t)((int32_t)now_ms - (int32_t)freshest);
    }

    /* 逆运动学(对应 chassis 正运动学: LF=vx+vy-rot, RF=vx-vy-rot, LR=vx-vy+rot, RR=vx+vy+rot) */
    v_lf = erpm_to_mps(s_lf.actual_rpm);
    v_rf = erpm_to_mps(s_rf.actual_rpm);
    v_lr = erpm_to_mps(s_lr.actual_rpm);
    v_rr = erpm_to_mps(s_rr.actual_rpm);
    body_vx = (v_lf + v_rf + v_lr + v_rr) * 0.25f * FUSION_BODY_VX_SIGN;          /* 车体 X(右) */
    body_vy = ((v_lf + v_rr) - (v_rf + v_lr)) * 0.25f * FUSION_BODY_VY_SIGN;       /* 车体 Y(前) */

    /* 经 IMU 偏航角转世界系 */
    yaw_rad = Imu_GetYaw() * FUSION_DEG2RAD * FUSION_YAW_SIGN;
    c = cosf(yaw_rad);
    s = sinf(yaw_rad);
    enc_world_vx = body_vx * c - body_vy * s;
    enc_world_vy = body_vx * s + body_vy * c;
    return true;
}

void ImuFusion_Update(uint32_t now_ms)
{
    float dt_s;
    float target_w;
    bool enc_ok;
    float imu_vx, imu_vy;

    /* dt */
    if (last_update_ms == 0U)
    {
        dt_s = 0.0f;
    }
    else
    {
        dt_s = (float)((int32_t)now_ms - (int32_t)last_update_ms) * 0.001f;
        if (dt_s < 0.0f) { dt_s = 0.0f; }
        if (dt_s > FUSION_DT_MAX_S) { dt_s = FUSION_DT_MAX_S; }
    }
    last_update_ms = now_ms;

    /* 读编码器；不可用则年龄置哨兵(强制走掉线路径) */
    enc_ok = read_encoder(now_ms);
    if (!enc_ok)
    {
        enc_age_ms = FUSION_AGE_NODATA;
        enc_world_vx = 0.0f;
        enc_world_vy = 0.0f;
    }

    /* -------- 延迟自适应权重状态机 -------- */
    if (enc_age_ms > FUSION_OFFLINE_MS)        /* age > 1s */
    {
        reconnect_cnt = 0U;
        if (offline_cnt < 0xFFFFFFFFu) { offline_cnt++; }
        if (offline_cnt >= FUSION_OFFLINE_CONFIRM) { offline = true; }
        target_w = 0.0f;
    }
    else                                        /* age <= 1s */
    {
        offline_cnt = 0U;
        if (offline)
        {
            /* 掉线后须延迟可控(新鲜)持续若干周期才恢复 */
            if (enc_age_ms <= FUSION_FRESH_MS)
            {
                if (reconnect_cnt < 0xFFFFFFFFu) { reconnect_cnt++; }
                if (reconnect_cnt >= FUSION_RECONNECT_CONFIRM)
                {
                    offline = false;
                    reconnect_cnt = 0U;
                }
            }
            else
            {
                reconnect_cnt = 0U;
            }
            target_w = 0.0f;   /* 恢复前权重为 0 */
        }
        else if (enc_age_ms <= FUSION_FRESH_MS)
        {
            target_w = FUSION_W_MAX;            /* 新鲜 -> 上限 0.5 */
        }
        else
        {
            /* FRESH_MS < age <= OFFLINE_MS：线性下降 */
            float f = 1.0f - (float)(enc_age_ms - FUSION_FRESH_MS) /
                      (float)(FUSION_OFFLINE_MS - FUSION_FRESH_MS);
            target_w = FUSION_W_MAX * clampf(f, 0.0f, 1.0f);
        }
    }

    /* 权重向目标值限速爬升/下降 */
    {
        float step = FUSION_W_RAMP_PER_S * dt_s;
        if (w_enc < target_w) { w_enc += step; if (w_enc > target_w) { w_enc = target_w; } }
        else if (w_enc > target_w) { w_enc -= step; if (w_enc < target_w) { w_enc = target_w; } }
    }
    if (offline || !enc_ok) { w_enc = 0.0f; }
    w_enc = clampf(w_enc, 0.0f, FUSION_W_MAX);

    /* -------- 融合：fused = w_enc*enc + (1-w_enc)*imu -------- */
    imu_vx = Imu_GetVelX();
    imu_vy = Imu_GetVelY();
    fused_vx = w_enc * enc_world_vx + (1.0f - w_enc) * imu_vx;
    fused_vy = w_enc * enc_world_vy + (1.0f - w_enc) * imu_vy;

    /* 位置积分 */
    fused_posx += fused_vx * dt_s;
    fused_posy += fused_vy * dt_s;
}

/* ---------------- getters ---------------- */
float ImuFusion_GetVelX(void) { return fused_vx; }
float ImuFusion_GetVelY(void) { return fused_vy; }
float ImuFusion_GetPosX(void) { return fused_posx; }
float ImuFusion_GetPosY(void) { return fused_posy; }
float ImuFusion_GetEncoderWeight(void) { return w_enc; }
uint8_t ImuFusion_IsEncoderOffline(void) { return offline ? 1U : 0U; }
uint32_t ImuFusion_GetEncoderAgeMs(void)
{
    return (enc_age_ms == FUSION_AGE_NODATA) ? FUSION_OFFLINE_MS + 1U : enc_age_ms;
}
