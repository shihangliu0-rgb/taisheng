#ifndef IMU_FUSION_H
#define IMU_FUSION_H

/**
 * @file    imu_fusion.h
 * @brief   编码器(VESC 经 CAN 反馈的轮速) 与 IMU 惯导速度的融合
 *
 * 思路：
 *  - 编码器线速度由 4 个 VESC 的 actual_rpm 反推(逆运动学) -> 车体系 -> 经 IMU 偏航角转世界系；
 *  - IMU 速度取 Imu_GetVelX/Y(世界系，来自双重积分，长距离会漂移)；
 *  - 二者按权重 w_enc 加权融合：fused = w_enc*v_enc + (1-w_enc)*v_imu。
 *
 * 延迟自适应权重(核心需求)：
 *  - VESC status 自带 last_rx_ms 时间戳(50Hz)，编码器数据年龄 age = now - 最新 last_rx_ms；
 *  - age 小(新鲜) -> 权重向上限 0.5 爬升；
 *  - age 偏大(延迟过大) -> 权重下降；
 *  - age 持续 > 1s(连续多次) -> 判干扰太大、编码器掉线，权重置 0、编码器停用；
 *  - 掉线后若重新稳定收到且延迟可控 -> 恢复，权重重新爬升(上限仍 0.5)。
 *
 * 不修改 imu_algo / vesc / chassis 任何代码，仅通过 getter 读取。
 */

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 *  物理参数 —— ★请按实车填写(影响 rpm->m/s 换算)
 *  VESC status 反馈的是电转速 e-rpm，需 /极对数 得机械转速，再 /减速比、乘轮周长。
 * ========================================================================== */
#ifndef FUSION_POLE_PAIRS
#define FUSION_POLE_PAIRS        21U      /* VESC 极对数，与 chassis 一致 */
#endif
#ifndef FUSION_GEAR_RATIO
#define FUSION_GEAR_RATIO        1.0f     /* 电机->轮 减速比，直驱填 1.0 */
#endif
#ifndef FUSION_WHEEL_RADIUS_M
#define FUSION_WHEEL_RADIUS_M    0.076f   /* 轮半径 m (轮径 152mm) */
#endif

/* ==========================================================================
 *  延迟 / 权重参数 —— VESC 反馈 50Hz(期望 20ms)
 * ========================================================================== */
#ifndef FUSION_FRESH_MS
#define FUSION_FRESH_MS          60U      /* age<=此值视为新鲜(约 3 帧)，权重向 0.5 升 */
#endif
#ifndef FUSION_OFFLINE_MS
#define FUSION_OFFLINE_MS        1000U    /* age>此值(>1s)开始累计掉线 */
#endif
#ifndef FUSION_OFFLINE_CONFIRM
#define FUSION_OFFLINE_CONFIRM   3U       /* 连续多少个周期 age>1s 判掉线(编码器停用) */
#endif
#ifndef FUSION_RECONNECT_CONFIRM
#define FUSION_RECONNECT_CONFIRM 5U       /* 掉线后连续多少周期新鲜(age<=FRESH)才恢复 */
#endif
#ifndef FUSION_W_MAX
#define FUSION_W_MAX             0.5f     /* 编码器权重上限 0.5 */
#endif
#ifndef FUSION_W_RAMP_PER_S
#define FUSION_W_RAMP_PER_S      1.0f     /* 权重升降速率(/s)，0->0.5 需 0.5s */
#endif

/* ==========================================================================
 *  轴向符号 —— ★IMU 与底盘安装方向对齐用，实车标定后可能需翻转
 *  body 系: X 向右、Y 向前(与 chassis 一致)；经 Imu_GetYaw 旋到世界系。
 * ========================================================================== */
#ifndef FUSION_BODY_VX_SIGN
#define FUSION_BODY_VX_SIGN      1.0f
#endif
#ifndef FUSION_BODY_VY_SIGN
#define FUSION_BODY_VY_SIGN      1.0f
#endif
#ifndef FUSION_YAW_SIGN
#define FUSION_YAW_SIGN          1.0f
#endif

/* ============================ API ============================ */
/* 复位融合器状态(权重、位置、掉线计数等) */
void ImuFusion_Reset(void);

/* 周期调用(建议 1ms，已挂在 ImuMain_Run1ms)：读编码器+IMU、更新权重与融合输出 */
void ImuFusion_Update(uint32_t now_ms);

/* 融合后世界系速度 / 位置 (m/s, m) */
float ImuFusion_GetVelX(void);
float ImuFusion_GetVelY(void);
float ImuFusion_GetPosX(void);
float ImuFusion_GetPosY(void);
void  ImuFusion_ResetPosition(void);

/* 诊断 */
float   ImuFusion_GetEncoderWeight(void);  /* 当前编码器权重 [0, 0.5] */
uint8_t ImuFusion_IsEncoderOffline(void);  /* 编码器是否已掉线 */
uint32_t ImuFusion_GetEncoderAgeMs(void);  /* 编码器数据年龄 ms(越小说明越新鲜) */

#ifdef __cplusplus
}
#endif
#endif /* IMU_FUSION_H */
