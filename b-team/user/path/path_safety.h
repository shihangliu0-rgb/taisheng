#ifndef PATH_SAFETY_H
#define PATH_SAFETY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** d_safe = d_base + v*t_reaction + v^2/(2*a_brake)。 */
float PathSafety_RequiredDistance(float speed_mps,
                                  float base_distance_m,
                                  float reaction_time_s,
                                  float brake_deceleration_mps2);

/** 根据可用距离反解上述公式，得到允许继续朝障碍运动的最大速度。 */
float PathSafety_MaxAllowedSpeed(float measured_distance_m,
                                 float base_distance_m,
                                 float reaction_time_s,
                                 float brake_deceleration_mps2);

/**
 * 将有符号的单轴指令绝对值限制到 allowed_speed_mps；方向保持不变。
 * command_to_mps 是 1 个底盘平移指令单位对应的 m/s。
 */
int16_t PathSafety_LimitAxisCommand(int16_t command,
                                    float allowed_speed_mps,
                                    float command_to_mps);

/**
 * 按平移速度模长等比例缩放 vx/vy，返回缩放是否发生。
 */
int PathSafety_LimitVectorCommand(int16_t *vx, int16_t *vy,
                                  float allowed_speed_mps,
                                  float command_to_mps);

#ifdef __cplusplus
}
#endif

#endif /* PATH_SAFETY_H */
