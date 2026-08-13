/**
 ******************************************************************************
 * @file    path_yaw_lock.h
 * @brief   航向锁定控制器(对应 yaw_lock.py)
 *
 * 依赖: path_config.h
 * 关键算法: 死区 ±1° 不输出;小误差 Kp=1.5、大误差 Kp=3.0 分段增益;
 *           速度自适应限幅 w_max = max(0.3, w_base - w_slope * speed)。
 ******************************************************************************
 */
#ifndef PATH_YAW_LOCK_H
#define PATH_YAW_LOCK_H

/**
 * @brief 计算航向锁定旋转指令
 * @param yaw_rad    融合 yaw(用户约定:0 朝 +y)
 * @param speed_ms   当前平移速度(m/s),用于自适应限幅
 * @retval w 指令(rad/s,正值逆时针),由调用方做 slew-rate 限幅
 */
float PathYawLock_Step(float yaw_rad, float speed_ms);

#endif /* PATH_YAW_LOCK_H */
