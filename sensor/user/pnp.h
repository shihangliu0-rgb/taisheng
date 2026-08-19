#ifndef PNP_H
#define PNP_H

#include "stm32f1xx_hal.h"

/* PNP_F 地址为二进制 1000000，PNP_B 地址为二进制 1000001。 */
#define PNP_ADDR_F 0x40U
#define PNP_ADDR_B 0x41U

extern uint16_t pnp_raw_f;             /* PNP_F 原始总线电压寄存器值。 */
extern uint16_t pnp_raw_b;             /* PNP_B 原始总线电压寄存器值。 */
extern uint16_t pnp_voltage_f_mv;      /* PNP_F 电压，单位 mV。 */
extern uint16_t pnp_voltage_b_mv;      /* PNP_B 电压，单位 mV。 */
extern uint8_t pnp_trigger_f;          /* PNP_F 是否超过触发阈值。 */
extern uint8_t pnp_trigger_b;          /* PNP_B 是否超过触发阈值。 */
extern HAL_StatusTypeDef pnp_state_f;  /* PNP_F 最近一次读取状态。 */
extern HAL_StatusTypeDef pnp_state_b;  /* PNP_B 最近一次读取状态。 */

/**
 * @brief 初始化 I2C1 上的两个 PNP 传感器
 * @param i2c I2C1 句柄
 * @retval HAL 状态
 */
HAL_StatusTypeDef PNP_Init(I2C_HandleTypeDef *i2c);

/**
 * @brief 读取两个 PNP 传感器的电压
 * @retval None
 */
void PNP_Update(void);

#endif
