#ifndef REMOTE_INPUT_H
#define REMOTE_INPUT_H

#include "Remote.h"

#define REMOTE_SHOULDER_NUM 2U  /* 肩键 ADC 通道数量 */
#define REMOTE_AXIS_NUM 4U      /* 摇杆 ADC 通道数量 */

typedef enum
{
    REMOTE_ADC_OK = 0,
    REMOTE_ADC1_ERROR,
    REMOTE_ADC2_ERROR
} remote_adc_state_t;

extern volatile uint16_t remote_shoulder_adc[REMOTE_SHOULDER_NUM];
extern volatile uint16_t remote_axis_adc[REMOTE_AXIS_NUM];
extern remote_adc_state_t remote_adc_state;

/**
 * @brief 初始化遥控器 ADC、按键和开关
 * @param None
 * @retval None
 */
void Remote_Init(void);

/**
 * @brief 更新摇杆、肩键、按键和开关数据
 * @param None
 * @retval None
 */
void Remote_Update(void);

#endif
