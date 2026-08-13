#include "mymain.h"
#include "RemoteInput.h"
#include "usart.h"

#define DEBUG_UART_TIMEOUT_MS 100U
#define VOFA_CHANNEL_NUM 8U

typedef struct
{
    float channel[VOFA_CHANNEL_NUM];
    uint8_t tail[4];
} vofa_frame_t;

/**
 * @brief 初始化遥控器输入模块
 * @param None
 * @retval None
 */
void MyMain_Init(void)
{
    Remote_Init();
}

/**
 * @brief 通过 USART1 输出 VOFA+ JustFloat 数据
 * @param None
 * @retval None
 */
void MyMain_Debug(void)
{
    vofa_frame_t frame;

    /* 前六路输出原始 ADC，便于现场检查接线和采样状态。 */
    frame.channel[0] = (float)remote_shoulder_adc[0];
    frame.channel[1] = (float)remote_shoulder_adc[1];
    frame.channel[2] = (float)remote_axis_adc[2];
    frame.channel[3] = (float)remote_axis_adc[3];
    frame.channel[4] = (float)remote_axis_adc[0];
    frame.channel[5] = (float)remote_axis_adc[1];
    frame.channel[6] = (float)remote_data.switch_state[0];
    frame.channel[7] = (float)remote_data.key_state[0];
    frame.tail[0] = 0x00U;
    frame.tail[1] = 0x00U;
    frame.tail[2] = 0x80U;
    frame.tail[3] = 0x7FU;

    HAL_UART_Transmit(&huart1, (uint8_t *)&frame, sizeof(frame), DEBUG_UART_TIMEOUT_MS);
}
