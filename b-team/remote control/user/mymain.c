#include "mymain.h"
#include "RemoteInput.h"
#include "usart.h"

#define VOFA_CHANNEL_NUM 8U

typedef struct
{
    float channel[VOFA_CHANNEL_NUM];
    uint8_t tail[4];
} vofa_frame_t;

static vofa_frame_t debug_frame;

/**
 * @brief 初始化遥控器输入模块
 * @param None
 * @retval None
 */
void MyMain_Init(void)
{
    Remote_Init();
    Remote_TransmitInit();
}

/**
 * @brief 通过 USART1 输出 VOFA+ JustFloat 数据
 * @param None
 * @retval None
 */
void MyMain_Debug(void)
{
    if (huart1.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    /* 前六路输出原始 ADC，便于现场检查接线和采样状态。 */
    debug_frame.channel[0] = (float)remote_shoulder_adc[0];
    debug_frame.channel[1] = (float)remote_shoulder_adc[1];
    debug_frame.channel[2] = (float)remote_axis_adc[2];
    debug_frame.channel[3] = (float)remote_axis_adc[3];
    debug_frame.channel[4] = (float)remote_axis_adc[0];
    debug_frame.channel[5] = (float)remote_axis_adc[1];
    debug_frame.channel[6] = (float)remote_data.switch_state[0];
    debug_frame.channel[7] = (float)remote_data.key_state[0];
    debug_frame.tail[0] = 0x00U;
    debug_frame.tail[1] = 0x00U;
    debug_frame.tail[2] = 0x80U;
    debug_frame.tail[3] = 0x7FU;

    (void)HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&debug_frame,
                                (uint16_t)sizeof(debug_frame));
}
