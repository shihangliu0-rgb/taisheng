#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"

#define REMOTE_SWITCH_COUNT 6U
#define REMOTE_KEY_COUNT 12U
#define REMOTE_SHOULDER_MAX 1000U
#define REMOTE_SHOULDER_TRIGGER_THRESHOLD 600U
#define REMOTE_LORA_CONFIG_SIZE 6U

typedef enum
{
    REMOTE_LORA_CONFIG_NOT_STARTED = 0U,
    REMOTE_LORA_CONFIGURING,
    REMOTE_LORA_CONFIG_READY,
    REMOTE_LORA_CONFIG_AUX_TIMEOUT,
    REMOTE_LORA_CONFIG_UART_ERROR,
    REMOTE_LORA_CONFIG_READ_ERROR,
    REMOTE_LORA_CONFIG_WRITE_ERROR,
    REMOTE_LORA_CONFIG_VERIFY_ERROR,
    REMOTE_LORA_CONFIG_NORMAL_MODE_TIMEOUT
} remote_lora_config_status_t;

typedef struct
{
    uint16_t left_shoulder;                 /* 左肩键幅度，0 到 1000 */
    uint16_t right_shoulder;                /* 右肩键幅度，0 到 1000 */
    uint8_t left_x;                         /* 左摇杆 X，0 到 255 */
    uint8_t left_y;                         /* 左摇杆 Y，0 到 255 */
    uint8_t right_x;                        /* 右摇杆 X，0 到 255 */
    uint8_t right_y;                        /* 右摇杆 Y，0 到 255 */
    uint8_t switch_state[REMOTE_SWITCH_COUNT]; /* 六个开关状态 */
    uint8_t key_state[REMOTE_KEY_COUNT];        /* 十二个按键状态 */
} remote_data_t;

extern volatile remote_data_t remote_data;
extern volatile uint32_t remote_tx_error_count;
extern volatile uint32_t remote_tx_frame_count;
extern volatile uint32_t remote_tx_busy_skip_count;
extern volatile uint32_t remote_tx_config_skip_count;
extern volatile uint32_t remote_lora_config_attempts;
extern volatile remote_lora_config_status_t remote_lora_config_status;
extern volatile uint8_t remote_lora_config_readback[REMOTE_LORA_CONFIG_SIZE];

void Remote_LoRaInit(void);
uint8_t Remote_LoRaReady(void);

/**
 * @brief 按固定周期交替发送 ID 1 和 ID 2 的独立遥控短帧
 * @retval None
 */
void Remote_Send(void);

#endif
