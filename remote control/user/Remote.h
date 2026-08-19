#ifndef REMOTE_H
#define REMOTE_H

#include "main.h"

#define REMOTE_SWITCH_COUNT 6U
#define REMOTE_KEY_COUNT 12U
#define REMOTE_SHOULDER_MAX 1000U
#define REMOTE_SHOULDER_TRIGGER_THRESHOLD 600U

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

void Remote_TransmitInit(void);

/**
 * @brief 按固定周期发送包含两组数据的固定 10 字节帧
 * @retval None
 */
void Remote_Send(void);

#endif
