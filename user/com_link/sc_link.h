#ifndef SC_LINK_H
#define SC_LINK_H

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define SC_LINK_BAUDRATE             921600U
#define SC_LINK_RX_BUFFER_SIZE       256U
#define SC_LINK_TX_BUFFER_SIZE       32U
#define SC_LINK_DATA_TIMEOUT_MS      500U
#define SC_LINK_STATUS_PERIOD_MS     100U

#define SC_LINK_FLAG_BLOCK_VALID     (1U << 0U)
#define SC_LINK_FLAG_BALL_VALID      (1U << 2U)
#define SC_LINK_FLAG_POSE_VALID      (1U << 0U)

typedef struct
{
    uint8_t sequence;
    uint8_t flags;
    uint32_t timestamp_ms;
    uint32_t received_ms;
    bool block_valid;
    bool ball_valid;
    float block_x_m;
    float block_y_m;
    float block_z_m;
    float ball_x_m;
    float ball_y_m;
    float ball_z_m;
} sc_link_perception_t;

typedef struct
{
    uint8_t sequence;
    uint8_t flags;
    uint32_t timestamp_ms;
    uint32_t received_ms;
    bool valid;
    float field_x_m;
    float field_y_m;
    float field_z_m;
    float field_yaw;
} sc_link_pose_t;

extern volatile uint32_t sc_link_rx_bytes;
extern volatile uint32_t sc_link_uart_error_count;
extern volatile uint32_t sc_link_tx_error_count;
extern volatile uint32_t sc_link_valid_frame_count;
extern volatile uint32_t sc_link_invalid_frame_count;

/** Initialize the UART8 small-computer link. */
HAL_StatusTypeDef ScLink_Init(UART_HandleTypeDef *uart);

/** Drain the DMA ring, parse frames, expire stale data, and send status. */
void ScLink_Run(void);

/** Copy the newest perception frame. */
bool ScLink_GetPerception(sc_link_perception_t *perception);

/** Copy the newest field-pose frame. */
bool ScLink_GetPose(sc_link_pose_t *pose);

/** Set the state and error bytes returned in controller status frames. */
void ScLink_SetStatus(uint8_t state, uint8_t error);

void ScLink_HandleTxCplt(UART_HandleTypeDef *uart);
void ScLink_HandleUartError(UART_HandleTypeDef *uart);

#endif
