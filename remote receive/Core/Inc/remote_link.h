#ifndef REMOTE_LINK_H
#define REMOTE_LINK_H

#include "main.h"

#define REMOTE_LINK_LORA_CONFIG_SIZE 6U

typedef enum
{
    REMOTE_LINK_LORA_CONFIG_NOT_STARTED = 0U,
    REMOTE_LINK_LORA_CONFIGURING,
    REMOTE_LINK_LORA_CONFIG_READY,
    REMOTE_LINK_LORA_CONFIG_AUX_TIMEOUT,
    REMOTE_LINK_LORA_CONFIG_UART_ERROR,
    REMOTE_LINK_LORA_CONFIG_READ_ERROR,
    REMOTE_LINK_LORA_CONFIG_WRITE_ERROR,
    REMOTE_LINK_LORA_CONFIG_VERIFY_ERROR,
    REMOTE_LINK_LORA_CONFIG_NORMAL_MODE_TIMEOUT
} remote_link_lora_config_status_t;

extern volatile uint32_t remote_link_uart_errors;
extern volatile uint32_t remote_link_rx_bytes;
extern volatile uint32_t remote_link_rx_callbacks;
extern volatile uint32_t remote_link_rx_arm_errors;
extern volatile uint32_t remote_link_forward_errors;
extern volatile uint32_t remote_link_forward_overflows;
extern volatile uint32_t remote_link_forwarded_bytes;
extern volatile uint32_t remote_link_return_errors;
extern volatile uint32_t remote_link_return_overflows;
extern volatile uint32_t remote_link_returned_bytes;
extern volatile uint32_t remote_link_lora_config_attempts;
extern volatile remote_link_lora_config_status_t remote_link_lora_config_status;
extern volatile uint8_t remote_link_lora_config_readback[REMOTE_LINK_LORA_CONFIG_SIZE];

void RemoteLink_Init(void);
void RemoteLink_ForwardRawData(void);
uint8_t RemoteLink_LoRaReady(void);

#endif
