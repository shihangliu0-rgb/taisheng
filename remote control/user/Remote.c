#include "Remote.h"
#include "usart.h"

#define REMOTE_FRAME_HEADER_0       0xA5U
#define REMOTE_FRAME_HEADER_1       0x5AU
#define REMOTE_FRAME_HEADER_SIZE    2U
#define REMOTE_LOCAL_PAYLOAD_SIZE   6U
#define REMOTE_SECOND_PAYLOAD_SIZE  2U
#define REMOTE_TX_BUFFER_SIZE       (REMOTE_FRAME_HEADER_SIZE + \
                                     REMOTE_LOCAL_PAYLOAD_SIZE + \
                                     REMOTE_SECOND_PAYLOAD_SIZE)
/* 20 Hz leaves the largest airtime margin for the externally configured LoRa link. */
#define REMOTE_TX_PERIOD_MS         50U
#define REMOTE_SHOULDER_RELEASE_THRESHOLD 550U
#define REMOTE_PE0_SWITCH_INDEX     3U
#define REMOTE_PD6_SWITCH_INDEX     4U
#define REMOTE_PE0_SWITCH_BIT       (1U << 0U)
#define REMOTE_PD6_SWITCH_BIT       (1U << 1U)

/* 遥控器输入数据，供后续下位机通信模块读取 */
volatile remote_data_t remote_data;
volatile uint32_t remote_tx_error_count;
volatile uint32_t remote_tx_frame_count;
volatile uint32_t remote_tx_busy_skip_count;

static uint32_t remote_next_tx_ms;
static uint8_t left_shoulder_pressed;
static uint8_t right_shoulder_pressed;
static uint8_t remote_tx_buffer[REMOTE_TX_BUFFER_SIZE];

void Remote_TransmitInit(void)
{
    left_shoulder_pressed = 0U;
    right_shoulder_pressed = 0U;
    remote_tx_error_count = 0U;
    remote_tx_frame_count = 0U;
    remote_tx_busy_skip_count = 0U;
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
    remote_next_tx_ms = HAL_GetTick();
}

static uint8_t Remote_UpdateShoulder(uint16_t value, uint8_t pressed)
{
    if (pressed != 0U)
    {
        return (value > REMOTE_SHOULDER_RELEASE_THRESHOLD) ? 1U : 0U;
    }
    return (value >= REMOTE_SHOULDER_TRIGGER_THRESHOLD) ? 1U : 0U;
}

void Remote_Send(void)
{
    uint8_t local_buttons = 0U;
    uint8_t second_keys = 0U;
    uint8_t second_switches = 0U;
    uint8_t i;
    uint32_t now_ms = HAL_GetTick();

    if ((int32_t)(now_ms - remote_next_tx_ms) < 0)
    {
        return;
    }
    remote_next_tx_ms += REMOTE_TX_PERIOD_MS;
    if ((int32_t)(now_ms - remote_next_tx_ms) >= 0)
    {
        remote_next_tx_ms = now_ms + REMOTE_TX_PERIOD_MS;
    }

    if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET)
    {
        remote_tx_busy_skip_count++;
        return;
    }
    if (huart6.gState != HAL_UART_STATE_READY)
    {
        remote_tx_busy_skip_count++;
        return;
    }

    left_shoulder_pressed = Remote_UpdateShoulder(remote_data.left_shoulder,
                                                   left_shoulder_pressed);
    right_shoulder_pressed = Remote_UpdateShoulder(remote_data.right_shoulder,
                                                    right_shoulder_pressed);
    for (i = 0U; i < 6U; i++)
    {
        local_buttons |= (uint8_t)(remote_data.key_state[i] << i);
        second_keys |= (uint8_t)(remote_data.key_state[i + 6U] << i);
        if ((i != REMOTE_PE0_SWITCH_INDEX) &&
            (i != REMOTE_PD6_SWITCH_INDEX))
        {
            second_switches |= (uint8_t)(remote_data.switch_state[i] << i);
        }
    }
    local_buttons |= (uint8_t)(left_shoulder_pressed << 6U);
    local_buttons |= (uint8_t)(right_shoulder_pressed << 7U);

    remote_tx_buffer[0] = REMOTE_FRAME_HEADER_0;
    remote_tx_buffer[1] = REMOTE_FRAME_HEADER_1;
    remote_tx_buffer[2] = remote_data.left_x;
    remote_tx_buffer[3] = remote_data.left_y;
    remote_tx_buffer[4] = remote_data.right_x;
    remote_tx_buffer[5] = remote_data.right_y;
    remote_tx_buffer[6] = local_buttons;
    remote_tx_buffer[7] =
        (uint8_t)((remote_data.switch_state[REMOTE_PE0_SWITCH_INDEX] != 0U)
                  ? REMOTE_PE0_SWITCH_BIT : 0U);
    remote_tx_buffer[7] |=
        (uint8_t)((remote_data.switch_state[REMOTE_PD6_SWITCH_INDEX] != 0U)
                  ? REMOTE_PD6_SWITCH_BIT : 0U);
    remote_tx_buffer[8] = second_keys;
    remote_tx_buffer[9] = second_switches;

    if (HAL_UART_Transmit_IT(&huart6, remote_tx_buffer,
                             REMOTE_TX_BUFFER_SIZE) == HAL_OK)
    {
        remote_tx_frame_count++;
    }
    else
    {
        remote_tx_error_count++;
    }
}
