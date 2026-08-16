#include "Remote.h"
#include "usart.h"

#include <string.h>

#define REMOTE_FRAME_HEADER_0       0xA5U
#define REMOTE_FRAME_HEADER_1       0x5AU
#define REMOTE_ID_LOCAL             0x01U
#define REMOTE_ID_SECONDARY         0x02U
#define REMOTE_LOCAL_PAYLOAD_SIZE   6U
#define REMOTE_SECOND_PAYLOAD_SIZE  2U
#define REMOTE_FRAME_OVERHEAD       3U
#define REMOTE_TX_BUFFER_SIZE       (REMOTE_FRAME_OVERHEAD + REMOTE_LOCAL_PAYLOAD_SIZE)
#define REMOTE_TX_PERIOD_MS         25U
#define REMOTE_SHOULDER_RELEASE_THRESHOLD 550U
#define REMOTE_PE0_SWITCH_INDEX     3U
#define E32_CONFIG_BAUDRATE         9600U
#define E32_NORMAL_BAUDRATE         115200U
#define E32_AUX_TIMEOUT_MS          3000U
#define E32_UART_TIMEOUT_MS         300U
#define E32_CONFIG_RETRY_COUNT      3U
#define E32_MODE_SETTLE_MS          10U
#define E32_COMMAND_SETTLE_MS       20U

/* 遥控器输入数据，供后续下位机通信模块读取 */
volatile remote_data_t remote_data;
volatile uint32_t remote_tx_error_count;
volatile uint32_t remote_tx_frame_count;
volatile uint32_t remote_tx_busy_skip_count;
volatile uint32_t remote_tx_config_skip_count;
volatile uint32_t remote_lora_config_attempts;
volatile remote_lora_config_status_t remote_lora_config_status;
volatile uint8_t remote_lora_config_readback[REMOTE_LORA_CONFIG_SIZE];

static uint8_t remote_next_target_id;
static uint32_t remote_next_tx_ms;
static uint8_t left_shoulder_pressed;
static uint8_t right_shoulder_pressed;
static uint8_t remote_lora_ready;
static uint8_t remote_tx_buffer[REMOTE_TX_BUFFER_SIZE];


static uint8_t Remote_LoRaWaitAuxHigh(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    do
    {
        if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_SET)
        {
            HAL_Delay(2U);
            if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_SET)
            {
                return 1U;
            }
        }
    } while ((uint32_t)(HAL_GetTick() - start) < timeout_ms);

    return 0U;
}

static HAL_StatusTypeDef Remote_LoRaSetBaudrate(uint32_t baudrate)
{
    if (HAL_UART_DeInit(&huart6) != HAL_OK)
    {
        return HAL_ERROR;
    }
    huart6.Init.BaudRate = baudrate;
    return HAL_UART_Init(&huart6);
}

static void Remote_LoRaFlushRx(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET)
    {
        (void)huart6.Instance->DR;
    }
}

static void Remote_LoRaSaveReadback(const uint8_t *config)
{
    uint8_t i;

    for (i = 0U; i < REMOTE_LORA_CONFIG_SIZE; i++)
    {
        remote_lora_config_readback[i] = config[i];
    }
}


static uint8_t Remote_LoRaReadConfig(uint8_t *config)
{
    uint8_t command[3] = {0xC1U, 0xC1U, 0xC1U};

    Remote_LoRaFlushRx();
    if (HAL_UART_Transmit(&huart6, command, sizeof(command),
                          E32_UART_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }
    if (HAL_UART_Receive(&huart6, config, REMOTE_LORA_CONFIG_SIZE,
                         E32_UART_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }
    Remote_LoRaSaveReadback(config);
    return 1U;
}



static uint8_t Remote_LoRaEnterNormalMode(void)
{
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
    HAL_Delay(E32_MODE_SETTLE_MS);

    if (Remote_LoRaWaitAuxHigh(E32_AUX_TIMEOUT_MS) == 0U)
    {
        return 0U;
    }
    return (Remote_LoRaSetBaudrate(E32_NORMAL_BAUDRATE) == HAL_OK) ? 1U : 0U;
}



void Remote_LoRaInit(void)
{
    remote_next_target_id = REMOTE_ID_LOCAL;
    left_shoulder_pressed = 0U;
    right_shoulder_pressed = 0U;
    remote_tx_error_count = 0U;
    remote_tx_frame_count = 0U;
    remote_tx_busy_skip_count = 0U;
    remote_tx_config_skip_count = 0U;
    remote_lora_config_attempts = 0U;
    remote_lora_config_status = REMOTE_LORA_CONFIG_NOT_STARTED;
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
    remote_lora_ready = Remote_LoRaWaitAuxHigh(E32_AUX_TIMEOUT_MS);
    remote_lora_config_status = (remote_lora_ready != 0U)
                                    ? REMOTE_LORA_CONFIG_READY
                                    : REMOTE_LORA_CONFIG_AUX_TIMEOUT;
    remote_next_tx_ms = HAL_GetTick();
}

uint8_t Remote_LoRaReady(void)
{
    return remote_lora_ready;
}

static uint8_t Remote_PackFrame(uint8_t *frame, uint8_t target_id,
                                 const uint8_t *payload, uint8_t payload_size)
{
    frame[0] = REMOTE_FRAME_HEADER_0;
    frame[1] = REMOTE_FRAME_HEADER_1;
    frame[2] = target_id;
    (void)memcpy(&frame[REMOTE_FRAME_OVERHEAD], payload, payload_size);
    return (uint8_t)(REMOTE_FRAME_OVERHEAD + payload_size);
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
    uint8_t payload[REMOTE_LOCAL_PAYLOAD_SIZE];
    uint8_t tx_length;
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

    if (Remote_LoRaReady() == 0U)
    {
        remote_tx_config_skip_count++;
        return;
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

    if (remote_next_target_id == REMOTE_ID_LOCAL)
    {
        uint8_t local_buttons = 0U;

        left_shoulder_pressed = Remote_UpdateShoulder(remote_data.left_shoulder,
                                                       left_shoulder_pressed);
        right_shoulder_pressed = Remote_UpdateShoulder(remote_data.right_shoulder,
                                                        right_shoulder_pressed);
        for (i = 0U; i < 6U; i++)
        {
            local_buttons |= (uint8_t)(remote_data.key_state[i] << i);
        }
        local_buttons |= (uint8_t)(left_shoulder_pressed << 6U);
        local_buttons |= (uint8_t)(right_shoulder_pressed << 7U);

        payload[0] = remote_data.left_x;
        payload[1] = remote_data.left_y;
        payload[2] = remote_data.right_x;
        payload[3] = remote_data.right_y;
        payload[4] = local_buttons;
        payload[5] = (uint8_t)(remote_data.switch_state[REMOTE_PE0_SWITCH_INDEX] & 1U);
        tx_length = Remote_PackFrame(remote_tx_buffer, REMOTE_ID_LOCAL,
                                     payload, REMOTE_LOCAL_PAYLOAD_SIZE);
    }
    else
    {
        uint8_t second_keys = 0U;
        uint8_t second_switches = 0U;

        for (i = 0U; i < 6U; i++)
        {
            second_keys |= (uint8_t)(remote_data.key_state[i + 6U] << i);
            if (i != REMOTE_PE0_SWITCH_INDEX)
            {
                second_switches |= (uint8_t)(remote_data.switch_state[i] << i);
            }
        }
        payload[0] = second_keys;
        payload[1] = second_switches;
        tx_length = Remote_PackFrame(remote_tx_buffer, REMOTE_ID_SECONDARY,
                                     payload, REMOTE_SECOND_PAYLOAD_SIZE);
    }
    if (HAL_UART_Transmit_IT(&huart6, remote_tx_buffer, tx_length) == HAL_OK)
    {
        remote_next_target_id = (remote_next_target_id == REMOTE_ID_LOCAL)
                                    ? REMOTE_ID_SECONDARY
                                    : REMOTE_ID_LOCAL;
        remote_tx_frame_count++;
    }
    else
    {
        remote_tx_error_count++;
    }
}
