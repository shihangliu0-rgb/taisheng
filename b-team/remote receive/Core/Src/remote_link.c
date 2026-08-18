#include "remote_link.h"
#include "usart.h"

volatile uint32_t remote_link_uart_errors;
volatile uint32_t remote_link_rx_bytes;
volatile uint32_t remote_link_rx_callbacks;
volatile uint32_t remote_link_rx_arm_errors;
volatile uint32_t remote_link_forward_errors;
volatile uint32_t remote_link_forward_overflows;
volatile uint32_t remote_link_forwarded_bytes;
volatile uint32_t remote_link_return_errors;
volatile uint32_t remote_link_return_overflows;
volatile uint32_t remote_link_returned_bytes;
volatile uint32_t remote_link_lora_config_attempts;
volatile remote_link_lora_config_status_t remote_link_lora_config_status;
volatile uint8_t remote_link_lora_config_readback[REMOTE_LINK_LORA_CONFIG_SIZE];

static uint8_t remote_link_rx_byte;
static uint8_t remote_link_return_rx_byte;
static uint8_t remote_link_lora_ready;
static uint8_t remote_link_forward_buffer[128U];
static volatile uint16_t remote_link_forward_head;
static volatile uint16_t remote_link_forward_tail;
static uint8_t remote_link_return_buffer[128U];
static volatile uint16_t remote_link_return_head;
static volatile uint16_t remote_link_return_tail;

#define E32_CONFIG_BAUDRATE 9600U
#define E32_NORMAL_BAUDRATE 115200U
#define E32_AUX_TIMEOUT_MS 3000U
#define E32_UART_TIMEOUT_MS 300U
#define E32_CONFIG_RETRY_COUNT 3U
#define E32_MODE_SETTLE_MS 10U
#define E32_COMMAND_SETTLE_MS 20U
#define REMOTE_LINK_FORWARD_BUFFER_SIZE 128U
#define REMOTE_LINK_FORWARD_CHUNK_SIZE 32U

/* Address 6, 115200 8N1, 9.6 kbps air rate,
 * transparent mode, channel 6. */
static const uint8_t remote_link_lora_expected_config[REMOTE_LINK_LORA_CONFIG_SIZE] = {
    0xC0U, 0x00U, 0x06U, 0x3CU, 0x06U, 0x44U
};

static uint8_t RemoteLink_LoRaWaitAuxHigh(uint32_t timeout_ms)
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

static HAL_StatusTypeDef RemoteLink_LoRaSetBaudrate(uint32_t baudrate)
{
    if (HAL_UART_DeInit(&huart3) != HAL_OK)
    {
        return HAL_ERROR;
    }
    huart3.Init.BaudRate = baudrate;
    return HAL_UART_Init(&huart3);
}

static void RemoteLink_LoRaFlushRx(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
    {
        (void)huart3.Instance->DR;
    }
}

static void RemoteLink_LoRaSaveReadback(const uint8_t *config)
{
    uint8_t i;

    for (i = 0U; i < REMOTE_LINK_LORA_CONFIG_SIZE; i++)
    {
        remote_link_lora_config_readback[i] = config[i];
    }
}

static uint8_t RemoteLink_LoRaConfigMatches(const uint8_t *config)
{
    uint8_t i;

    for (i = 0U; i < REMOTE_LINK_LORA_CONFIG_SIZE; i++)
    {
        if (config[i] != remote_link_lora_expected_config[i])
        {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t RemoteLink_LoRaReadConfig(uint8_t *config)
{
    uint8_t command[3] = {0xC1U, 0xC1U, 0xC1U};

    RemoteLink_LoRaFlushRx();
    if (HAL_UART_Transmit(&huart3, command, sizeof(command), E32_UART_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }
    if (HAL_UART_Receive(&huart3, config, REMOTE_LINK_LORA_CONFIG_SIZE,
                         E32_UART_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }
    RemoteLink_LoRaSaveReadback(config);
    return 1U;
}

static uint8_t RemoteLink_LoRaWriteConfig(void)
{
    uint8_t command[REMOTE_LINK_LORA_CONFIG_SIZE];
    uint8_t i;

    for (i = 0U; i < REMOTE_LINK_LORA_CONFIG_SIZE; i++)
    {
        command[i] = remote_link_lora_expected_config[i];
    }
    RemoteLink_LoRaFlushRx();
    if (HAL_UART_Transmit(&huart3, command, sizeof(command), E32_UART_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }
    HAL_Delay(E32_COMMAND_SETTLE_MS);
    return RemoteLink_LoRaWaitAuxHigh(E32_AUX_TIMEOUT_MS);
}

static uint8_t RemoteLink_LoRaEnterNormalMode(void)
{
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
    HAL_Delay(E32_MODE_SETTLE_MS);

    if (RemoteLink_LoRaWaitAuxHigh(E32_AUX_TIMEOUT_MS) == 0U)
    {
        return 0U;
    }
    return (RemoteLink_LoRaSetBaudrate(E32_NORMAL_BAUDRATE) == HAL_OK) ? 1U : 0U;
}

static void RemoteLink_LoRaConfigure(void)
{
    uint8_t config[REMOTE_LINK_LORA_CONFIG_SIZE];
    uint8_t attempt;
    uint8_t configured = 0U;

    remote_link_lora_ready = 0U;
    remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIGURING;
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_SET);

    if (RemoteLink_LoRaSetBaudrate(E32_CONFIG_BAUDRATE) != HAL_OK)
    {
        remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_UART_ERROR;
        (void)RemoteLink_LoRaEnterNormalMode();
        return;
    }
    HAL_Delay(E32_MODE_SETTLE_MS);
    if (RemoteLink_LoRaWaitAuxHigh(E32_AUX_TIMEOUT_MS) == 0U)
    {
        remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_AUX_TIMEOUT;
        (void)RemoteLink_LoRaEnterNormalMode();
        return;
    }

    for (attempt = 0U; attempt < E32_CONFIG_RETRY_COUNT; attempt++)
    {
        remote_link_lora_config_attempts++;
        if (RemoteLink_LoRaReadConfig(config) == 0U)
        {
            remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_READ_ERROR;
            HAL_Delay(50U);
            continue;
        }
        if (RemoteLink_LoRaConfigMatches(config) != 0U)
        {
            configured = 1U;
            break;
        }
        if (RemoteLink_LoRaWriteConfig() == 0U)
        {
            remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_WRITE_ERROR;
            HAL_Delay(50U);
            continue;
        }
        if ((RemoteLink_LoRaReadConfig(config) != 0U) &&
            (RemoteLink_LoRaConfigMatches(config) != 0U))
        {
            configured = 1U;
            break;
        }
        remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_VERIFY_ERROR;
        HAL_Delay(50U);
    }

    if (configured == 0U)
    {
        (void)RemoteLink_LoRaEnterNormalMode();
        return;
    }
    if (RemoteLink_LoRaEnterNormalMode() == 0U)
    {
        remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_NORMAL_MODE_TIMEOUT;
        return;
    }

    remote_link_lora_ready = 1U;
    remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_READY;
}

static void RemoteLink_QueueForwardByte(uint8_t byte)
{
    uint16_t head = remote_link_forward_head;
    uint16_t next = head + 1U;

    if (next >= REMOTE_LINK_FORWARD_BUFFER_SIZE)
    {
        next = 0U;
    }
    if (next == remote_link_forward_tail)
    {
        remote_link_forward_overflows++;
        return;
    }

    remote_link_forward_buffer[head] = byte;
    remote_link_forward_head = next;
}

static void RemoteLink_QueueReturnByte(uint8_t byte)
{
    uint16_t head = remote_link_return_head;
    uint16_t next = head + 1U;

    if (next >= REMOTE_LINK_FORWARD_BUFFER_SIZE)
    {
        next = 0U;
    }
    if (next == remote_link_return_tail)
    {
        remote_link_return_overflows++;
        return;
    }

    remote_link_return_buffer[head] = byte;
    remote_link_return_head = next;
}

static void RemoteLink_ArmLoraReceive(void)
{
    if (huart3.RxState == HAL_UART_STATE_BUSY_RX)
    {
        return;
    }
    if (HAL_UART_Receive_IT(&huart3, &remote_link_rx_byte, 1U) != HAL_OK)
    {
        remote_link_rx_arm_errors++;
    }
}

static void RemoteLink_ArmControllerReceive(void)
{
    if (huart2.RxState == HAL_UART_STATE_BUSY_RX)
    {
        return;
    }
    if (HAL_UART_Receive_IT(&huart2, &remote_link_return_rx_byte, 1U) != HAL_OK)
    {
        remote_link_rx_arm_errors++;
    }
}

void RemoteLink_Init(void)
{
    remote_link_uart_errors = 0U;
    remote_link_rx_bytes = 0U;
    remote_link_rx_callbacks = 0U;
    remote_link_rx_arm_errors = 0U;
    remote_link_forward_errors = 0U;
    remote_link_forward_overflows = 0U;
    remote_link_forwarded_bytes = 0U;
    remote_link_return_errors = 0U;
    remote_link_return_overflows = 0U;
    remote_link_returned_bytes = 0U;
    remote_link_lora_config_attempts = 0U;
    remote_link_lora_config_status = REMOTE_LINK_LORA_CONFIG_NOT_STARTED;
    remote_link_forward_head = 0U;
    remote_link_forward_tail = 0U;
    remote_link_return_head = 0U;
    remote_link_return_tail = 0U;
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
    RemoteLink_LoRaConfigure();
    if (RemoteLink_LoRaReady() != 0U)
    {
        RemoteLink_ArmLoraReceive();
        RemoteLink_ArmControllerReceive();
    }
}

uint8_t RemoteLink_LoRaReady(void)
{
    return remote_link_lora_ready;
}

void RemoteLink_ForwardRawData(void)
{
    uint8_t data[REMOTE_LINK_FORWARD_CHUNK_SIZE];
    uint8_t length = 0U;
    uint16_t head = remote_link_forward_head;
    uint16_t tail = remote_link_forward_tail;

    while ((tail != head) && (length < REMOTE_LINK_FORWARD_CHUNK_SIZE))
    {
        data[length++] = remote_link_forward_buffer[tail];
        tail++;
        if (tail >= REMOTE_LINK_FORWARD_BUFFER_SIZE)
        {
            tail = 0U;
        }
    }
    if ((length != 0U) &&
        (HAL_UART_Transmit(&huart2, data, length, 10U) == HAL_OK))
    {
        remote_link_forward_tail = tail;
        remote_link_forwarded_bytes += length;
    }
    else if (length != 0U)
    {
        remote_link_forward_errors++;
    }

    length = 0U;
    head = remote_link_return_head;
    tail = remote_link_return_tail;
    while ((tail != head) && (length < REMOTE_LINK_FORWARD_CHUNK_SIZE))
    {
        data[length++] = remote_link_return_buffer[tail];
        tail++;
        if (tail >= REMOTE_LINK_FORWARD_BUFFER_SIZE)
        {
            tail = 0U;
        }
    }
    if ((length == 0U) ||
        (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET))
    {
        return;
    }

    if (HAL_UART_Transmit(&huart3, data, length, 10U) == HAL_OK)
    {
        remote_link_return_tail = tail;
        remote_link_returned_bytes += length;
    }
    else
    {
        remote_link_return_errors++;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        remote_link_rx_callbacks++;
        if (RemoteLink_LoRaReady() != 0U)
        {
            remote_link_rx_bytes++;
            RemoteLink_QueueForwardByte(remote_link_rx_byte);
        }
        RemoteLink_ArmLoraReceive();
    }
    else if (huart->Instance == USART2)
    {
        remote_link_rx_callbacks++;
        RemoteLink_QueueReturnByte(remote_link_return_rx_byte);
        RemoteLink_ArmControllerReceive();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        remote_link_uart_errors++;
        if (RemoteLink_LoRaReady() != 0U)
        {
            RemoteLink_ArmLoraReceive();
        }
    }
    else if (huart->Instance == USART2)
    {
        remote_link_uart_errors++;
        RemoteLink_ArmControllerReceive();
    }
}
