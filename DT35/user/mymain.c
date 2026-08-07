#include "mymain.h"

#include "i2c.h"
#include "usart.h"

#define SAMPLE_TIME_MS     50U
#define RETRY_TIME_MS      1000U
#define DT35_FRAME_HEADER  0xAAU
#define DT35_TX_TIMEOUT_MS 20U

dt35_data_t dt35_data_40;
dt35_data_t dt35_data_41;
HAL_StatusTypeDef dt35_state_40 = HAL_ERROR;
HAL_StatusTypeDef dt35_state_41 = HAL_ERROR;

static uint32_t sample_tick;
static uint32_t retry_tick;
static HAL_StatusTypeDef init_state;

static void dt35_send_distance(uint8_t address, uint16_t distance_cm)
{
    uint8_t frame[5];

    frame[0] = DT35_FRAME_HEADER;
    frame[1] = address;
    frame[2] = (uint8_t)distance_cm;
    frame[3] = (uint8_t)(distance_cm >> 8U);
    frame[4] = frame[0] ^ frame[1] ^ frame[2] ^ frame[3];
    (void)HAL_UART_Transmit(&huart1, frame, sizeof(frame),
                            DT35_TX_TIMEOUT_MS);
}

void MyMain_Init(void)
{
    init_state = DT35_Init(&hi2c2);
    dt35_state_40 = HAL_ERROR;
    dt35_state_41 = HAL_ERROR;
    sample_tick = HAL_GetTick();
    retry_tick = sample_tick;
}

void MyMain_Loop(void)
{
    uint32_t now;
    dt35_data_t data;

    now = HAL_GetTick();

    if (((init_state != HAL_OK) || (dt35_state_40 != HAL_OK) ||
         (dt35_state_41 != HAL_OK)) &&
        ((uint32_t)(now - retry_tick) >= RETRY_TIME_MS))
    {
        retry_tick = now;
        init_state = DT35_Init(&hi2c2);
    }

    if ((uint32_t)(now - sample_tick) < SAMPLE_TIME_MS)
    {
        return;
    }
    sample_tick = now;

    dt35_state_41 = DT35_Read_41(&data);
    if (dt35_state_41 == HAL_OK)
    {
        dt35_data_41 = data;
        dt35_send_distance(DT35_ADDR_41, data.distance_cm);
    }

    dt35_state_40 = DT35_Read_40(&data);
    if (dt35_state_40 == HAL_OK)
    {
        dt35_data_40 = data;
        dt35_send_distance(DT35_ADDR_40, data.distance_cm);
    }
}
