#include "mymain.h"

#include "i2c.h"
#include "usart.h"

#define SAMPLE_TIME_MS     50U
#define RETRY_TIME_MS      1000U
#define DT35_FRAME_HEADER  0xAAU
#define PNP_FRAME_HEADER   0xABU
#define DT35_TX_TIMEOUT_MS 20U

dt35_data_t dt35_data_F; //前面的dt35  100000
dt35_data_t dt35_data_L; //左边的dt35  100001
HAL_StatusTypeDef dt35_state_F = HAL_ERROR;
HAL_StatusTypeDef dt35_state_L = HAL_ERROR;

static uint32_t sample_tick;
static uint32_t retry_tick;
static HAL_StatusTypeDef init_state;
static HAL_StatusTypeDef pnp_init_state;

static void sensor_send_value(uint8_t header, uint8_t address, uint16_t value)
{
    uint8_t frame[5];

    frame[0] = header;
    frame[1] = address;
    frame[2] = (uint8_t)value;
    frame[3] = (uint8_t)(value >> 8U);
    frame[4] = frame[0] ^ frame[1] ^ frame[2] ^ frame[3];
    (void)HAL_UART_Transmit(&huart1, frame, sizeof(frame),
                            DT35_TX_TIMEOUT_MS);
}

void MyMain_Init(void)
{
    init_state = DT35_Init(&hi2c2);
    pnp_init_state = PNP_Init(&hi2c1);
    dt35_state_F = HAL_ERROR;
    dt35_state_L = HAL_ERROR;
    sample_tick = HAL_GetTick();
    retry_tick = sample_tick;
}

void MyMain_Loop(void)
{
    uint32_t now;
    dt35_data_t data;

    now = HAL_GetTick();

    if (((init_state != HAL_OK) || (dt35_state_F != HAL_OK) ||
         (dt35_state_L != HAL_OK) || (pnp_init_state != HAL_OK) ||
         (pnp_state_f != HAL_OK) || (pnp_state_b != HAL_OK)) &&
        ((uint32_t)(now - retry_tick) >= RETRY_TIME_MS))
    {
        retry_tick = now;
        init_state = DT35_Init(&hi2c2);
        pnp_init_state = PNP_Init(&hi2c1);
    }

    if ((uint32_t)(now - sample_tick) < SAMPLE_TIME_MS)
    {
        return;
    }
    sample_tick = now;

    dt35_state_L = DT35_Read_41(&data);
    if (dt35_state_L == HAL_OK)
    {
        dt35_data_L = data;
        sensor_send_value(DT35_FRAME_HEADER, DT35_ADDR_L, data.distance_cm);
    }

    dt35_state_F = DT35_Read_40(&data);
    if (dt35_state_F == HAL_OK)
    {
        dt35_data_F = data;
        sensor_send_value(DT35_FRAME_HEADER, DT35_ADDR_F, data.distance_cm);
    }

    PNP_Update();

    if (pnp_state_f == HAL_OK)
    {
        sensor_send_value(PNP_FRAME_HEADER, PNP_ADDR_F, pnp_trigger_f);
    }
    if (pnp_state_b == HAL_OK)
    {
        sensor_send_value(PNP_FRAME_HEADER, PNP_ADDR_B, pnp_trigger_b);
    }
}
