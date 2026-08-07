#include "dt35.h"

/* INA226 寄存器映射和 DT35 传感器固定范围。 */
#define INA226_REG_CONFIG   0x00U
#define INA226_REG_BUS      0x02U
#define INA226_REG_MAN_ID   0xFEU
#define INA226_REG_DIE_ID   0xFFU
#define INA226_RESET        0x8000U
#define INA226_CONFIG       0x4526U
#define INA226_MAN_ID       0x5449U
#define INA226_DIE_ID       0x2260U
#define INA226_DIE_MASK     0xFFF0U
#define DT35_I2C_TIMEOUT_MS 20U
#define DT35_VOLT_NEAR_MV   0U
#define DT35_VOLT_FAR_MV    10000U
#define DT35_DIST_NEAR_CM   5U
#define DT35_DIST_FAR_CM    20U

static I2C_HandleTypeDef *dt35_i2c;
static uint8_t dt35_ready_40;
static uint8_t dt35_ready_41;

static HAL_StatusTypeDef dt35_init_sensor(uint8_t address);
static HAL_StatusTypeDef dt35_read_sensor(uint8_t address, dt35_data_t *data);
static HAL_StatusTypeDef dt35_reg_write(uint8_t address, uint8_t reg,
                                        uint16_t value);
static HAL_StatusTypeDef dt35_reg_read(uint8_t address, uint8_t reg,
                                       uint16_t *value);
static uint16_t dt35_to_distance(uint16_t voltage_mv);

HAL_StatusTypeDef DT35_Init(I2C_HandleTypeDef *i2c)
{
    if (i2c == NULL)
    {
        return HAL_ERROR;
    }

    dt35_i2c = i2c;
    dt35_ready_41 = (dt35_init_sensor(DT35_ADDR_41) == HAL_OK) ? 1U : 0U;
    dt35_ready_40 = (dt35_init_sensor(DT35_ADDR_40) == HAL_OK) ? 1U : 0U;

    return ((dt35_ready_41 != 0U) || (dt35_ready_40 != 0U))
               ? HAL_OK
               : HAL_ERROR;
}

HAL_StatusTypeDef DT35_Read_41(dt35_data_t *data)
{
    if (dt35_ready_41 == 0U)
    {
        return HAL_ERROR;
    }
    return dt35_read_sensor(DT35_ADDR_41, data);
}

HAL_StatusTypeDef DT35_Read_40(dt35_data_t *data)
{
    if (dt35_ready_40 == 0U)
    {
        return HAL_ERROR;
    }
    return dt35_read_sensor(DT35_ADDR_40, data);
}

static HAL_StatusTypeDef dt35_init_sensor(uint8_t address)
{
    HAL_StatusTypeDef state;
    uint16_t id;

    state = HAL_I2C_IsDeviceReady(dt35_i2c, address << 1U,
                                  2U, DT35_I2C_TIMEOUT_MS);
    if (state != HAL_OK)
    {
        return state;
    }

    state = dt35_reg_write(address, INA226_REG_CONFIG, INA226_RESET);
    if (state != HAL_OK)
    {
        return state;
    }
    HAL_Delay(2U);

    state = dt35_reg_read(address, INA226_REG_MAN_ID, &id);
    if ((state != HAL_OK) || (id != INA226_MAN_ID))
    {
        return HAL_ERROR;
    }

    state = dt35_reg_read(address, INA226_REG_DIE_ID, &id);
    if ((state != HAL_OK) || ((id & INA226_DIE_MASK) != INA226_DIE_ID))
    {
        return HAL_ERROR;
    }

    state = dt35_reg_write(address, INA226_REG_CONFIG, INA226_CONFIG);
    if (state == HAL_OK)
    {
        HAL_Delay(20U);
    }

    return state;
}

static HAL_StatusTypeDef dt35_read_sensor(uint8_t address, dt35_data_t *data)
{
    HAL_StatusTypeDef state;
    uint8_t rx_data[2];
    uint16_t raw;

    if ((dt35_i2c == NULL) || (data == NULL))
    {
        return HAL_ERROR;
    }

    state = HAL_I2C_Mem_Read(dt35_i2c, address << 1U, INA226_REG_BUS,
                             I2C_MEMADD_SIZE_8BIT, rx_data, 2U,
                             DT35_I2C_TIMEOUT_MS);
    if (state != HAL_OK)
    {
        return state;
    }

    raw = ((uint16_t)rx_data[0] << 8U) | rx_data[1];
    data->raw = raw;
    data->voltage_mv = (uint16_t)(((uint32_t)raw * 5U + 2U) / 4U);
    data->distance_cm = dt35_to_distance(data->voltage_mv);
    return HAL_OK;
}

static uint16_t dt35_to_distance(uint16_t voltage_mv)
{
    if (voltage_mv <= DT35_VOLT_NEAR_MV)
    {
        return DT35_DIST_NEAR_CM;
    }
    if (voltage_mv >= DT35_VOLT_FAR_MV)
    {
        return DT35_DIST_FAR_CM;
    }

    return (uint16_t)(DT35_DIST_NEAR_CM +
        ((uint32_t)(voltage_mv - DT35_VOLT_NEAR_MV) *
         (DT35_DIST_FAR_CM - DT35_DIST_NEAR_CM)) /
        (DT35_VOLT_FAR_MV - DT35_VOLT_NEAR_MV));
}

static HAL_StatusTypeDef dt35_reg_write(uint8_t address, uint8_t reg,
                                        uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)value;
    return HAL_I2C_Mem_Write(dt35_i2c, address << 1U, reg,
                             I2C_MEMADD_SIZE_8BIT, data, 2U,
                             DT35_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef dt35_reg_read(uint8_t address, uint8_t reg,
                                       uint16_t *value)
{
    HAL_StatusTypeDef state;
    uint8_t data[2];

    state = HAL_I2C_Mem_Read(dt35_i2c, address << 1U, reg,
                             I2C_MEMADD_SIZE_8BIT, data, 2U,
                             DT35_I2C_TIMEOUT_MS);
    if (state == HAL_OK)
    {
        *value = ((uint16_t)data[0] << 8U) | data[1];
    }

    return state;
}
