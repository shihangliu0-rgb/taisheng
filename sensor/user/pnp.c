#include "pnp.h"

/* INA226 寄存器映射和 PNP 采样参数。 */
#define INA226_REG_CONFIG       0x00U
#define INA226_REG_BUS          0x02U
#define INA226_REG_MAN_ID       0xFEU
#define INA226_REG_DIE_ID       0xFFU
#define INA226_RESET            0x8000U
#define INA226_CONFIG           0x4526U
#define INA226_MAN_ID           0x5449U
#define INA226_DIE_ID           0x2260U
#define INA226_DIE_MASK         0xFFF0U
#define PNP_I2C_TIMEOUT_MS      20U
#define INA226_BUS_LSB_UV       1250U
#define PNP_TRIGGER_THRESHOLD_MV 23000U

static I2C_HandleTypeDef *pnp_i2c;
static uint8_t pnp_ready_f;
static uint8_t pnp_ready_b;

uint16_t pnp_raw_f;
uint16_t pnp_raw_b;
uint16_t pnp_voltage_f_mv;
uint16_t pnp_voltage_b_mv;
uint8_t pnp_trigger_f;
uint8_t pnp_trigger_b;
HAL_StatusTypeDef pnp_state_f = HAL_ERROR;
HAL_StatusTypeDef pnp_state_b = HAL_ERROR;

static HAL_StatusTypeDef pnp_init_sensor(uint8_t address);
static HAL_StatusTypeDef pnp_read_voltage(uint8_t address, uint16_t *raw,
                                          uint16_t *voltage_mv);
static HAL_StatusTypeDef pnp_reg_write(uint8_t address, uint8_t reg,
                                       uint16_t value);
static HAL_StatusTypeDef pnp_reg_read(uint8_t address, uint8_t reg,
                                      uint16_t *value);

HAL_StatusTypeDef PNP_Init(I2C_HandleTypeDef *i2c)
{
    if (i2c == NULL)
    {
        return HAL_ERROR;
    }

    pnp_i2c = i2c;
    pnp_ready_f = (pnp_init_sensor(PNP_ADDR_F) == HAL_OK) ? 1U : 0U;
    pnp_ready_b = (pnp_init_sensor(PNP_ADDR_B) == HAL_OK) ? 1U : 0U;
    pnp_state_f = (pnp_ready_f != 0U) ? HAL_OK : HAL_ERROR;
    pnp_state_b = (pnp_ready_b != 0U) ? HAL_OK : HAL_ERROR;
    pnp_trigger_f = 0U;
    pnp_trigger_b = 0U;

    return ((pnp_ready_f != 0U) || (pnp_ready_b != 0U))
               ? HAL_OK
               : HAL_ERROR;
}

void PNP_Update(void)
{
    if (pnp_ready_f != 0U)
    {
        pnp_state_f = pnp_read_voltage(PNP_ADDR_F, &pnp_raw_f,
                                       &pnp_voltage_f_mv);
        if (pnp_state_f == HAL_OK)
        {
            pnp_trigger_f = (pnp_voltage_f_mv > PNP_TRIGGER_THRESHOLD_MV)
                                ? 1U
                                : 0U;
        }
        else
        {
            pnp_trigger_f = 0U;
        }
    }
    else
    {
        pnp_state_f = HAL_ERROR;
        pnp_trigger_f = 0U;
    }

    if (pnp_ready_b != 0U)
    {
        pnp_state_b = pnp_read_voltage(PNP_ADDR_B, &pnp_raw_b,
                                       &pnp_voltage_b_mv);
        if (pnp_state_b == HAL_OK)
        {
            pnp_trigger_b = (pnp_voltage_b_mv > PNP_TRIGGER_THRESHOLD_MV)
                                ? 1U
                                : 0U;
        }
        else
        {
            pnp_trigger_b = 0U;
        }
    }
    else
    {
        pnp_state_b = HAL_ERROR;
        pnp_trigger_b = 0U;
    }
}

static HAL_StatusTypeDef pnp_init_sensor(uint8_t address)
{
    HAL_StatusTypeDef state;
    uint16_t id;

    state = HAL_I2C_IsDeviceReady(pnp_i2c, address << 1U,
                                  2U, PNP_I2C_TIMEOUT_MS);
    if (state != HAL_OK)
    {
        return state;
    }

    state = pnp_reg_write(address, INA226_REG_CONFIG, INA226_RESET);
    if (state != HAL_OK)
    {
        return state;
    }
    HAL_Delay(2U);

    state = pnp_reg_read(address, INA226_REG_MAN_ID, &id);
    if ((state != HAL_OK) || (id != INA226_MAN_ID))
    {
        return HAL_ERROR;
    }

    state = pnp_reg_read(address, INA226_REG_DIE_ID, &id);
    if ((state != HAL_OK) || ((id & INA226_DIE_MASK) != INA226_DIE_ID))
    {
        return HAL_ERROR;
    }

    state = pnp_reg_write(address, INA226_REG_CONFIG, INA226_CONFIG);
    if (state == HAL_OK)
    {
        HAL_Delay(20U);
    }

    return state;
}

static HAL_StatusTypeDef pnp_read_voltage(uint8_t address, uint16_t *raw,
                                          uint16_t *voltage_mv)
{
    HAL_StatusTypeDef state;
    uint8_t data[2];

    if ((pnp_i2c == NULL) || (raw == NULL) || (voltage_mv == NULL))
    {
        return HAL_ERROR;
    }

    state = HAL_I2C_Mem_Read(pnp_i2c, address << 1U, INA226_REG_BUS,
                             I2C_MEMADD_SIZE_8BIT, data, 2U,
                             PNP_I2C_TIMEOUT_MS);
    if (state != HAL_OK)
    {
        return state;
    }

    *raw = ((uint16_t)data[0] << 8U) | data[1];
    *voltage_mv = (uint16_t)(((uint32_t)(*raw) * INA226_BUS_LSB_UV + 500U) /
                             1000U);
    return HAL_OK;
}

static HAL_StatusTypeDef pnp_reg_write(uint8_t address, uint8_t reg,
                                       uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)(value >> 8U);
    data[1] = (uint8_t)value;
    return HAL_I2C_Mem_Write(pnp_i2c, address << 1U, reg,
                             I2C_MEMADD_SIZE_8BIT, data, 2U,
                             PNP_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef pnp_reg_read(uint8_t address, uint8_t reg,
                                      uint16_t *value)
{
    HAL_StatusTypeDef state;
    uint8_t data[2];

    if (value == NULL)
    {
        return HAL_ERROR;
    }

    state = HAL_I2C_Mem_Read(pnp_i2c, address << 1U, reg,
                             I2C_MEMADD_SIZE_8BIT, data, 2U,
                             PNP_I2C_TIMEOUT_MS);
    if (state == HAL_OK)
    {
        *value = ((uint16_t)data[0] << 8U) | data[1];
    }

    return state;
}
