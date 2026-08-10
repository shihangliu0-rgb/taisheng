/**
 * @file flash_params.c
 * @brief 机器人参数管理实现: RAM 配置 + W25Q128 双备份 + 范围限制
 */

#include "flash_params.h"
#include <string.h>

/* ============================ CRC32 (软件, zlib 多项式) ============================ */
static uint32_t crc32_calc(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t i, j;
    for (i = 0U; i < len; i++)
    {
        crc ^= p[i];
        for (j = 0U; j < 8U; j++)
        {
            crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320U) : (crc >> 1);
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/* ============================ 工具 ============================ */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/* ============================ 默认值(编译时常量) ============================ */
static const Robot_Config_t s_defaults = {
    .magic              = CONFIG_MAGIC,
    .version            = CONFIG_VERSION,
    .size               = sizeof(Robot_Config_t),
    .yaw_track          = { .kp = 1.8f,  .ki = 0.25f, .kd = 1.8f, .i_max = 8.0f,  .out_max = 1000.0f },
    .yaw_hold           = { .kp = 1.0f,  .ki = 0.18f, .kd = 1.8f, .i_max = 12.0f, .out_max = 250.0f  },
    .yaw_gyro_k         = 5.0f,
    .yaw_deadzone_deg   = 0.5f,
    .yaw_cmd_threshold  = 5.0f,
    .wheel_radius       = 0.076f,
    .half_width_scale   = 4.31f,
    .half_length_scale  = 5.30f,
    .max_rpm            = 4000.0f,
    .fusion_w_max       = 0.5f,
    .fusion_w_ramp      = 1.0f,
    .zupt_gyro_threshold    = 0.25f,
    .zupt_acc_std_threshold = 0.08f,
    .kalman_q           = 0.001f,
    .kalman_r           = 3.0f,
    .crc32              = 0U,
};

/* ============================ RAM 配置(全局, 所有模块通过 Config_Get 读) ============================ */
static Robot_Config_t g_config;

/* ============================ Flash 底层(条件编译) ============================ */
#ifdef HAL_SPI_MODULE_ENABLED

#include "w25q128.h"

static W25Q_HandleTypeDef s_flash;
static bool s_flash_attached = false;

/* 外部调用: 把 SPI 句柄和 CS 绑定到 flash 驱动 */
void FlashParams_AttachFlash(void *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (W25Q_Init(&s_flash, (SPI_HandleTypeDef *)spi, cs_port, cs_pin) == W25Q_OK)
    {
        s_flash_attached = true;
    }
}

static bool flash_read(uint32_t addr, Robot_Config_t *out)
{
    if (!s_flash_attached) { return false; }
    if (W25Q_Read(&s_flash, addr, (uint8_t *)out, sizeof(Robot_Config_t)) != W25Q_OK)
    {
        return false;
    }
    if (out->magic != CONFIG_MAGIC) { return false; }
    if (out->size > sizeof(Robot_Config_t)) { return false; } /* 未来版本, 本版无法用 */
    if (out->version == 0U) { return false; }
    uint32_t crc = crc32_calc(out, sizeof(Robot_Config_t) - 4U);
    return (crc == out->crc32);
}

static bool flash_write_slot(uint32_t addr)
{
    if (!s_flash_attached) { return false; }
    /* 填充头部 */
    g_config.magic   = CONFIG_MAGIC;
    g_config.size    = sizeof(Robot_Config_t);
    g_config.version = (uint16_t)((g_config.version + 1U) & 0xFFFFU); /* 递增(掉电保护用) */
    if (g_config.version == 0U) { g_config.version = 1U; }
    g_config.crc32   = crc32_calc(&g_config, sizeof(Robot_Config_t) - 4U);

    /* 擦扇区 + 写入 */
    if (W25Q_EraseSector(&s_flash, addr) != W25Q_OK) { return false; }
    if (W25Q_Write(&s_flash, addr, (const uint8_t *)&g_config, sizeof(Robot_Config_t)) != W25Q_OK)
    {
        return false;
    }
    return true;
}

#else  /* !HAL_SPI_MODULE_ENABLED — SPI 未开: stub */

void FlashParams_AttachFlash(void *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    (void)spi; (void)cs_port; (void)cs_pin;
}

static bool flash_read(uint32_t addr, Robot_Config_t *out)
{
    (void)addr; (void)out;
    return false; /* SPI 未开, 无法读 */
}

static bool flash_write_slot(uint32_t addr)
{
    (void)addr;
    return false; /* SPI 未开, 无法写 */
}

#endif /* HAL_SPI_MODULE_ENABLED */

/* ============================ 公开 API ============================ */

void FlashParams_ResetDefaults(void)
{
    memcpy(&g_config, &s_defaults, sizeof(Robot_Config_t));
}

void FlashParams_Init(void)
{
    FlashParams_ResetDefaults();   /* 先填默认值 */

    /* 尝试从 Flash 双 slot 加载 */
    Robot_Config_t buf_a, buf_b;
    bool ok_a = flash_read(FLASH_SLOT_A_ADDR, &buf_a);
    bool ok_b = flash_read(FLASH_SLOT_B_ADDR, &buf_b);

    if (ok_a && ok_b)
    {
        /* 两个都有效: 取 version 大的 */
        g_config = (buf_a.version >= buf_b.version) ? buf_a : buf_b;
    }
    else if (ok_a)
    {
        g_config = buf_a;
    }
    else if (ok_b)
    {
        g_config = buf_b;
    }
    /* 都无效: 保持默认值(上面 ResetDefaults 已填) */
}

bool FlashParams_Load(void)
{
    Robot_Config_t buf_a, buf_b;
    bool ok_a = flash_read(FLASH_SLOT_A_ADDR, &buf_a);
    bool ok_b = flash_read(FLASH_SLOT_B_ADDR, &buf_b);

    if (ok_a && ok_b)
    {
        g_config = (buf_a.version >= buf_b.version) ? buf_a : buf_b;
        return true;
    }
    if (ok_a) { g_config = buf_a; return true; }
    if (ok_b) { g_config = buf_b; return true; }
    return false;
}

bool FlashParams_Save(void)
{
    /* 双备份交替写: 读两个 slot 的 version, 写到较旧的那个 */
    static uint8_t s_last_slot = 0U;   /* 0=A, 1=B, 交替 */
    uint32_t addr = (s_last_slot == 0U) ? FLASH_SLOT_A_ADDR : FLASH_SLOT_B_ADDR;
    s_last_slot ^= 1U;
    return flash_write_slot(addr);
}

const Robot_Config_t *Config_Get(void)
{
    return &g_config;
}

/* ============================ 范围限制 + 参数设置 ============================ */

void Config_SetParam(Param_Id_t id, float value)
{
    switch (id)
    {
    case PARAM_WHEEL_RADIUS:         g_config.wheel_radius       = clampf(value, 0.01f, 0.5f);    break;
    case PARAM_HALF_WIDTH_SCALE:     g_config.half_width_scale   = clampf(value, 0.0f, 50.0f);    break;
    case PARAM_HALF_LENGTH_SCALE:    g_config.half_length_scale  = clampf(value, 0.0f, 50.0f);    break;
    case PARAM_MAX_RPM:              g_config.max_rpm            = clampf(value, 0.0f, 20000.0f); break;
    case PARAM_FUSION_W_MAX:         g_config.fusion_w_max       = clampf(value, 0.0f, 1.0f);     break;
    case PARAM_FUSION_W_RAMP:        g_config.fusion_w_ramp      = clampf(value, 0.0f, 10.0f);    break;
    case PARAM_ZUPT_GYRO:            g_config.zupt_gyro_threshold = clampf(value, 0.0f, 10.0f);  break;
    case PARAM_ZUPT_ACC_STD:         g_config.zupt_acc_std_threshold = clampf(value, 0.0f, 10.0f); break;
    case PARAM_KALMAN_Q:             g_config.kalman_q           = clampf(value, 0.0f, 1.0f);     break;
    case PARAM_KALMAN_R:             g_config.kalman_r           = clampf(value, 0.0f, 100.0f);   break;
    case PARAM_YAW_GYRO_K:           g_config.yaw_gyro_k         = clampf(value, 0.0f, 50.0f);    break;
    case PARAM_YAW_DEADZONE:         g_config.yaw_deadzone_deg   = clampf(value, 0.0f, 10.0f);    break;
    case PARAM_YAW_CMD_THRESHOLD:    g_config.yaw_cmd_threshold  = clampf(value, 0.0f, 100.0f);   break;
    default: break;
    }
}

float Config_GetParam(Param_Id_t id)
{
    switch (id)
    {
    case PARAM_WHEEL_RADIUS:         return g_config.wheel_radius;
    case PARAM_HALF_WIDTH_SCALE:     return g_config.half_width_scale;
    case PARAM_HALF_LENGTH_SCALE:    return g_config.half_length_scale;
    case PARAM_MAX_RPM:              return g_config.max_rpm;
    case PARAM_FUSION_W_MAX:         return g_config.fusion_w_max;
    case PARAM_FUSION_W_RAMP:        return g_config.fusion_w_ramp;
    case PARAM_ZUPT_GYRO:            return g_config.zupt_gyro_threshold;
    case PARAM_ZUPT_ACC_STD:         return g_config.zupt_acc_std_threshold;
    case PARAM_KALMAN_Q:             return g_config.kalman_q;
    case PARAM_KALMAN_R:             return g_config.kalman_r;
    case PARAM_YAW_GYRO_K:           return g_config.yaw_gyro_k;
    case PARAM_YAW_DEADZONE:         return g_config.yaw_deadzone_deg;
    case PARAM_YAW_CMD_THRESHOLD:    return g_config.yaw_cmd_threshold;
    default: return 0.0f;
    }
}

/* ---- PID 通道(5 参数一组) ---- */

static void clamp_pid(PID_Config_t *pid)
{
    pid->kp      = clampf(pid->kp,      0.0f, 100.0f);
    pid->ki      = clampf(pid->ki,      0.0f, 50.0f);
    pid->kd      = clampf(pid->kd,      0.0f, 100.0f);
    pid->i_max   = clampf(pid->i_max,   0.0f, 1000.0f);
    pid->out_max = clampf(pid->out_max, 0.0f, 5000.0f);
}

bool Config_SetPID(uint8_t channel, const float params[5])
{
    PID_Config_t *target = NULL;
    switch (channel)
    {
    case PID_CH_YAW_TRACK: target = &g_config.yaw_track; break;
    case PID_CH_YAW_HOLD:  target = &g_config.yaw_hold;  break;
    default: return false;
    }
    target->kp = params[0];
    target->ki = params[1];
    target->kd = params[2];
    target->i_max = params[3];
    target->out_max = params[4];
    clamp_pid(target);
    return true;
}

bool Config_GetPID(uint8_t channel, float params[5])
{
    const PID_Config_t *src = NULL;
    switch (channel)
    {
    case PID_CH_YAW_TRACK: src = &g_config.yaw_track; break;
    case PID_CH_YAW_HOLD:  src = &g_config.yaw_hold;  break;
    default: return false;
    }
    params[0] = src->kp;
    params[1] = src->ki;
    params[2] = src->kd;
    params[3] = src->i_max;
    params[4] = src->out_max;
    return true;
}
