#include "RemoteInput.h"
#include "adc.h"
#include "gpio.h"

#define REMOTE_AXIS_MAX 255U        /* 摇杆输出最大值 */
#define REMOTE_AXIS_CENTER 128U     /* 摇杆输出中心值 */
#define REMOTE_SHOULDER_RAW_MIN 675U
#define REMOTE_SHOULDER_RAW_IDLE_MIN 2030U
#define REMOTE_SHOULDER_RAW_IDLE_MAX 2070U
#define REMOTE_SHOULDER_RAW_MAX 3722U
#define REMOTE_ADC_MARGIN 10U       /* 模拟量抗抖余量 */
#define REMOTE_DEBOUNCE_SAMPLES 10U /* 数字输入稳定 10 ms 后生效 */
#define REMOTE_BEEP_MS 30U          /* 按键提示音时长(ms) */
#define REMOTE_BEEP_GAP_MS 100U     /* 上电提示音间隔(ms) */

typedef struct
{
    uint16_t min;
    uint16_t mid;
    uint16_t max;
} remote_axis_cal_t;

volatile uint16_t remote_shoulder_adc[REMOTE_SHOULDER_NUM];
volatile uint16_t remote_axis_adc[REMOTE_AXIS_NUM];
remote_adc_state_t remote_adc_state;

static uint16_t remote_shoulder[REMOTE_SHOULDER_NUM];
static uint8_t remote_shoulder_ready;
static const remote_axis_cal_t remote_axis_cal[REMOTE_AXIS_NUM] =
{
    {10U, 2103U, 4080U}, /* 左 Y，PC2 / ADC2_IN12 */
    {10U, 1960U, 4080U}, /* 左 X，PC3 / ADC2_IN13 */
    {10U, 2142U, 4080U}, /* 右 X，PB0 / ADC2_IN8 */
    {10U, 1964U, 4080U}  /* 右 Y，PB1 / ADC2_IN9 */
};

static GPIO_TypeDef *const remote_sw_ports[REMOTE_SWITCH_COUNT] =
{
    SW_1_GPIO_Port, SW_2_GPIO_Port, SW_3_GPIO_Port,
    SW_4_GPIO_Port, SW_5_GPIO_Port, SW_6_GPIO_Port
};
static const uint16_t remote_sw_pins[REMOTE_SWITCH_COUNT] =
{
    SW_1_Pin, SW_2_Pin, SW_3_Pin, SW_4_Pin, SW_5_Pin, SW_6_Pin
};
static GPIO_TypeDef *const remote_key_ports[REMOTE_KEY_COUNT] =
{
    KEY_1_GPIO_Port, KEY_2_GPIO_Port, KEY_3_GPIO_Port,
    KEY_4_GPIO_Port, KEY_5_GPIO_Port, KEY_6_GPIO_Port,
    KEY_7_GPIO_Port, KEY_8_GPIO_Port, KEY_9_GPIO_Port,
    KEY_10_GPIO_Port, KEY_11_GPIO_Port, KEY_12_GPIO_Port
};
static const uint16_t remote_key_pins[REMOTE_KEY_COUNT] =
{
    KEY_1_Pin, KEY_2_Pin, KEY_3_Pin, KEY_4_Pin, KEY_5_Pin, KEY_6_Pin,
    KEY_7_Pin, KEY_8_Pin, KEY_9_Pin, KEY_10_Pin, KEY_11_Pin, KEY_12_Pin
};
static uint8_t remote_last_sw[REMOTE_SWITCH_COUNT];
static uint8_t remote_last_key[REMOTE_KEY_COUNT];
static uint8_t remote_sw_candidate[REMOTE_SWITCH_COUNT];
static uint8_t remote_key_candidate[REMOTE_KEY_COUNT];
static uint8_t remote_sw_count[REMOTE_SWITCH_COUNT];
static uint8_t remote_key_count[REMOTE_KEY_COUNT];
static uint32_t remote_beep_stop_ms;
static uint8_t remote_beep_active;

/**
 * @brief 将输入数值线性映射到指定范围
 * @param val 输入值
 * @param in_min 输入下限
 * @param in_max 输入上限
 * @param out_min 输出下限
 * @param out_max 输出上限
 * @retval 映射后的数值
 */
static uint32_t Remote_MapValue(uint32_t val, uint32_t in_min, uint32_t in_max,
                                uint32_t out_min, uint32_t out_max)
{
    if (val <= in_min)
    {
        return out_min;
    }
    if (val >= in_max)
    {
        return out_max;
    }
    return out_min + ((val - in_min) * (out_max - out_min)) / (in_max - in_min);
}

/**
 * @brief 对摇杆输出施加中心和边缘死区
 * @param val 摇杆映射值
 * @retval 处理后的摇杆值
 */
static uint8_t Remote_ApplyDeadzone(uint32_t val)
{
    if (val <= REMOTE_ADC_MARGIN)
    {
        return 0U;
    }
    if (val >= REMOTE_AXIS_MAX - REMOTE_ADC_MARGIN)
    {
        return REMOTE_AXIS_MAX;
    }
    if ((val >= REMOTE_AXIS_CENTER - REMOTE_ADC_MARGIN) &&
        (val <= REMOTE_AXIS_CENTER + REMOTE_ADC_MARGIN))
    {
        return REMOTE_AXIS_CENTER;
    }
    if (val < REMOTE_AXIS_CENTER - REMOTE_ADC_MARGIN)
    {
        return (uint8_t)Remote_MapValue(val, REMOTE_ADC_MARGIN + 1U,
                                        REMOTE_AXIS_CENTER - REMOTE_ADC_MARGIN - 1U,
                                        0U, REMOTE_AXIS_CENTER - 1U);
    }
    return (uint8_t)Remote_MapValue(val, REMOTE_AXIS_CENTER + REMOTE_ADC_MARGIN + 1U,
                                    REMOTE_AXIS_MAX - REMOTE_ADC_MARGIN - 1U,
                                    REMOTE_AXIS_CENTER + 1U, REMOTE_AXIS_MAX);
}

/**
 * @brief 使用统一余量保持肩键 ADC 稳定值
 * @param val 当前 ADC 值
 * @param prev 上一次稳定值
 * @retval 滤波后的 ADC 值
 */
static uint16_t Remote_FilterAdc(uint16_t val, uint16_t prev)
{
    uint16_t diff;

    if (val >= prev)
    {
        diff = val - prev;
    }
    else
    {
        diff = prev - val;
    }
    if (diff <= REMOTE_ADC_MARGIN)
    {
        return prev;
    }
    return val;
}

/**
 * @brief 将肩键两个方向的 ADC 行程转换为无方向的按动幅度
 * @param raw 12 位 ADC 原始值
 * @retval 肩键幅度，松开为 0，任一方向按到底为 1000
 */
static uint16_t Remote_ConvertShoulder(uint16_t raw)
{
    if (raw < REMOTE_SHOULDER_RAW_IDLE_MIN)
    {
        return (uint16_t)(REMOTE_SHOULDER_MAX -
                          Remote_MapValue(raw,
                                          REMOTE_SHOULDER_RAW_MIN,
                                          REMOTE_SHOULDER_RAW_IDLE_MIN,
                                          0U, REMOTE_SHOULDER_MAX));
    }
    if (raw > REMOTE_SHOULDER_RAW_IDLE_MAX)
    {
        return (uint16_t)Remote_MapValue(raw,
                                         REMOTE_SHOULDER_RAW_IDLE_MAX,
                                         REMOTE_SHOULDER_RAW_MAX,
                                         0U, REMOTE_SHOULDER_MAX);
    }
    return 0U;
}

/**
 * @brief 根据单轴标定点将 ADC 原始值转换为 0 到 255
 * @param axis 摇杆轴序号
 * @param raw ADC 原始值
 * @retval 转换后的摇杆值
 */
static uint8_t Remote_ConvertAxis(uint8_t axis, uint16_t raw)
{
    const remote_axis_cal_t *cal = &remote_axis_cal[axis];
    uint32_t val;

    if (raw <= cal->min)
    {
        val = 0U;
    }
    else if (raw >= cal->max)
    {
        val = REMOTE_AXIS_MAX;
    }
    else if (raw <= cal->mid)
    {
        val = Remote_MapValue(raw, cal->min, cal->mid, 0U, REMOTE_AXIS_CENTER);
    }
    else
    {
        val = Remote_MapValue(raw, cal->mid, cal->max,
                              REMOTE_AXIS_CENTER, REMOTE_AXIS_MAX);
    }
    return Remote_ApplyDeadzone(val);
}

/**
 * @brief 读取一个低电平有效的按键或开关
 * @param port GPIO 端口
 * @param pin GPIO 引脚
 * @retval 1 表示按下或闭合，0 表示松开或断开
 */
static uint8_t Remote_ReadLow(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static uint8_t Remote_Debounce(uint8_t raw, uint8_t *stable,
                               uint8_t *candidate, uint8_t *count)
{
    if (raw == *stable)
    {
        *candidate = raw;
        *count = 0U;
        return 0U;
    }
    if (raw != *candidate)
    {
        *candidate = raw;
        *count = 1U;
        return 0U;
    }
    if (*count < REMOTE_DEBOUNCE_SAMPLES)
    {
        (*count)++;
    }
    if (*count < REMOTE_DEBOUNCE_SAMPLES)
    {
        return 0U;
    }

    *stable = raw;
    *count = 0U;
    return 1U;
}

static void Remote_StartBeep(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    remote_beep_stop_ms = HAL_GetTick() + REMOTE_BEEP_MS;
    remote_beep_active = 1U;
}

static void Remote_UpdateBeep(void)
{
    if ((remote_beep_active != 0U) &&
        ((int32_t)(HAL_GetTick() - remote_beep_stop_ms) >= 0))
    {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        remote_beep_active = 0U;
    }
}

/**
 * @brief 控制蜂鸣器输出一个短音
 * @param ms 提示音时长(ms)
 * @retval None
 */
static void Remote_Beep(uint32_t ms)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_Delay(ms);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 启动遥控器 ADC DMA 并保存数字输入初始状态
 * @param None
 * @retval None
 */
void Remote_Init(void)
{
    uint8_t i;

    for (i = 0U; i < REMOTE_SWITCH_COUNT; i++)
    {
        remote_last_sw[i] = Remote_ReadLow(remote_sw_ports[i], remote_sw_pins[i]);
        remote_sw_candidate[i] = remote_last_sw[i];
        remote_sw_count[i] = 0U;
        remote_data.switch_state[i] = remote_last_sw[i];
    }
    for (i = 0U; i < REMOTE_KEY_COUNT; i++)
    {
        remote_last_key[i] = Remote_ReadLow(remote_key_ports[i], remote_key_pins[i]);
        remote_key_candidate[i] = remote_last_key[i];
        remote_key_count[i] = 0U;
        remote_data.key_state[i] = remote_last_key[i];
    }
    remote_beep_active = 0U;

    remote_adc_state = REMOTE_ADC_OK;
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)remote_shoulder_adc,
                          REMOTE_SHOULDER_NUM) != HAL_OK)
    {
        remote_adc_state = REMOTE_ADC1_ERROR;
        return;
    }
    /* 任务直接读取循环 DMA 缓冲区，不使用高频完成中断。 */
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT | DMA_IT_TC);

    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)remote_axis_adc,
                          REMOTE_AXIS_NUM) != HAL_OK)
    {
        remote_adc_state = REMOTE_ADC2_ERROR;
        return;
    }
    __HAL_DMA_DISABLE_IT(hadc2.DMA_Handle, DMA_IT_HT | DMA_IT_TC);

    Remote_Beep(REMOTE_BEEP_MS);
    HAL_Delay(REMOTE_BEEP_GAP_MS);
    Remote_Beep(REMOTE_BEEP_MS);
    HAL_Delay(REMOTE_BEEP_GAP_MS);
    Remote_Beep(REMOTE_BEEP_MS);
    HAL_Delay(REMOTE_BEEP_GAP_MS);
}

/**
 * @brief 更新摇杆、肩键、按键和开关数据
 * @param None
 * @retval None
 */
void Remote_Update(void)
{
    uint8_t i;
    uint8_t raw;
    uint8_t changed = 0U;

    Remote_UpdateBeep();

    if (remote_adc_state == REMOTE_ADC_OK)
    {
        if (remote_shoulder_ready == 0U)
        {
            for (i = 0U; i < REMOTE_SHOULDER_NUM; i++)
            {
                remote_shoulder[i] = remote_shoulder_adc[i];
            }
            remote_shoulder_ready = 1U;
        }
        else
        {
            for (i = 0U; i < REMOTE_SHOULDER_NUM; i++)
            {
                remote_shoulder[i] = Remote_FilterAdc(remote_shoulder_adc[i],
                                                       remote_shoulder[i]);
            }
        }

        remote_data.left_shoulder = Remote_ConvertShoulder(remote_shoulder[0]);
        remote_data.right_shoulder = Remote_ConvertShoulder(remote_shoulder[1]);
        remote_data.left_x = Remote_ConvertAxis(1U, remote_axis_adc[1]);
        remote_data.left_y = Remote_ConvertAxis(0U, remote_axis_adc[0]);
        remote_data.right_x = Remote_ConvertAxis(2U, remote_axis_adc[2]);
        remote_data.right_y = Remote_ConvertAxis(3U, remote_axis_adc[3]);
    }

    for (i = 0U; i < REMOTE_SWITCH_COUNT; i++)
    {
        raw = Remote_ReadLow(remote_sw_ports[i], remote_sw_pins[i]);
        if (Remote_Debounce(raw, &remote_last_sw[i],
                            &remote_sw_candidate[i], &remote_sw_count[i]) != 0U)
        {
            changed = 1U;
        }
        remote_data.switch_state[i] = remote_last_sw[i];
    }
    for (i = 0U; i < REMOTE_KEY_COUNT; i++)
    {
        raw = Remote_ReadLow(remote_key_ports[i], remote_key_pins[i]);
        if (Remote_Debounce(raw, &remote_last_key[i],
                            &remote_key_candidate[i], &remote_key_count[i]) != 0U)
        {
            changed = 1U;
        }
        remote_data.key_state[i] = remote_last_key[i];
    }
    if (changed != 0U)
    {
        Remote_StartBeep();
    }
}
