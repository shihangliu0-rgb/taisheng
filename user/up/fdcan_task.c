#include "fdcan_task.h"

#include "fdcan.h"

#include <stdbool.h>
#include <stddef.h>

static rs_bus_t rs_bus; // FDCAN2：RS 扩展帧
static std_can_t std_can; // FDCAN3：DM 与 M2006 标准帧
static bool task_ready;

HAL_StatusTypeDef Fdcan_Init(const fdcan_config_t *config)
{
    if (task_ready)
    {
        return HAL_OK;
    }
    if ((config == NULL) || (config->std_rx_ids == NULL) ||
        (config->std_rx_id_count == 0U))
    {
        return HAL_ERROR;
    }

    if (RsBus_Init(&rs_bus, &hfdcan2, config->rs_host_id) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (StdCan_Init(&std_can, &hfdcan3, config->std_rx_ids,
                    config->std_rx_id_count) != HAL_OK)
    {
        RsBus_Stop(&rs_bus);
        return HAL_ERROR;
    }

    task_ready = true;
    return HAL_OK;
}

void Fdcan_Run1ms(void)
{
    if (!task_ready)
    {
        return;
    }

    RsBus_ProcessRx(&rs_bus);
    StdCan_ProcessRx(&std_can);
}

void Fdcan_Stop(void)
{
    if (!task_ready)
    {
        return;
    }

    RsBus_Stop(&rs_bus);
    StdCan_Stop(&std_can);
    task_ready = false;
}

rs_bus_t *Fdcan_GetRsBus(void)
{
    return task_ready ? &rs_bus : NULL;
}

std_can_t *Fdcan_GetStdBus(void)
{
    return task_ready ? &std_can : NULL;
}

void Fdcan_HandleRxIsr(FDCAN_HandleTypeDef *hfdcan,
                       uint32_t interrupt_flags)
{
    // 中断只按总线取帧入队，协议解析留在 1 ms 调度中执行。
    if (hfdcan == &hfdcan2)
    {
        RsBus_HandleRxIsr(&rs_bus, interrupt_flags);
    }
    else if (hfdcan == &hfdcan3)
    {
        StdCan_HandleRxIsr(&std_can, interrupt_flags);
    }
}

void Fdcan_HandleErrorIsr(FDCAN_HandleTypeDef *hfdcan,
                          uint32_t interrupt_flags)
{
    if (hfdcan == &hfdcan3)
    {
        StdCan_HandleErrorIsr(&std_can, interrupt_flags);
    }
}
