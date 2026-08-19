#include "fdcan_task.h"

#include "fdcan.h"

#define FDCAN_RECOVERY_PERIOD_MS 100U

static rs_bus_t rs_bus;
static bool task_ready;
static uint32_t next_recovery_ms;

static bool time_reached(uint32_t now_ms, uint32_t due_ms)
{
    return (int32_t)(now_ms - due_ms) >= 0;
}

HAL_StatusTypeDef Fdcan_Init(const fdcan_config_t *config)
{
    if (task_ready)
    {
        return HAL_OK;
    }
    if ((config == NULL) ||
        (RsBus_Init(&rs_bus, &hfdcan2, config->rs_host_id) != HAL_OK))
    {
        return HAL_ERROR;
    }
    next_recovery_ms = 0U;
    task_ready = true;
    return HAL_OK;
}

void Fdcan_Run1ms(void)
{
    uint32_t now_ms;

    if (!task_ready)
    {
        return;
    }
    now_ms = HAL_GetTick();
    if (RsBus_BusOff(&rs_bus) && time_reached(now_ms, next_recovery_ms))
    {
        next_recovery_ms = now_ms + FDCAN_RECOVERY_PERIOD_MS;
        (void)RsBus_Recover(&rs_bus);
    }
    RsBus_ProcessRx(&rs_bus);
}

void Fdcan_Stop(void)
{
    if (task_ready)
    {
        RsBus_Stop(&rs_bus);
        task_ready = false;
    }
}

rs_bus_t *Fdcan_GetRsBus(void)
{
    return task_ready ? &rs_bus : NULL;
}

void Fdcan_HandleRxIsr(FDCAN_HandleTypeDef *hfdcan,
                       uint32_t interrupt_flags)
{
    if ((hfdcan == &hfdcan2) && task_ready)
    {
        RsBus_HandleRxIsr(&rs_bus, interrupt_flags);
    }
}

void Fdcan_HandleErrorIsr(FDCAN_HandleTypeDef *hfdcan,
                          uint32_t interrupt_flags)
{
    if ((hfdcan == &hfdcan2) && task_ready)
    {
        RsBus_HandleErrorIsr(&rs_bus, interrupt_flags);
    }
}

bool Fdcan_BusOff(void)
{
    return task_ready && RsBus_BusOff(&rs_bus);
}
