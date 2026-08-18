#ifndef FDCAN_TASK_H
#define FDCAN_TASK_H

#include "dm_2006_bus.h"
#include "rs_bus.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t rs_host_id;
    const uint16_t *std_rx_ids;
    uint8_t std_rx_id_count;
} fdcan_config_t;

/** @brief 初始化抬升机构使用的两条 FDCAN 总线。 */
HAL_StatusTypeDef Fdcan_Init(const fdcan_config_t *config);
/** @brief 在 1 ms 任务中处理接收队列并尝试恢复 Bus-Off。 */
void Fdcan_Run1ms(void);
/** @brief 停止两条 FDCAN 总线。 */
void Fdcan_Stop(void);
/** @brief 获取 RS00 总线对象。 */
rs_bus_t *Fdcan_GetRsBus(void);
/** @brief 获取标准 CAN 总线对象。 */
std_can_t *Fdcan_GetStdBus(void);
/** @brief 分发接收中断，只做取帧入队。 */
void Fdcan_HandleRxIsr(FDCAN_HandleTypeDef *hfdcan,
                       uint32_t interrupt_flags);
/** @brief 分发错误中断，只记录总线状态。 */
void Fdcan_HandleErrorIsr(FDCAN_HandleTypeDef *hfdcan,
                          uint32_t interrupt_flags);
/** @brief 查询任一抬升总线当前是否处于 Bus-Off 状态。 */
bool Fdcan_BusOff(void);
bool Fdcan_RsBusOff(void);
bool Fdcan_StdBusOff(void);

#endif
