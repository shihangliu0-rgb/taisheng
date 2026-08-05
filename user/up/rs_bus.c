#include "rs_bus.h"

#include <stddef.h>
#include <string.h>

#define RS_BUS_RX_QUEUE_MASK (RS_BUS_RX_QUEUE_SIZE - 1U)

#if ((RS_BUS_RX_QUEUE_SIZE & RS_BUS_RX_QUEUE_MASK) != 0U)
#error "RS_BUS_RX_QUEUE_SIZE must be a power of two"
#endif

HAL_StatusTypeDef RsBus_Init(rs_bus_t *bus, FDCAN_HandleTypeDef *device,
                             uint8_t host_id)
{
    FDCAN_FilterTypeDef filter = { 0 };
    HAL_StatusTypeDef status;

    if ((bus == NULL) || (device == NULL))
    {
        return HAL_ERROR;
    }

    memset(bus, 0, sizeof(*bus));
    bus->device = device;
    bus->host_id = host_id;

    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = host_id;
    filter.FilterID2 = 0xFFU;

    status = HAL_FDCAN_ConfigFilter(device, &filter);
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_ConfigGlobalFilter(device, FDCAN_REJECT, FDCAN_REJECT,
                                              FDCAN_REJECT_REMOTE,
                                              FDCAN_REJECT_REMOTE);
    }
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_Start(device);
    }
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_ActivateNotification(
            device, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    }
    if (status != HAL_OK)
    {
        (void)HAL_FDCAN_Stop(device);
        return status;
    }

    bus->ready = true;
    return HAL_OK;
}

void RsBus_Stop(rs_bus_t *bus)
{
    if ((bus == NULL) || !bus->ready)
    {
        return;
    }

    (void)HAL_FDCAN_Stop(bus->device);
    bus->ready = false;
}

HAL_StatusTypeDef RsBus_SetHandler(rs_bus_t *bus,
                                   rs_bus_rx_handler_t handler,
                                   void *context)
{
    if ((bus == NULL) || !bus->ready || (handler == NULL))
    {
        return HAL_ERROR;
    }

    bus->rx_handler = handler;
    bus->rx_context = context;
    return HAL_OK;
}

HAL_StatusTypeDef RsBus_Send(rs_bus_t *bus, uint32_t id,
                             const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = { 0 };

    if ((bus == NULL) || !bus->ready || (data == NULL))
    {
        return HAL_ERROR;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(bus->device) == 0U)
    {
        return HAL_BUSY;
    }

    header.Identifier = id & 0x1FFFFFFFU;
    header.IdType = FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(bus->device, &header, data);
}

void RsBus_HandleRxIsr(rs_bus_t *bus, uint32_t interrupt_flags)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if ((bus == NULL) || !bus->ready ||
        ((interrupt_flags & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U))
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(bus->device, FDCAN_RX_FIFO0) > 0U)
    {
        uint8_t head;
        uint8_t next;

        if (HAL_FDCAN_GetRxMessage(bus->device, FDCAN_RX_FIFO0, &header, data) !=
            HAL_OK)
        {
            return;
        }
        if ((header.IdType != FDCAN_EXTENDED_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME) ||
            (header.DataLength != FDCAN_DLC_BYTES_8))
        {
            continue;
        }

        head = bus->rx_head;
        next = (uint8_t)((head + 1U) & RS_BUS_RX_QUEUE_MASK);
        if (next == bus->rx_tail)
        {
            continue;
        }

        bus->rx_queue[head].id = header.Identifier;
        bus->rx_queue[head].tick_ms = HAL_GetTick();
        memcpy(bus->rx_queue[head].data, data, sizeof(data));
        __DMB();
        bus->rx_head = next;
    }
}

void RsBus_ProcessRx(rs_bus_t *bus)
{
    if ((bus == NULL) || !bus->ready || (bus->rx_handler == NULL))
    {
        return;
    }

    while (bus->rx_tail != bus->rx_head)
    {
        rs_bus_frame_t frame;
        uint8_t tail = bus->rx_tail;

        frame = bus->rx_queue[tail];
        __DMB();
        bus->rx_tail = (uint8_t)((tail + 1U) & RS_BUS_RX_QUEUE_MASK);
        bus->rx_handler(bus->rx_context, frame.id, frame.data, frame.tick_ms);
    }
}
