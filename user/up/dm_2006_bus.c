#include "dm_2006_bus.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* 环形队列掩码，队列长度必须保持为 2 的幂。 */
#define STD_CAN_RX_QUEUE_MASK (STD_CAN_RX_QUEUE_SIZE - 1U)
#define STD_CAN_ID_MAX 0x7FFU

#if ((STD_CAN_RX_QUEUE_SIZE & STD_CAN_RX_QUEUE_MASK) != 0U)
#error "STD_CAN_RX_QUEUE_SIZE must be a power of two"
#endif

static HAL_StatusTypeDef configure_filters(FDCAN_HandleTypeDef *device,
                                            const uint16_t *rx_ids,
                                            uint8_t rx_id_count)
{
    uint8_t filter_count = (uint8_t)((rx_id_count + 1U) / 2U);
    uint8_t i;

    if (filter_count > device->Init.StdFiltersNbr)
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < filter_count; i++)
    {
        uint8_t first = (uint8_t)(i * 2U);
        FDCAN_FilterTypeDef filter = {0};

        filter.IdType = FDCAN_STANDARD_ID;
        filter.FilterIndex = i;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        filter.FilterID1 = rx_ids[first];
        if ((uint8_t)(first + 1U) < rx_id_count)
        {
            filter.FilterType = FDCAN_FILTER_DUAL;
            filter.FilterID2 = rx_ids[first + 1U];
        }
        else
        {
            filter.FilterType = FDCAN_FILTER_MASK;
            filter.FilterID2 = STD_CAN_ID_MAX;
        }

        if (HAL_FDCAN_ConfigFilter(device, &filter) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

static bool rx_ids_valid(const uint16_t *rx_ids, uint8_t rx_id_count)
{
    uint8_t i;
    uint8_t j;

    if ((rx_ids == NULL) || (rx_id_count == 0U))
    {
        return false;
    }

    for (i = 0U; i < rx_id_count; i++)
    {
        if (rx_ids[i] > STD_CAN_ID_MAX)
        {
            return false;
        }
        for (j = (uint8_t)(i + 1U); j < rx_id_count; j++)
        {
            if (rx_ids[i] == rx_ids[j])
            {
                return false;
            }
        }
    }

    return true;
}

HAL_StatusTypeDef StdCan_Init(std_can_t *bus, FDCAN_HandleTypeDef *device,
                              const uint16_t *rx_ids, uint8_t rx_id_count)
{
    HAL_StatusTypeDef status;

    if ((bus == NULL) || (device == NULL) ||
        !rx_ids_valid(rx_ids, rx_id_count))
    {
        return HAL_ERROR;
    }

    memset(bus, 0, sizeof(*bus));
    bus->device = device;

    status = configure_filters(device, rx_ids, rx_id_count);
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_ConfigGlobalFilter(
            device, FDCAN_REJECT, FDCAN_REJECT,
            FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    }
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_Start(device);
    }
    if (status == HAL_OK)
    {
        status = HAL_FDCAN_ActivateNotification(
            device,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_ERROR_WARNING |
                FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_BUS_OFF,
            0U);
    }
    if (status != HAL_OK)
    {
        (void)HAL_FDCAN_Stop(device);
        return status;
    }

    bus->ready = true;
    return HAL_OK;
}

void StdCan_Stop(std_can_t *bus)
{
    if ((bus == NULL) || !bus->ready)
    {
        return;
    }

    (void)HAL_FDCAN_Stop(bus->device);
    bus->ready = false;
}

HAL_StatusTypeDef StdCan_AddHandler(std_can_t *bus,
                                    std_can_rx_handler_t callback,
                                    void *context)
{
    uint8_t i;

    if ((bus == NULL) || !bus->ready || (callback == NULL))
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < bus->handler_count; i++)
    {
        if ((bus->handlers[i].callback == callback) &&
            (bus->handlers[i].context == context))
        {
            return HAL_OK;
        }
    }
    if (bus->handler_count >= STD_CAN_MAX_HANDLERS)
    {
        return HAL_ERROR;
    }

    bus->handlers[bus->handler_count].callback = callback;
    bus->handlers[bus->handler_count].context = context;
    bus->handler_count++;
    return HAL_OK;
}

HAL_StatusTypeDef StdCan_Send(std_can_t *bus, uint16_t id,
                              const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};

    if ((bus == NULL) || !bus->ready || bus->bus_off || (data == NULL) ||
        (id > STD_CAN_ID_MAX))
    {
        return HAL_ERROR;
    }
    if (HAL_FDCAN_GetTxFifoFreeLevel(bus->device) == 0U)
    {
        return HAL_BUSY;
    }

    header.Identifier = id;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    return HAL_FDCAN_AddMessageToTxFifoQ(bus->device, &header, data);
}

void StdCan_HandleRxIsr(std_can_t *bus, uint32_t interrupt_flags)
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

        if (HAL_FDCAN_GetRxMessage(bus->device, FDCAN_RX_FIFO0, &header,
                                   data) != HAL_OK)
        {
            return;
        }
        if ((header.IdType != FDCAN_STANDARD_ID) ||
            (header.RxFrameType != FDCAN_DATA_FRAME) ||
            (header.DataLength != FDCAN_DLC_BYTES_8))
        {
            continue;
        }

        head = bus->rx_head;
        next = (uint8_t)((head + 1U) & STD_CAN_RX_QUEUE_MASK);
        if (next == bus->rx_tail)
        {
            continue;
        }

        bus->rx_queue[head].id = (uint16_t)header.Identifier;
        bus->rx_queue[head].tick_ms = HAL_GetTick();
        memcpy(bus->rx_queue[head].data, data, sizeof(data));
        __DMB();
        bus->rx_head = next;
    }
}

void StdCan_HandleErrorIsr(std_can_t *bus, uint32_t interrupt_flags)
{
    if ((bus == NULL) || !bus->ready)
    {
        return;
    }
    if ((interrupt_flags & FDCAN_IT_BUS_OFF) != 0U)
    {
        bus->bus_off = true;
    }
}

bool StdCan_BusOff(const std_can_t *bus)
{
    return (bus != NULL) && bus->bus_off;
}

void StdCan_ProcessRx(std_can_t *bus)
{
    if ((bus == NULL) || !bus->ready)
    {
        return;
    }

    while (bus->rx_tail != bus->rx_head)
    {
        std_can_frame_t frame;
        uint8_t tail = bus->rx_tail;
        uint8_t i;

        frame = bus->rx_queue[tail];
        __DMB();
        bus->rx_tail = (uint8_t)((tail + 1U) & STD_CAN_RX_QUEUE_MASK);

        for (i = 0U; i < bus->handler_count; i++)
        {
            bus->handlers[i].callback(bus->handlers[i].context, frame.id,
                                      frame.data, frame.tick_ms);
        }
    }
}
