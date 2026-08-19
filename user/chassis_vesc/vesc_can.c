#include "vesc_can.h"

#include <stddef.h>

static uint32_t get_dlc(uint8_t length)
{
    static const uint32_t dlc[] = {
        FDCAN_DLC_BYTES_0,
        FDCAN_DLC_BYTES_1,
        FDCAN_DLC_BYTES_2,
        FDCAN_DLC_BYTES_3,
        FDCAN_DLC_BYTES_4,
        FDCAN_DLC_BYTES_5,
        FDCAN_DLC_BYTES_6,
        FDCAN_DLC_BYTES_7,
        FDCAN_DLC_BYTES_8
    };

    return dlc[length];
}

static uint8_t get_length(uint32_t dlc)
{
    if (dlc > FDCAN_DLC_BYTES_8)
    {
        return 0U;
    }

    return (uint8_t)(dlc >> 16);
}

HAL_StatusTypeDef VescCan_Init(vesc_can_t *bus,
                               FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef filter = {0};

    if ((bus == NULL) || (hfdcan == NULL))
    {
        return HAL_ERROR;
    }
    if (bus->ready)
    {
        return HAL_OK;
    }

    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;

    if (HAL_FDCAN_ConfigRxFifoOverwrite(hfdcan, FDCAN_RX_FIFO0,
                                        FDCAN_RX_FIFO_OVERWRITE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        return HAL_ERROR;
    }

    bus->hfdcan = hfdcan;
    bus->ready = true;
    return HAL_OK;
}

HAL_StatusTypeDef VescCan_Send(vesc_can_t *bus, uint8_t motor_id,
                               uint8_t packet_id, const uint8_t *data,
                               uint8_t length)
{
    FDCAN_TxHeaderTypeDef header = {0};

    if ((bus == NULL) || !bus->ready || (bus->hfdcan == NULL) ||
        (length > 8U) || ((length > 0U) && (data == NULL)))
    {
        return HAL_ERROR;
    }

    VescCan_Recover(bus);
    if (HAL_FDCAN_GetTxFifoFreeLevel(bus->hfdcan) == 0U)
    {
        return HAL_BUSY;
    }

    header.Identifier = ((uint32_t)packet_id << 8) | motor_id;
    header.IdType = FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = get_dlc(length);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(bus->hfdcan, &header,
                                         (uint8_t *)data);
}

HAL_StatusTypeDef VescCan_Read(vesc_can_t *bus, vesc_can_msg_t *msg)
{
    FDCAN_RxHeaderTypeDef header = {0};

    if ((bus == NULL) || !bus->ready || (bus->hfdcan == NULL) ||
        (msg == NULL))
    {
        return HAL_ERROR;
    }
    if (HAL_FDCAN_GetRxFifoFillLevel(bus->hfdcan,
                                     FDCAN_RX_FIFO0) == 0U)
    {
        return HAL_BUSY;
    }
    if (HAL_FDCAN_GetRxMessage(bus->hfdcan, FDCAN_RX_FIFO0,
                               &header, msg->data) != HAL_OK)
    {
        return HAL_ERROR;
    }
    if ((header.IdType != FDCAN_EXTENDED_ID) ||
        (header.RxFrameType != FDCAN_DATA_FRAME))
    {
        return HAL_ERROR;
    }

    msg->motor_id = (uint8_t)(header.Identifier & 0xFFU);
    msg->packet_id = (uint8_t)((header.Identifier >> 8) & 0xFFU);
    msg->length = get_length(header.DataLength);
    return HAL_OK;
}

void VescCan_Recover(vesc_can_t *bus)
{
    FDCAN_ProtocolStatusTypeDef protocol = {0};
    bool bus_off = false;

    if ((bus == NULL) || !bus->ready || (bus->hfdcan == NULL) ||
        (bus->hfdcan->Instance == NULL))
    {
        return;
    }

    if (HAL_FDCAN_GetProtocolStatus(bus->hfdcan, &protocol) == HAL_OK)
    {
        bus_off = (protocol.BusOff != 0U);
    }
    if (__HAL_FDCAN_GET_FLAG(bus->hfdcan, FDCAN_FLAG_BUS_OFF) != 0U)
    {
        __HAL_FDCAN_CLEAR_FLAG(bus->hfdcan, FDCAN_FLAG_BUS_OFF);
        bus_off = true;
    }
    if (bus_off ||
        ((bus->hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) != 0U))
    {
        CLEAR_BIT(bus->hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }

    if (__HAL_FDCAN_GET_FLAG(bus->hfdcan,
                             FDCAN_FLAG_RX_FIFO0_FULL) != 0U)
    {
        __HAL_FDCAN_CLEAR_FLAG(bus->hfdcan,
                               FDCAN_FLAG_RX_FIFO0_FULL);
    }
    if (__HAL_FDCAN_GET_FLAG(bus->hfdcan,
                             FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
        __HAL_FDCAN_CLEAR_FLAG(bus->hfdcan,
                               FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
    }
}

void VescCan_Stop(vesc_can_t *bus)
{
    if ((bus == NULL) || !bus->ready || (bus->hfdcan == NULL))
    {
        return;
    }

    (void)HAL_FDCAN_Stop(bus->hfdcan);
    bus->ready = false;
}
