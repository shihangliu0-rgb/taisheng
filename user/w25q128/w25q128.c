/**
 ******************************************************************************
 * @file    w25q128.c
 * @brief   Handle-based W25Q128 SPI NOR flash driver (DMA transfers).
 ******************************************************************************
 */

#include "w25q128.h"
/* 移植自 C++ 工程：<cstdint> 是 C++ 头，C 工程改用 stdint.h(w25q128.h 已包含) */
#include <stdint.h>
#include <string.h>

//超时保护
#define W25Q_SPI_TIMEOUT_MS        HAL_MAX_DELAY
#define W25Q_BUSY_TIMEOUT_PROG_MS  50U        /* page program ~ 0.7 ms typ  */
#define W25Q_BUSY_TIMEOUT_SEC_MS   1000U      /* 4KB  sector erase ~ 45 ms  */
#define W25Q_BUSY_TIMEOUT_BLK_MS   3000U      /* 64KB block  erase ~ 150ms  */
#define W25Q_BUSY_TIMEOUT_CHIP_MS  200000U    /* chip erase up to ~100 s    */

/**
 ******************************************************************************
 * @file    cs_w_low/cs_w_high
 * @brief   拉低或拉高cs
 ******************************************************************************
 */
static inline void cs_w_low(W25Q_HandleTypeDef *handle)
{
    HAL_GPIO_WritePin(handle->cs_gpio_port, handle->cs_gpio_pin, GPIO_PIN_RESET);
}
static inline void cs_w_high(W25Q_HandleTypeDef *handle)
{
    HAL_GPIO_WritePin(handle->cs_gpio_port, handle->cs_gpio_pin, GPIO_PIN_SET);
}


static W25Q_Status spi_wait_ready(W25Q_HandleTypeDef *handle, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    while (HAL_SPI_GetState(handle->spi_handle) != HAL_SPI_STATE_READY)
    {
        if ((HAL_GetTick() - start_tick) > timeout_ms) return W25Q_ERR_TIMEOUT;
    }
    return W25Q_OK;
}

/**
 ******************************************************************************
 * @file    spi_transmit/spi_receive/spi_transmit_receive
 * @brief   发送数据/接受数据
 ******************************************************************************
 */
static W25Q_Status spi_transmit(W25Q_HandleTypeDef *handle, const uint8_t *transmit_data, uint16_t length)
{
    if (HAL_SPI_Transmit_DMA(handle->spi_handle, (uint8_t *)transmit_data, length) != HAL_OK)
        return W25Q_ERR_SPI;
    return spi_wait_ready(handle, W25Q_SPI_TIMEOUT_MS);
}

/* Blocking Receive via DMA. */
static W25Q_Status spi_receive(W25Q_HandleTypeDef *handle, uint8_t *receive_data, uint16_t length)
{
    if (HAL_SPI_Receive_DMA(handle->spi_handle, receive_data, length) != HAL_OK)
        return W25Q_ERR_SPI;
    return spi_wait_ready(handle, W25Q_SPI_TIMEOUT_MS);
}

/* Full-duplex TX/RX via DMA (used for ID reads). */
static W25Q_Status spi_transmit_receive(W25Q_HandleTypeDef *handle,
                                        const uint8_t *transmit_data, uint8_t *receive_data, uint16_t length)
{
    if (HAL_SPI_TransmitReceive_DMA(handle->spi_handle, (uint8_t *)transmit_data, receive_data, length) != HAL_OK)
        return W25Q_ERR_SPI;
    return spi_wait_ready(handle, W25Q_SPI_TIMEOUT_MS);
}

/**
 ******************************************************************************
 * @file    write_enable
 * @brief   写使能
 ******************************************************************************
 */
static W25Q_Status write_enable(W25Q_HandleTypeDef *handle)
{
    uint8_t command = W25Q_CMD_WRITE_ENABLE;
    cs_w_low(handle);
    W25Q_Status status = spi_transmit(handle, &command, 1);
    cs_w_high(handle);
    return status;
}
/**
 ******************************************************************************
 * @file    W25Q_ReadStatus1
 * @brief   读取寄存器1的状态
 ******************************************************************************
 */
W25Q_Status W25Q_ReadStatus1(W25Q_HandleTypeDef *handle, uint8_t *status_out)
{
    if(handle == NULL || status_out == NULL) return W25Q_ERR_PARAM;//空指针返回
    uint8_t command = W25Q_CMD_READ_SR1;
    W25Q_Status status;

    cs_w_low(handle);
    status = spi_transmit(handle, &command, 1);
    if (status == W25Q_OK) status = spi_receive(handle, status_out, 1);//读取出来status，——out
    cs_w_high(handle);
    return status;
}

W25Q_Status W25Q_WaitBusy(W25Q_HandleTypeDef *handle, uint32_t timeout_ms)
{
    if (handle == NULL) return W25Q_ERR_PARAM;
    uint8_t status_register = 0;
    uint32_t start_tick = HAL_GetTick();
    do
    {
        W25Q_Status status = W25Q_ReadStatus1(handle, &status_register);//status_register=0是空闲
        if (status != W25Q_OK) return status;
        if ((status_register & W25Q_SR1_BUSY) == 0) return W25Q_OK;
        if ((HAL_GetTick() - start_tick) > timeout_ms) return W25Q_ERR_TIMEOUT;
    } while (1);
}

/**
 ******************************************************************************
 * @file    W25Q_ReadJEDEC
 * @brief   读取JEDEC ID，把他存储在handle->jedec_id中，返回值是W25Q_OK
 ******************************************************************************
 */
W25Q_Status W25Q_ReadJEDEC(W25Q_HandleTypeDef *handle, uint32_t *jedec_id_out)
{
    if (handle == NULL || jedec_id_out == NULL) return W25Q_ERR_PARAM;
    uint8_t command = W25Q_CMD_JEDEC_ID;
    uint8_t receive_data[3] = {0};
    W25Q_Status status;

    cs_w_low(handle);
    status = spi_transmit(handle, &command, 1);
    if (status == W25Q_OK) status = spi_receive(handle, receive_data, 3);
    cs_w_high(handle);
    if (status != W25Q_OK) return status;

    *jedec_id_out = ((uint32_t)receive_data[0] << 16) | ((uint32_t)receive_data[1] << 8) | receive_data[2];
    return W25Q_OK;
}
/**
 ******************************************************************************
 * @file    W25Q_ReadMfrDevID
 * @brief   读取制造商和设备 ID，把他存储在handle->manufacturer_id和handle->device_id中，返回值是W25Q_OK
 ******************************************************************************
 */
W25Q_Status W25Q_ReadMfrDevID(W25Q_HandleTypeDef *handle, uint8_t *manufacturer_id_out, uint8_t *device_id_out)
{
    if (!handle || !manufacturer_id_out || !device_id_out) return W25Q_ERR_PARAM;
    /* Cmd 0x90 + 3 address bytes (all zero) then reads MFR, DEV. */
    uint8_t transmit_data[4] = { W25Q_CMD_MFR_DEV_ID, 0, 0, 0 };
    uint8_t receive_data[2] = {0};
    W25Q_Status status;

    cs_w_low(handle);
    status = spi_transmit(handle, transmit_data, 4);
    if (status == W25Q_OK) status = spi_receive(handle, receive_data, 2);
    cs_w_high(handle);
    if (status != W25Q_OK) return status;

    *manufacturer_id_out = receive_data[0];
    *device_id_out = receive_data[1];
    return W25Q_OK;
}
/**
 ******************************************************************************
 * @file    W25Q_Init
 * @brief   初始化W25Q128 flash
 ******************************************************************************
 */
W25Q_Status W25Q_Init(W25Q_HandleTypeDef *handle,SPI_HandleTypeDef *spi_handle,GPIO_TypeDef*cs_gpio_port, uint16_t cs_gpio_pin)
{
    if (!handle || !spi_handle || !cs_gpio_port) return W25Q_ERR_PARAM;

    handle->spi_handle       = spi_handle;
    handle->cs_gpio_port = cs_gpio_port;
    handle->cs_gpio_pin  = cs_gpio_pin;
    handle->capacity         = W25Q128_TOTAL_SIZE;
    handle->sector_size      = W25Q128_SECTOR_SIZE;

   
    cs_w_high(handle);

    
    (void)W25Q_ReleasePowerDown(handle);
    HAL_Delay(1);

    W25Q_Status status = W25Q_ReadJEDEC(handle, &handle->jedec_id);
    if (status != W25Q_OK) return status;

    handle->manufacturer_id = (uint8_t)((handle->jedec_id >> 16) & 0xFF);
    handle->device_id = (uint8_t)(handle->jedec_id & 0xFF);   /* capacity code */

    if (handle->manufacturer_id != W25Q_MANUFACTURER_WINBOND || handle->jedec_id != W25Q128_JEDEC_ID)
        return W25Q_ERR_ID;

    return W25Q_OK;
}

/**
 ******************************************************************************
 * @file    W25Q_Read
 * @brief   读取W25Q128 flash，address开始地址，buffer存储数据的缓冲区，length读取的长度，flash自动递增地址，连续输出数据
 ******************************************************************************
 */
W25Q_Status W25Q_Read(W25Q_HandleTypeDef *handle,uint32_t address, uint8_t *buffer, uint32_t length)
{
    if (!handle || !buffer)return W25Q_ERR_PARAM;
    if (length == 0)return W25Q_OK;
    if ((uint64_t)address + length > handle->capacity)  return W25Q_ERR_RANGE; //超出范围


    W25Q_Status status = W25Q_WaitBusy(handle, W25Q_BUSY_TIMEOUT_SEC_MS);
    if (status != W25Q_OK) return status;

    while (length)
    {
        uint32_t chunk_size = (length > 0xFF00U) ? 0xFF00U : length;//如果读取的范围过大，规定最大范围

        uint8_t header[5];
        header[0] = W25Q_CMD_FAST_READ;
        header[1] = (uint8_t)(address >> 16);
        header[2] = (uint8_t)(address >> 8);
        header[3] = (uint8_t)(address      );
        header[4] = 0x00;    /* dummy */

        cs_w_low(handle);
        status = spi_transmit(handle, header, sizeof(header));
        if (status == W25Q_OK) status = spi_receive(handle, buffer, (uint16_t)chunk_size);//读取到数组的大小
        cs_w_high(handle);

        if (status != W25Q_OK) return status;

        address += chunk_size;
        buffer  += chunk_size;
        length  -= chunk_size;
    }
    return W25Q_OK;
}

/**
 ******************************************************************************
 * @file    page_program
 * @brief   页管理，控制写入的页
 ******************************************************************************
 */
static W25Q_Status page_program(W25Q_HandleTypeDef *handle, uint32_t address, const uint8_t *buffer, uint16_t length)
{
    /* Caller guarantees: address..address+length-1 fits inside a single 256-byte page. */
    W25Q_Status status = W25Q_WaitBusy(handle, W25Q_BUSY_TIMEOUT_SEC_MS);
    if (status != W25Q_OK) return status;

    status = write_enable(handle);
    if (status != W25Q_OK) return status;

    uint8_t header[4];
    header[0] = W25Q_CMD_PAGE_PROGRAM;
    header[1] = (uint8_t)(address >> 16);
    header[2] = (uint8_t)(address >> 8);
    header[3] = (uint8_t)(address      );

    cs_w_low(handle);
    status = spi_transmit(handle, header, sizeof(header));
    if (status == W25Q_OK) status = spi_transmit(handle, (uint8_t *)buffer, length);
    cs_w_high(handle);
    if (status != W25Q_OK) return status;

    return W25Q_WaitBusy(handle, W25Q_BUSY_TIMEOUT_PROG_MS);
}

/**
 ******************************************************************************
 * @file    W25Q_Write
 * @brief   向W25Q128 flash写入数据，address开始地址，buffer存储数据的缓冲区，length写入的长度，flash自动递增地址，连续写入数据
 ******************************************************************************
 */
W25Q_Status W25Q_Write(W25Q_HandleTypeDef *handle,uint32_t address, const uint8_t *buffer, uint32_t length)
{
    if (!handle || !buffer)return W25Q_ERR_PARAM;
    if (length == 0)return W25Q_OK;
    if ((uint64_t)address + length > handle->capacity)  return W25Q_ERR_RANGE;

    while (length)
    {
        /* Bytes left in the current 256-byte page. */
        uint32_t page_offset      = address & (W25Q128_PAGE_SIZE - 1U);//当前页使用量  0xff   0x00F0 & 0xFF
        uint32_t page_remaining   = W25Q128_PAGE_SIZE - page_offset;//当前页剩余
        uint32_t chunk_size       = (length < page_remaining) ? length : page_remaining;//当前写入大小，

        W25Q_Status status = page_program(handle, address, buffer, (uint16_t)chunk_size);
        if (status != W25Q_OK) return status;

        address += chunk_size;
        buffer  += chunk_size;
        length  -= chunk_size;
    }
    return W25Q_OK;
}

/**
 ******************************************************************************
 * @file    erase_command
 * @brief   执行擦除命令 ，command不同命令擦除不同的大小操作	
 *           SPI命令
 *          4KB扇区擦除	0x20
 *          32KB块擦除	0x52
 *          64KB块擦除	0xD8
 *          整片擦除	0xC7
 ******************************************************************************
 */
static W25Q_Status erase_command(W25Q_HandleTypeDef *handle,uint8_t command, uint32_t address, uint32_t timeout_ms)
{
    W25Q_Status status = W25Q_WaitBusy(handle, W25Q_BUSY_TIMEOUT_SEC_MS);
    if (status != W25Q_OK) return status;

    status = write_enable(handle);
    if (status != W25Q_OK) return status;

    uint8_t command_frame[4];
    command_frame[0] = command;
    command_frame[1] = (uint8_t)(address >> 16);
    command_frame[2] = (uint8_t)(address >> 8);
    command_frame[3] = (uint8_t)(address      );

    cs_w_low(handle);
    status = spi_transmit(handle, command_frame, sizeof(command_frame));
    cs_w_high(handle);
    if (status != W25Q_OK) return status;

    return W25Q_WaitBusy(handle, timeout_ms);
}

W25Q_Status W25Q_EraseSector(W25Q_HandleTypeDef *handle, uint32_t address)
{
    if (!handle)                          return W25Q_ERR_PARAM;
    if (address >= handle->capacity)      return W25Q_ERR_RANGE;
    address &= ~(W25Q128_SECTOR_SIZE - 1U);
    return erase_command(handle, W25Q_CMD_SECTOR_ERASE_4K, address, W25Q_BUSY_TIMEOUT_SEC_MS);
}

W25Q_Status W25Q_EraseBlock32K(W25Q_HandleTypeDef *handle, uint32_t address)
{
    if (!handle)                          return W25Q_ERR_PARAM;
    if (address >= handle->capacity)      return W25Q_ERR_RANGE;
    address &= ~(W25Q128_BLOCK32_SIZE - 1U); //0x0fff ,擦除整个4kb区域，获取该地址实在那个区块
    return erase_command(handle, W25Q_CMD_BLOCK_ERASE_32K, address, W25Q_BUSY_TIMEOUT_BLK_MS);
}

W25Q_Status W25Q_EraseBlock64K(W25Q_HandleTypeDef *handle, uint32_t address)
{
    if (!handle)                          return W25Q_ERR_PARAM;
    if (address >= handle->capacity)      return W25Q_ERR_RANGE;
    address &= ~(W25Q128_BLOCK64_SIZE - 1U);
    return erase_command(handle, W25Q_CMD_BLOCK_ERASE_64K, address, W25Q_BUSY_TIMEOUT_BLK_MS);
}

W25Q_Status W25Q_EraseChip(W25Q_HandleTypeDef *handle)
{
    if (!handle) return W25Q_ERR_PARAM;

    W25Q_Status status = W25Q_WaitBusy(handle, W25Q_BUSY_TIMEOUT_SEC_MS);
    if (status != W25Q_OK) return status;

    status = write_enable(handle);
    if (status != W25Q_OK) return status;

    uint8_t command = W25Q_CMD_CHIP_ERASE;
    cs_w_low(handle);
    status = spi_transmit(handle, &command, 1);
    cs_w_high(handle);
    if (status != W25Q_OK) return status;

    return W25Q_WaitBusy(handle, W25Q_BUSY_TIMEOUT_CHIP_MS);
}

/**
 ******************************************************************************
 * @file    W25Q_Write
 * @brief   向W25Q128 flash写入数据，address开始地址，buffer存储数据的缓冲区，length写入的长度，flash自动递增地址，连续写入数据
 * @func    用户给一个任意地址和长度，我自动计算需要擦哪些扇区，并选择最优擦除方式。
 ******************************************************************************
 */
W25Q_Status W25Q_EraseRange(W25Q_HandleTypeDef *handle,uint32_t address, uint32_t length)
{
    if (!handle)                                  return W25Q_ERR_PARAM;
    if (length == 0)                              return W25Q_OK;
    if ((uint64_t)address + length > handle->capacity)  return W25Q_ERR_RANGE;

   
    uint32_t start_address = address & ~(W25Q128_SECTOR_SIZE - 1U);
    uint32_t end_address   = (address + length + W25Q128_SECTOR_SIZE - 1U) & ~(W25Q128_SECTOR_SIZE - 1U);//获取结束地址，向上取整到4kb的整数倍

    for (uint32_t current_address = start_address; current_address < end_address; current_address += W25Q128_SECTOR_SIZE)
    {
        /* Opportunistically use 64KB erase when whole block fits. */
        if (((current_address & (W25Q128_BLOCK64_SIZE - 1U)) == 0) &&(end_address - current_address >= W25Q128_BLOCK64_SIZE))//如何剩余空间很大，直接使用64kb擦除方式
        {
            W25Q_Status status = W25Q_EraseBlock64K(handle, current_address);
            if (status != W25Q_OK) return status;
            current_address += (W25Q128_BLOCK64_SIZE - W25Q128_SECTOR_SIZE); /* loop adds one sector */
        }
        else
        {
            W25Q_Status status = W25Q_EraseSector(handle, current_address);
            if (status != W25Q_OK) return status;
        }
    }
    return W25Q_OK;
}
/**
 ******************************************************************************
 * @file    W25Q_EraseWrite
 * @brief   先擦除后写入数据
 ******************************************************************************
 */
W25Q_Status W25Q_EraseWrite(W25Q_HandleTypeDef *handle, uint32_t address, const uint8_t *buffer, uint32_t length)
{
    W25Q_Status status = W25Q_EraseRange(handle, address, length);
    if (status != W25Q_OK) return status;
    return W25Q_Write(handle, address, buffer, length);
}

/**
 ******************************************************************************
 * @file    W25Q_PowerDown
 * @brief   使能W25Q128 flash进入掉电模式
 ******************************************************************************
 */
W25Q_Status W25Q_PowerDown(W25Q_HandleTypeDef *handle)
{
    if (!handle) return W25Q_ERR_PARAM;
    uint8_t command = W25Q_CMD_POWER_DOWN;
    cs_w_low(handle);
    W25Q_Status status = spi_transmit(handle, &command, 1);
    cs_w_high(handle);
    return status;
}
/**
 ******************************************************************************
 * @file    W25Q_ReleasePowerDown
 * @brief   使能W25Q128 flash退出掉电模式
 ******************************************************************************
 */
W25Q_Status W25Q_ReleasePowerDown(W25Q_HandleTypeDef *handle)
{
    if (!handle) return W25Q_ERR_PARAM;
    uint8_t command = W25Q_CMD_RELEASE_PD;
    cs_w_low(handle);
    W25Q_Status status = spi_transmit(handle, &command, 1);
    cs_w_high(handle);
    /* tRES1 = 3us max — a HAL_Delay(1) after this is enough. */
    return status;
}

/**
 ******************************************************************************
 * @file    W25Q_Reset
 * @brief   重置W25Q128 flash
 ******************************************************************************
 */
W25Q_Status W25Q_Reset(W25Q_HandleTypeDef *handle)
{
    if (!handle) return W25Q_ERR_PARAM;
    uint8_t reset_enable_command = W25Q_CMD_ENABLE_RESET;
    uint8_t reset_device_command = W25Q_CMD_RESET_DEVICE;

    cs_w_low(handle);
    W25Q_Status status = spi_transmit(handle, &reset_enable_command, 1);
    cs_w_high(handle);
    if (status != W25Q_OK) return status;

    cs_w_low(handle);
    status = spi_transmit(handle, &reset_device_command, 1);
    cs_w_high(handle);
    HAL_Delay(1);         /* tRST = 30 us max */
    return status;
}
