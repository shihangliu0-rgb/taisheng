/**
 ******************************************************************************
 * @file    w25q128.h
 * @brief   Handle-based W25Q128 SPI NOR flash driver (DMA transfers).
 *
 * Chip facts:
 *   - Density : 128 Mbit = 16 MB
 *   - Page    : 256 B      (program unit)
 *   - Sector  : 4 KB       (smallest erase unit)
 *   - Block   : 32 KB / 64 KB
 *   - Full    : 16 MB
 ******************************************************************************
 */

#ifndef __W25Q128_H
#define __W25Q128_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------- Geometry ---------------------------------------------------- */
#define W25Q128_PAGE_SIZE         256U
#define W25Q128_SECTOR_SIZE       4096U
#define W25Q128_BLOCK32_SIZE      (32U  * 1024U)
#define W25Q128_BLOCK64_SIZE      (64U  * 1024U)
#define W25Q128_TOTAL_SIZE        (16U  * 1024U * 1024U)
#define W25Q128_PAGE_COUNT        (W25Q128_TOTAL_SIZE / W25Q128_PAGE_SIZE)
#define W25Q128_SECTOR_COUNT      (W25Q128_TOTAL_SIZE / W25Q128_SECTOR_SIZE)

/* ---------- JEDEC / Manufacturer IDs ------------------------------------ */
#define W25Q_MANUFACTURER_WINBOND 0xEFU   //厂商
#define W25Q128_DEV_ID            0x17U      /* Read Device ID  (ABh) */
#define W25Q128_JEDEC_ID          0xEF4018U  /* Read JEDEC ID   (9Fh) */

/* ---------- Command set ------------------------------------------------- */
//控制命令
#define W25Q_CMD_WRITE_ENABLE     0x06
#define W25Q_CMD_WRITE_DISABLE    0x04
#define W25Q_CMD_READ_SR1         0x05
#define W25Q_CMD_READ_SR2         0x35
#define W25Q_CMD_READ_SR3         0x15
#define W25Q_CMD_WRITE_SR1        0x01
#define W25Q_CMD_READ_DATA        0x03
#define W25Q_CMD_FAST_READ        0x0B
#define W25Q_CMD_PAGE_PROGRAM     0x02
#define W25Q_CMD_SECTOR_ERASE_4K  0x20
#define W25Q_CMD_BLOCK_ERASE_32K  0x52
#define W25Q_CMD_BLOCK_ERASE_64K  0xD8
#define W25Q_CMD_CHIP_ERASE       0xC7   /* or 0x60 */
#define W25Q_CMD_POWER_DOWN       0xB9
#define W25Q_CMD_RELEASE_PD       0xAB
#define W25Q_CMD_MFR_DEV_ID       0x90
#define W25Q_CMD_JEDEC_ID         0x9F
#define W25Q_CMD_ENABLE_RESET     0x66
#define W25Q_CMD_RESET_DEVICE     0x99

/* ---------- Status register bits ---------------------------------------- */
#define W25Q_SR1_BUSY             0x01
#define W25Q_SR1_WEL              0x02

/* ---------- Error codes ------------------------------------------------- */
typedef enum
{
    W25Q_OK              = 0,
    W25Q_ERR_PARAM       = -1,
    W25Q_ERR_SPI         = -2,   /* SPI HAL failure                      */
    W25Q_ERR_TIMEOUT     = -3,   /* chip stayed busy too long            */
    W25Q_ERR_ID          = -4,   /* JEDEC ID mismatch                    */
    W25Q_ERR_RANGE       = -5    /* addr/len outside chip                */
} W25Q_Status;

/* ---------- Handle ------------------------------------------------------ */
typedef struct
{
    SPI_HandleTypeDef *spi_handle;         /* SPI peripheral driving the chip   */
    GPIO_TypeDef      *cs_gpio_port;   /* CS# GPIO port                     */
    uint16_t           cs_gpio_pin;    /* CS# GPIO pin                      */

    /* Filled by W25Q_Init(): */
    uint8_t            manufacturer_id;//厂商
    uint8_t            device_id;       //设备id
    uint32_t           jedec_id;        //完整的id
    uint32_t           capacity;           /* bytes                             */
    uint32_t           sector_size;        /* bytes                             */
} W25Q_HandleTypeDef;

/* ---------- Public API -------------------------------------------------- */
W25Q_Status W25Q_Init          (W25Q_HandleTypeDef *handle,SPI_HandleTypeDef  *spi_handle,GPIO_TypeDef *cs_gpio_port, uint16_t cs_gpio_pin);

W25Q_Status W25Q_ReadJEDEC     (W25Q_HandleTypeDef *handle, uint32_t *jedec_id_out);
W25Q_Status W25Q_ReadMfrDevID  (W25Q_HandleTypeDef *handle, uint8_t *manufacturer_id_out, uint8_t *device_id_out);
W25Q_Status W25Q_ReadStatus1   (W25Q_HandleTypeDef *handle, uint8_t *status_out);
W25Q_Status W25Q_WaitBusy      (W25Q_HandleTypeDef *handle, uint32_t timeout_ms);

//读取flash
W25Q_Status W25Q_Read          (W25Q_HandleTypeDef *handle,
                                uint32_t address, uint8_t *buffer, uint32_t length);

//写flash
W25Q_Status W25Q_Write         (W25Q_HandleTypeDef *handle,uint32_t address, const uint8_t *buffer, uint32_t length);

//擦除函数，分为不同大小
W25Q_Status W25Q_EraseSector   (W25Q_HandleTypeDef *handle, uint32_t address);      /* 4KB  */
W25Q_Status W25Q_EraseBlock32K (W25Q_HandleTypeDef *handle, uint32_t address);      /* 32KB */
W25Q_Status W25Q_EraseBlock64K (W25Q_HandleTypeDef *handle, uint32_t address);      /* 64KB */
W25Q_Status W25Q_EraseChip     (W25Q_HandleTypeDef *handle);                       /* full */

/* Erase an arbitrary range: rounds outward to 4KB sector alignment. */
W25Q_Status W25Q_EraseRange    (W25Q_HandleTypeDef *handle, uint32_t address, uint32_t length);

/* Convenience: erase-then-write across an arbitrary range. */
W25Q_Status W25Q_EraseWrite    (W25Q_HandleTypeDef *handle, uint32_t address, const uint8_t *buffer, uint32_t length);

W25Q_Status W25Q_PowerDown     (W25Q_HandleTypeDef *handle);
W25Q_Status W25Q_ReleasePowerDown(W25Q_HandleTypeDef *handle);
W25Q_Status W25Q_Reset         (W25Q_HandleTypeDef *handle);

#ifdef __cplusplus
}
#endif

#endif /* __W25Q128_H */
