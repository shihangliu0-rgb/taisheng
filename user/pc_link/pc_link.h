/**
 ******************************************************************************
 * @file    pc_link.h
 * @brief   小电脑(ROS2 competition_gateway)串口对接 —— 公共接口
 *
 * 协议要点(与上位机 competition_gateway/serial_protocol.hpp 完全一致):
 *
 *  上位机 -> STM32(同一串口交替发送):
 *   感知帧 44B: AA 55 | 10 | seq | flags | red_xyz | blue_xyz | ball_xyz
 *              (9 个 float,IEEE754 小端) | checksum | 0D 0A
 *   位置帧 24B: AA 55 | 11 | seq | flags | field_xyz | field_w | checksum | 0D 0A
 *     (field_w = yaw 角,单位 rad,用户约定 yaw=0 朝 +y;非四元数 w 分量)
 *   STM32 -> 上位机(>=10Hz):
 *   状态帧  8B: 55 AA | 20 | state | error | checksum | 0D 0A
 *
 *  校验和 = 帧类型字节起到校验字节前一字节的 8 位累加和。
 ******************************************************************************
 */
#ifndef PC_LINK_H
#define PC_LINK_H

#include "pc_link_config.h"

#if PC_LINK_ENABLE

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/* 帧字段常量 */
#define PC_LINK_HEADER_0               0xAAU
#define PC_LINK_HEADER_1               0x55U
#define PC_LINK_TYPE_PERCEPTION        0x10U
#define PC_LINK_TYPE_POSITION          0x11U
#define PC_LINK_TYPE_STATUS            0x20U
#define PC_LINK_TAIL_0                 0x0DU
#define PC_LINK_TAIL_1                 0x0AU

/* 帧长 */
#define PC_LINK_PERCEPTION_FRAME_SIZE  44U
#define PC_LINK_POSITION_FRAME_SIZE    24U
#define PC_LINK_STATUS_FRAME_SIZE      8U

/* 感知帧有效位 flags(与上位机 perception_flag_t 一致) */
#define PC_LINK_FLAG_RED_VALID         (1U << 0)
#define PC_LINK_FLAG_BLUE_VALID        (1U << 1)
#define PC_LINK_FLAG_BALL_VALID        (1U << 2)

/* 位置帧有效位 flags(与上位机 position_flag_t 一致) */
#define PC_LINK_FLAG_FIELD_VALID       (1U << 0)

/**
 * @brief 感知数据:红蓝块 + 金球位置,单位 m
 */
typedef struct
{
    float red_x_m;
    float red_y_m;
    float red_z_m;
    float blue_x_m;
    float blue_y_m;
    float blue_z_m;
    float ball_x_m;
    float ball_y_m;
    float ball_z_m;
    uint8_t flags;              /* PC_LINK_FLAG_* 组合,无效位对应的数据已清零 */
} pc_perception_t;

/**
 * @brief 位置数据:机器人赛场坐标 + 旋转四元数 w 分量
 */
typedef struct
{
    float field_x_m;
    float field_y_m;
    float field_z_m;
    float field_w;
    uint8_t flags;              /* PC_LINK_FLAG_FIELD_VALID */
} pc_position_t;

/**
 * @brief 初始化小电脑串口:按配置宏选定串口并重配波特率,启动中断接收
 * @retval HAL 状态
 */
HAL_StatusTypeDef PcLink_Init(void);

/**
 * @brief 周期调用:解析环形缓冲区字节流、处理数据超时、周期回传状态帧
 * @note  建议在通信任务中以 1ms 周期调用(与 ComputerLink_Run 并列)
 */
void PcLink_Run(void);

/**
 * @brief 串口单字节接收完成回调(ISR 上下文)
 * @param uart 触发回调的串口句柄
 */
void PcLink_RxCplt(UART_HandleTypeDef *uart);

/**
 * @brief 串口错误回调,请求恢复接收(ISR 上下文)
 * @param uart 触发错误的串口句柄
 */
void PcLink_Error(UART_HandleTypeDef *uart);

/**
 * @brief 读取最新感知数据(红蓝块 / 金球)
 * @param perception 输出缓冲
 * @retval true 数据中有至少一个目标有效;false 无有效目标或参数为空
 */
bool PcLink_GetPerception(pc_perception_t *perception);

/**
 * @brief 读取最新位置数据(赛场坐标 + 旋转角)
 * @param position 输出缓冲
 * @retval true 定位有效;false 定位无效或参数为空
 */
bool PcLink_GetPosition(pc_position_t *position);

/**
 * @brief 设置回传上位机的板端状态(状态机状态 / 错误码)
 * @param state 机器人状态机状态
 * @param error 板端错误码
 */
void PcLink_SetStatus(uint8_t state, uint8_t error);

/**
 * @brief 查询小电脑链路是否在线(超时内收到过感知帧或位置帧)
 * @retval true 在线;false 离线
 */
bool PcLink_IsOnline(void);

/**
 * @brief 读取链路统计:累计收到的好帧数与校验失败帧数(CRC 错误数)
 * @param frames     输出:通过校验的帧数
 * @param crc_errors 输出:帧尾/校验和错误帧数
 */
void PcLink_GetStats(uint32_t *frames, uint32_t *crc_errors);

/**
 * @brief 读取位置帧序号(累计收到且通过校验的 0x11 帧数)
 * @note  上位机感知帧/位置帧交替发送,消费者可用该值判断是否收到
 *        "新的"位置帧,避免同一帧被周期任务重复消费
 */
uint32_t PcLink_GetPositionSeq(void);

#else /* PC_LINK_ENABLE == 0:全部接口退化为空操作 */

#define PcLink_Init()              (HAL_OK)
#define PcLink_Run()               ((void)0)
#define PcLink_RxCplt(uart)        ((void)(uart))
#define PcLink_Error(uart)         ((void)(uart))
#define PcLink_GetPerception(p)    (false)
#define PcLink_GetPosition(p)      (false)
#define PcLink_SetStatus(s, e)     ((void)(s), (void)(e))
#define PcLink_IsOnline()          (false)
#define PcLink_GetStats(f, c)      ((void)(f), (void)(c))
#define PcLink_GetPositionSeq()    (0U)

#endif /* PC_LINK_ENABLE */

#endif /* PC_LINK_H */
