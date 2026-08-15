#ifndef COMPUTER_LINK_H
#define COMPUTER_LINK_H

#include "stm32h7xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/* 手动速度指令(底盘坐标系:RPM/控制量) */
typedef struct
{
    int16_t vx;
    int16_t vy;
    int16_t z;
} computer_cmd_t;

/**
 * @brief 初始化电脑端 UART 接收
 * @param uart 电脑无线串口句柄
 * @retval HAL 状态
 */
HAL_StatusTypeDef ComputerLink_Init(UART_HandleTypeDef *uart);

/**
 * @brief 电脑链路周期任务:解析缓存、更新链路存活
 * @note  不再直接写底盘;底盘指令统一由 PathRunner_Arbitrate 仲裁
 */
void ComputerLink_Run(void);

void ComputerLink_RxCplt(UART_HandleTypeDef *uart);
void ComputerLink_Error(UART_HandleTypeDef *uart);

/* ---- 仲裁器查询接口(commTask 单消费者) ---- */

/** 遥控急停是否锁存(最高优先级) */
bool ComputerLink_EstopLatched(void);

/** 急停清除/复位边沿(收到 A5 5D 00,消费一次) */
bool ComputerLink_ResetRequested(void);

/** 人工请求恢复自主边沿(收到 A5 5E 00,消费一次) */
bool ComputerLink_AutoResumeRequested(void);

/** 是否有人工接管请求(仅显式模式帧 A5 5E 01;速度/动作帧不触发切换) */
bool ComputerLink_ManualRequested(void);

/** 取本周期手动速度指令(消费) */
bool ComputerLink_GetCommand(computer_cmd_t *cmd);

/** 取本周期手动动作指令(消费) */
bool ComputerLink_GetAction(uint8_t *action);

/** 操作手链路是否存活(500ms 内收到过合法帧) */
bool ComputerLink_LinkOnline(void);

#endif
