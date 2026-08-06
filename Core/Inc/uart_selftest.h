#ifndef UART_SELFTEST_H
#define UART_SELFTEST_H

/* =====================================================================
 *  串口自测（诊断用）—— 隔离、宏开关、阻塞发送、不影响原有功能
 * ---------------------------------------------------------------------
 *  开启：取消下一行注释(或在 Keil Options->C/C++->Define 加 UART_SELFTEST)。
 *  开启后：main() 在初始化完各串口后会调用 UartSelftest_Run()，
 *         用【阻塞 HAL_UART_Transmit】(无中断、无 DMA) 依次向
 *         USART1(PB14/PB15)、USART2(PA2/PA3)、UART4(PA0/PA1) 循环发送
 *         "xxx is ok"，并【不会启动 FreeRTOS 调度器】，与原有功能完全隔离。
 *  用法：拿串口助手/示波器逐个量上述引脚，谁有 "xxx is ok" 输出就是好的，
 *        没输出即该串口硬件或引脚有问题。
 *  关闭时：本模块不参与编译，main.c 里的调用也被 #ifdef 挡掉，零影响。
 * ===================================================================== */

/* #define UART_SELFTEST */

#ifdef UART_SELFTEST
/* 阻塞运行，不返回(用于 main 里替代正常流程) */
void UartSelftest_Run(void);
#endif

#endif /* UART_SELFTEST_H */
