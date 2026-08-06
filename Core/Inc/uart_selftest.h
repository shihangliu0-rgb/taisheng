#ifndef UART_SELFTEST_H
#define UART_SELFTEST_H

/* =====================================================================
 *  串口自测（诊断用）—— 隔离、宏开关、阻塞发送、不影响原有功能
 * ---------------------------------------------------------------------
 *  开启：取消下一行注释(或在 Keil Options->C/C++->Define 加 UART_SELFTEST)。
 *  开启后：main() 会调用 UartSelftest_Run()，它【自己初始化】下列串口(不依赖 main
 *         的 MX_*_Init、不碰 usart.c)，用【阻塞 HAL_UART_Transmit】(无中断、无 DMA)
 *         依次循环发送 "xxx is ok"，并【不会启动 FreeRTOS 调度器】，与原有功能完全隔离：
 *           USART1(PB14/PB15) USART2(PA2/PA3) USART3(PB10/PB11) UART4(PA0/PA1)
 *           USART6(PG14/PG9)  UART7(PE8/PE7)  UART8(PE1/PE0)   (均 115200 8N1)
 *  用法：拿串口助手/示波器逐个量上述引脚，谁有 "xxx is ok" 输出就是该 UART 在工作；
 *        没输出即该串口硬件或接线有问题。要加/减串口，改 uart_selftest.c 里的 uart_list 表。
 *  关闭时：本模块不参与编译，main.c 里的调用也被 #ifdef 挡掉，零影响。
 * ===================================================================== */

/* #define UART_SELFTEST */

#ifdef UART_SELFTEST
/* 阻塞运行，不返回(用于 main 里替代正常流程) */
void UartSelftest_Run(void);
#endif

#endif /* UART_SELFTEST_H */
