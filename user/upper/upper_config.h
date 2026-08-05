#ifndef UPPER_CONFIG_H
#define UPPER_CONFIG_H

/* =====================================================================
 *  user/upper —— 上位机调试协议模块 · 总开关
 * ---------------------------------------------------------------------
 *  开启方式（二选一）：
 *    1) 取消下方的  #define UPPER_DEBUG  注释；或
 *    2) 在 Keil 工程 Options -> C/C++(AC6) -> Define 里加入 UPPER_DEBUG
 *
 *  ★ 未定义 UPPER_DEBUG 时，本模块(upper_protocol.c)整体编译为空，
 *    upper_protocol.h 里所有 Upper_* 接口退化为空实现，
 *    对现有 H723 固件【零影响】——既不占用串口，也不会改变任何行为。
 *
 *  串口遵循 H7 既有配置：不修改 usart.c，句柄由调用方(Upper_Init)传入，
 *  默认沿用原来的 &huart4 (UART4 @115200)。
 * ===================================================================== */

/* #define UPPER_DEBUG */

/* ★ 上位机通信串口 —— 723 上为 UART4 (PA0=TX, PA1=RX，已在 usart.c 的
 *   HAL_UART_MspInit 中配好 PA0/PA1, AF8)。改这里即可切换上位机串口，
 *   不必动 freertos.c / usart.c。当前接线与你要求一致(PA0/PA1)。 */
#ifndef UPPER_UART_HANDLE
#define UPPER_UART_HANDLE   huart4
#endif

#ifdef UPPER_DEBUG

/* 通信看门狗：超过该时间(ms)未收到任何上位机帧 -> 制动/失能，
 * 对应协议中 HEARTBEAT(0x01) 的链路保活要求。 */
#ifndef UPPER_WATCHDOG_MS
#define UPPER_WATCHDOG_MS        1000U
#endif

/* 周期遥测 TELEMETRY(0x91) 默认上报周期(ms)，可被 TELEM_CTRL(0x30) 在线修改 */
#ifndef UPPER_TELEM_PERIOD_MS
#define UPPER_TELEM_PERIOD_MS    100U
#endif

/* 协议单帧最大 payload = 240；发送缓冲按最大整帧 247 预留 */
#define UPPER_MAX_PAYLOAD        240U
#ifndef UPPER_TX_BUF_SIZE
#define UPPER_TX_BUF_SIZE        247U
#endif

/* TX 字节环形缓冲，缓存放队列里、在 Upper_Run(任务上下文)中用 IT 发出 */
#ifndef UPPER_TX_RING_SIZE
#define UPPER_TX_RING_SIZE       512U
#endif

/* RX 完整帧环形队列深度(突发收包缓冲) */
#ifndef UPPER_RX_RING_SIZE
#define UPPER_RX_RING_SIZE       8U
#endif

/* 可订阅数据流通道数上限 */
#ifndef UPPER_MAX_SUB_CH
#define UPPER_MAX_SUB_CH         32U
#endif

#endif /* UPPER_DEBUG */
#endif /* UPPER_CONFIG_H */
