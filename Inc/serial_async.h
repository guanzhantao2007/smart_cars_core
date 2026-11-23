#ifndef __SERIAL_ASYNC_H
#define __SERIAL_ASYNC_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * USART2 异步收发层（VOFA 调参专用）
 *
 * 硬件：PA2 = TX，PA3 = RX，115200 8N1，无流控（配置在 usart.c / .ioc 里，本文件不动它）
 *
 * 【为什么需要这一层】
 *   波形要 50 Hz 稳定发，而 HAL_UART_Transmit() 是阻塞的：一帧约 30 字节，
 *   115200 下要 2.6 ms。如果放在 10 ms 控制中断里同步发，等于把 26% 的节拍
 *   花在等串口上；放在主循环又会被 OLED 的 I2C 阻塞拉长间隔，横轴就失真了。
 *   所以必须有「塞进去就返回」的非阻塞发送。
 *
 * 【两套接口，用之前想清楚在哪调用】
 *   Serial_SendAsync()  非阻塞，可在中断里调（波形打点就用它）。
 *   Serial_GetLine()    只能在主循环调（内部用 HAL_GetTick 判超时、要拼字符串）。
 *
 * 【和参照工程（STM32F1 那台小车）的差别】
 *   那边是自己写 USART1 寄存器 + 环形缓冲；这边 USART2 已经由 CubeMX 接管，
 *   MspInit 也是 HAL 生成的，硬塞寄存器级 ISR 会和 HAL 状态机打架。
 *   所以这里改用 HAL 的 IT 模式，语义一样（非阻塞、可在中断里调），
 *   代价是发送只有「单帧在飞」：上一帧没发完时新帧直接丢掉，不排队。
 *   50 Hz 一帧约 30 字节，占空比 13%，正常不会丢；丢了也只是少一个点。
 *
 * 【一个必须知道的坑】
 *   串口一旦溢出（ORE），HAL 会把接收状态机置成错误并【不再自动重挂接收】，
 *   现象是「命令突然永远收不到」，而且没有任何报错。所以
 *   HAL_UART_ErrorCallback() 里必须重新 HAL_UART_Receive_IT()，
 *   见本文件实现，删了这段调着调着就哑。
 */

/**
  * @brief  打开 USART2 的 NVIC 与接收中断
  * @note   必须在 MX_USART2_UART_Init() 之后调用。
  *         本函数只做「挂中断」，不重配波特率和引脚（那些归 CubeMX 管）。
  */
void Serial_Init(void);

/**
  * @brief  非阻塞发送：拷进内部缓冲就开始发，立即返回
  * @param  s   数据首地址
  * @param  len 字节数（超过内部缓冲容量会被截断，波形帧不会那么长）
  * @note   可在中断上下文调用。上一帧还在发时本次直接丢弃，不排队、不等待——
  *         波形丢一帧只是少一个点，等下去会把控制环卡死。
  */
void Serial_SendAsync(const char *s, uint16_t len);

/**
  * @brief  从接收缓冲里拼出一整行
  * @param  out 调用者缓冲区，取到的行以 '\0' 结尾（\r\n 已去掉）
  * @param  max out 容量（至少 2）
  * @retval 1 = 取到一行，0 = 还没有完整行
  * @note   只能在主循环调用。两条收尾路径：
  *           1. 见到 \r 或 \n —— 常规路径
  *           2. 半行静默 15 ms —— 兜底。VOFA 控件若没在命令末尾补换行，
  *              没有这条路径就会表现为「滑条拖了但下位机没反应」。
  */
int Serial_GetLine(char *out, uint16_t max);

/**
  * @brief  紧急发送：绕过 HAL 状态机，直接轮询 TXE 写 DR（只给故障路径用）
  * @param  s 以 '\0' 结尾的字符串
  * @note   为什么非要有这么个"野路子"：
  *         Error_Handler 里要吐一行字告诉人"我停在哪一步"，但那时
  *         HAL 的 gState 很可能还停在 BUSY_TX（上一帧没发完），
  *         HAL_UART_Transmit() 会直接返回 HAL_BUSY —— 一个字都发不出来，
  *         于是"卡在初始化"和"线没接"在串口上看起来完全一样。
  *         这里不看 gState、不看 hdmatx，等得到 TXE 就写，等不到就放弃。
  *         代价：会打断正在用 HAL_UART_Transmit_IT 发的那一帧（波形上出现
  *         半截/凹口）。调试期可以接受，正式版应该只在 Fault_Report 里调。
  */
void Serial_EmergencyPuts(const char *s);

/**
  * @brief  串口出错后重新挂接收（由 HAL_UART_ErrorCallback 转调）
  * @note   本工程里 HAL_UART_ErrorCallback 已经在 stm32f4xx_it.c 定义了一份
  *         （它负责 USART1 的上位机帧错误），所以不能在 serial_async.c 里
  *         再定义同名回调。那里加一个 USART2 分支调本函数即可。
  *         漏了这一句，串口溢出一次就永久收不到命令，而且不报错。
  */
void Serial_RxRecover(void);

#endif
