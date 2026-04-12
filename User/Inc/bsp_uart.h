#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "usart.h"
#include <stdint.h>

/* 串口接收缓存（以 '\\n' 或 '\\r' 结束一条命令） */
extern char rx_buffer[96];
/* USART 中断接收的当前字节 */
extern uint8_t rx_data;
/* 新命令就绪标志：1=有待处理命令，0=无 */
extern volatile uint8_t new_cmd_flag;

/**
 * @brief 初始化 USART1 中断接收（单字节循环接收）
 * @param 无
 * @return 无
 */
void UART_Init_Receive(void);

/**
 * @brief 解析并处理一条串口命令
 * @param 无
 * @return 无
 */
void UART_ProcessCommand(void);

#endif
