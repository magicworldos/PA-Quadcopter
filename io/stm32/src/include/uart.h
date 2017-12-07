/*
 * uart.h
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_

#include <typedef.h>

void UART1_GPIO_Configuration(void);

void UART1_Configuration(void);

u8 Uart1_PutChar(u8 ch);

void Uart1_PutString(u8* buf, u8 len);

void USART1_IRQHandler(void);

void UART_Init();

#endif /* SRC_UART_H_ */
