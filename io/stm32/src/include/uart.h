/*
 * uart.h
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_

#include <typedef.h>

#define BUFFER_SIZE	(32)

struct uart_buffer_s
{
	s16 head;
	s16 tail;
	s16 size;
	u8 buffer[BUFFER_SIZE];
	u32 total_len;
	u32 over;
	u32 user_buf_over;
};

#define	PWM_POS_START1                         	0
#define	PWM_POS_START2	                      	1
#define	PWM_POS_LEN	                      		2
#define	PWM_POS_DATA	                      	3
#define	PWM_POS_CRC1 	                      	11
#define	PWM_POS_CRC2  	                      	12
#define	PWM_POS_END1  	                     	13
#define	PWM_POS_END2  	                     	14

#define PWM_BYTE_HEAD_1                         0X55
#define PWM_BYTE_HEAD_2                         0XAA
#define PWM_BYTE_END_1                          0XA5
#define PWM_BYTE_END_2		                   	0X5A

#define PWM_HEAD                                0
#define PWM_LEN    								1
#define PWM_END    								2

void UART1_GPIO_Configuration(void);

void UART1_Configuration(void);

u8 Uart1_PutChar(u8 ch);

void Uart1_PutString(u8* buf, u8 len);

void USART1_IRQHandler(void);

void UART_Init();

int uart_buffer_count(struct uart_buffer_s *lb);

int parse_mag_feedback();

int get_motor_pwm();

#endif /* SRC_UART_H_ */
