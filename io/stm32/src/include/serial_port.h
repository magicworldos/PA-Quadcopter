/*
 * pwm_out.h
 *
 *  Created on: Jun 25, 2017
 *      Author: lidq
 */

#ifndef __SERIAL_PORT_H
#define __SERIAL_PORT_H

#include <typedef.h>

#define USART_BAUDRATE							(115200)
#define BUFF_SIZE								(32)
#define LIMIT_MAX								(2000)
#define LIMIT_MIN								(0)
#define PWM_ERR_MAX								(10)

#define	RC_POS_START1                         	0
#define	RC_POS_START2	                      	1
#define	RC_POS_LEN	                      		2
#define	RC_POS_DATA	                      		3
#define	RC_POS_CRC1  	                      	19
#define	RC_POS_CRC2  	                      	20
#define	RC_POS_END1  	                     	21
#define	RC_POS_END2  	                     	22

#define RC_BYTE_HEAD_1                         	0X55
#define RC_BYTE_HEAD_2                         	0XAA
#define RC_BYTE_END_1                          	0XA5
#define RC_BYTE_END_2		                   	0X5A

#define RC_HEAD                                 0
#define RC_LEN    								1
#define RC_END    								2

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

typedef struct s_serial_port
{
	s16 head;
	s16 tail;
	s16 size;
	u8 buffer[BUFF_SIZE];
	u32 total_len;
	u32 over;
	u32 user_buf_over;
} s_serial_port;

void serial_port_init();

void serial_port_gpio_config();

void serial_port_mode_config();

u8 serial_port_putchar(u8 ch);

void serial_port_puts(u8* buf, u8 len);

int serial_port_buff_count(s_serial_port *sp);

int serial_port_frame_pase();

void serial_port_limit(u16 *value, int len);

int serial_port_frame_recv_pwm(u16 *pwm);

int serial_port_frame_send_rc(u16 *rc);

#endif /* SRC_UART_H_ */
