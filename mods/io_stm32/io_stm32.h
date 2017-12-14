/*
 * driver.h
 *
 *  Created on: Apr 16, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#ifndef _INCLUDE_IO_STM32_H_
#define _INCLUDE_IO_STM32_H_

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <typedef.h>

//摇控器接收机的4个通道读数范围
#define CTL_PWM_MIN (1000)
#define CTL_PWM_MAX (2000)
//摇控器幅度通道比例范围,通过此通道来修改方向通道的数值比例
#define CTL_DI_MIN (1000)
#define CTL_DI_MAX (2000)
//最大倾斜角
#define MAX_ANGLE (30)
#define MOTOR_COUNT	(4)

#define BUFFER_SIZE	(256)

#define	RC_POS_START1                         	0
#define	RC_POS_START2	                      	1
#define	RC_POS_LEN	                      		2
#define	RC_POS_DATA	                      		3
#define	RC_POS_CRC1 	                      	19
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

//

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

s32 __init(s_engine* engine, s_params* params);

s32 __destory(s_engine* e, s_params* p);

s32 __status();

void motor_balance_compensation();

void read_controller();

int crc8_check(u8 *buff, u8 len, u8 crc8);

int crc16_check(u8 *buff, u8 len, u16 crc16);

u16 crc16_value(u8 *buff, u8 len);

int frame_send_rc_data(u16 *pwm);

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);

#endif /* INCLUDE_DRIVER_H_ */
