/*
 * driver.h
 *
 *  Created on: Apr 16, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#ifndef _INCLUDE_MOTOR_H_
#define _INCLUDE_MOTOR_H_

#include <typedef.h>

#define BUFF_SIZE			(8*1024)
#define WRITE_SIZE		(BUFF_SIZE/4)

typedef struct s_buff_rw
{
	u8 *buff;
	u8 *data;
	s32 pos_w;
	s32 pos_r;
	s32 rw_size;
	s32 buff_size;
} s_buff_rw;

s32 __init(s_engine* engine, s_params* params);

s32 __destory(s_engine* e, s_params* p);

s32 __status();

void logger_start();

void logger_stop();

void logger_write_data(int count);

void logger_logging();

void logger_writting();

void logger_run();

void logger_write();

int config_env(char* value, char *name);

#endif /* INCLUDE_DRIVER_H_ */
