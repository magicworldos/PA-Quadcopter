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

#define BUFF_SIZE	(2*0x400)

s32 __init(s_engine* e, s_params* p);

s32 __destory(s_engine* e, s_params* p);

s32 __status();

void logger_run();

int config_env(char* value, char *name);

#endif /* INCLUDE_DRIVER_H_ */
