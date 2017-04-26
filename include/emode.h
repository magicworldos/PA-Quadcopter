/*
 * emode.h
 *
 *  Created on: Apr 26, 2017
 *      Author: lidq
 */

#ifndef INCLUDE_EMODE_H_
#define INCLUDE_EMODE_H_

#include <typedef.h>

//陀螺仪读数
void emode_start_gyro(char* argv2);

//摇控器校准
void emode_start_control();

//电机调试
void emode_start_test(char* argv2, char* argv3, char* argv4);

//电机调试
void emode_ent_run(int en_port, int en_speed, int en_msecs);

#endif /* INCLUDE_EMODE_H_ */
