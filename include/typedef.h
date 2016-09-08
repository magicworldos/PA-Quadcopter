/*
 * typedef.h
 *
 *  Created on: Mar 4, 2016
 *      Author: lidq
 */

#ifndef _INCLUDE_TYPEDEF_H_
#define _INCLUDE_TYPEDEF_H_

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <semaphore.h>

#include <wiringPi.h>
#include <if_mpu6050.h>


#define null						((void *)0)

typedef unsigned int boolean;

typedef char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;

#endif /* INCLUDE_TYPEDEF_H_ */
