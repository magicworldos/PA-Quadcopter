/*
 * getch.h
 *
 *  Created on: May 11, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#ifndef MODS_PARAMSCTL_GETCH_H_
#define MODS_PARAMSCTL_GETCH_H_

#include <stdio.h>
#include <termios.h>
#include <typedef.h>

void initTermios(s32 echo);

void resetTermios();

char getch_(s32 echo);

char getch(void);

char getche(void);

#endif /* MODS_PARAMSCTL_GETCH_H_ */
