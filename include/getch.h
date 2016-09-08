/*
 * getch.h
 *
 *  Created on: May 11, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#ifndef _INCLUDE_GETCH_H_
#define _INCLUDE_GETCH_H_

#include <stdio.h>
#include <termios.h>

void initTermios(int echo);

void resetTermios();

char getch_(int echo);

char getch(void);

char getche(void);

#endif /* INCLUDE_GETCH_H_ */
