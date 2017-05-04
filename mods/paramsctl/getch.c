/*
 * getch.c
 *
 *  Created on: May 11, 2016
 *
 *  四轴飞行控制器  Copyright (C) 2016  李德强
 */

#include <getch.h>

struct termios tm, tm_old;
void initTermios(int echo)
{
	tcgetattr(0, &tm_old);
	tm = tm_old;
	tm.c_lflag &= ~ICANON;
	tm.c_lflag &= echo ? ECHO : ~ECHO;
	tcsetattr(0, TCSANOW, &tm);
}

void resetTermios() { tcsetattr(0, TCSANOW, &tm_old); }

char getch_(int echo)
{
	char ch;
	initTermios(echo);
	ch = getchar();
	resetTermios();
	return ch;
}

char getch(void) { return getch_(0); }

char getche(void) { return getch_(1); }
