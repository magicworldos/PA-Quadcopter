/*
 * getch.c
 *
 *  Created on: May 11, 2016
 *      Author: lidq
 */

#include <getch.h>

struct termios tm, tm_old;
/* Initialize new terminal i/o settings */
void initTermios(int echo)
{
	tcgetattr(0, &tm_old); /* grab old terminal i/o settings */
	tm = tm_old; /* make new settings same as old settings */
	tm.c_lflag &= ~ICANON; /* disable buffered i/o */
	tm.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
	tcsetattr(0, TCSANOW, &tm); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios()
{
	tcsetattr(0, TCSANOW, &tm_old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo)
{
	char ch;
	initTermios(echo);
	ch = getchar();
	resetTermios();
	return ch;
}

/* Read 1 character without echo */
char getch(void)
{
	return getch_(0);
}

/* Read 1 character with echo */
char getche(void)
{
	return getch_(1);
}
