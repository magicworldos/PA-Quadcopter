/*
 * getch.h
 *
 *  Created on: May 11, 2016
 *      Author: lidq
 */

#ifndef _INCLUDE_GETCH_H_
#define _INCLUDE_GETCH_H_

#include <stdio.h>
#include <termios.h>

void initTermios(int echo);

/* Restore old terminal i/o settings */
void resetTermios();

/* Read 1 character - echo defines echo mode */
char getch_(int echo);

/* Read 1 character without echo */
char getch(void);

/* Read 1 character with echo */
char getche(void);

#endif /* INCLUDE_GETCH_H_ */
