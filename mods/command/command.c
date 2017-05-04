/*
 * command.c
 *
 *  Created on: Sep 27, 2016
 *      Author: lidq
 */

#include <command.h>

int st = 0;

int __init(s_engine* e, s_params* p)
{
	st = 1;

	printf("[ OK ] Command Init.\n");

	return 0;
}

int __destory(s_engine* e, s_params* p)
{
	st = 0;
	return 0;
}

int __status() { return st; }
