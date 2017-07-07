/*
 * command.c
 *
 *  Created on: Sep 27, 2016
 *      Author: lidq
 */

#include <command.h>

s32 st = 0;

s32 __init(s_engine* e, s_params* p)
{
	st = 1;

	printf("[ OK ] Command Init.\n");

	return 0;
}

s32 __destory(s_engine* e, s_params* p)
{
	st = 0;
	return 0;
}

s32 __status() { return st; }
