/*
 * command.h
 *
 *  Created on: Sep 27, 2016
 *      Author: lidq
 */

#ifndef MODS_COMMAND_COMMAND_H_
#define MODS_COMMAND_COMMAND_H_

#include <typedef.h>

int __init(s_engine *e, s_params *p);

int __destory(s_engine *e, s_params *p);

int __status();

#endif /* MODS_COMMAND_COMMAND_H_ */
