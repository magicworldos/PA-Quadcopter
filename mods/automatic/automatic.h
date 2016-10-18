/*
 * automatic.h
 *
 *  Created on: Oct 18, 2016
 *      Author: lidq
 */

#ifndef MODS_AUTOMATIC_AUTOMATIC_H_
#define MODS_AUTOMATIC_AUTOMATIC_H_

#include <typedef.h>

int __init(s_engine *engine, s_params *params);

int __destory(s_engine *e, s_params *p);

int __status();

void automatic();

#endif /* MODS_AUTOMATIC_AUTOMATIC_H_ */
