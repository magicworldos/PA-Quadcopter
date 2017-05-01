/*
 * automatic.h
 *
 *  Created on: Oct 18, 2016
 *      Author: lidq
 */

#ifndef MODS_FHEIGHT_FHEIGHT_H_
#define MODS_FHEIGHT_FHEIGHT_H_

#include <typedef.h>

#define FHEIGHT_MAX_SPEED	(10)

int __init(s_engine *engine, s_params *params);

int __destory(s_engine *e, s_params *p);

int __status();

void fheight_automatic();

float fheight_pid(float et, float et_1, float *sum);

void fheight_limit(float *v);

void fheight_ev_limit(float *v);

#endif /* MODS_FHEIGHT_FHEIGHT_H_ */
