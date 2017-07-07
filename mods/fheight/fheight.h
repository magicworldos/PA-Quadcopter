/*
 * automatic.h
 *
 *  Created on: Oct 18, 2016
 *      Author: lidq
 */

#ifndef MODS_FHEIGHT_FHEIGHT_H_
#define MODS_FHEIGHT_FHEIGHT_H_

#include <typedef.h>

#define FHEIGHT_MAX_SPEED (1000)

s32 __init(s_engine* engine, s_params* params);

s32 __destory(s_engine* e, s_params* p);

s32 __status();

void fheight_automatic();

f32 fheight_pid(f32 et, f32 et_1, float* sum);

void fheight_limit(float* v);

void fheight_ev_limit(float* v);

#endif /* MODS_FHEIGHT_FHEIGHT_H_ */
