/*
 * display.h
 *
 *  Created on: Sep 27, 2016
 *      Author: lidq
 */

#ifndef MODS_DISPLAY_DISPLAY_H_
#define MODS_DISPLAY_DISPLAY_H_

#include <typedef.h>

//多信息同行显示
#define __DISPLAY_MODE_MORE__
//显示频度
#define DISPLAY_SPEED (10)

s32 __init(s_engine* engine, s_params* params);

s32 __destory(s_engine* e, s_params* p);

s32 __status();

void display_run();

//取得显示开关
s32 display_get_onoff(s32 bit);

#endif /* MODS_DISPLAY_DISPLAY_H_ */
