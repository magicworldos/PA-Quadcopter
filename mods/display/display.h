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

int __init(s_engine* engine, s_params* params);

int __destory(s_engine* e, s_params* p);

int __status();

void run();

#endif /* MODS_DISPLAY_DISPLAY_H_ */
