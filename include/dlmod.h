/*
 * dlmod.h
 *
 *  Created on: Sep 27, 2016
 *      Author: lidq
 */

#ifndef INCLUDE_DLMOD_H_
#define INCLUDE_DLMOD_H_

#include <typedef.h>
#include <list.h>
#include <engine.h>

typedef struct s_dlmod
{
	void *handler;
	int (*init)();
	int (*destory)();
	int (*status)();
	void **args;
} s_dlmod;

void dlmod_run_init(void *args);

int dlmod_run_pt_init(s_dlmod *mod);

void dlmod_run_destory(void *args);

int dlmod_run_pt_destory(s_dlmod *mod);

int dlmod_dlclose(s_dlmod *mod);

int dlmod_free_mod(s_dlmod *mod);

int dlmod_init();

int dlmod_mods_status();

int dlmod_destory();

#endif /* INCLUDE_DLMOD_H_ */
