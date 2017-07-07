/*
 * dlmod.h
 *
 *  Created on: Sep 27, 2016
 *      Author: lidq
 */

#ifndef INCLUDE_DLMOD_H_
#define INCLUDE_DLMOD_H_

#include <engine.h>
#include <list.h>
#include <typedef.h>

typedef struct s_dlmod
{
	void* handler;
	s32 (*init)();
	s32 (*destory)();
	s32 (*status)();
	void** args;
} s_dlmod;

//读取lib文件夹，并载入*.so动态链接库
s32 dlmod_init();

//销毁模块链表
s32 dlmod_destory();

//载入一个*.so的动态链接库
s_dlmod* dlmod_open(char* filename);

//关闭一个动态链接库
s32 dlmod_dlclose(s_dlmod* mod);

//释放内存资源
s32 dlmod_free_mod(s_dlmod* mod);

//取得当前模块状态
s32 dlmod_mods_status();

//执行__init函数
void dlmod_run_init(void* args);

//采用多线程方式调用__init函数
s32 dlmod_run_pt_init(s_dlmod* mod);

//执行__destory函数
void dlmod_run_destory(void* args);

//采用多线程方式调用__destory函数
s32 dlmod_run_pt_destory(s_dlmod* mod);

#endif /* INCLUDE_DLMOD_H_ */
