/*
 * dlmod.c
 *
 *  Created on: Sep 27, 2016
 *      Author: lidq
 */

#include <dlmod.h>

//引擎
extern s_engine engine;
//参数
extern s_params params;

pthread_t pthdmod;

s_list list;

void dlmod_run_init(void *args)
{
	if (args == NULL)
	{
		return;
	}
	void **ags = (void **) args;
	s_dlmod *mod = ags[0];
	if (mod == NULL)
	{
		return;
	}
	mod->init(ags[1], ags[2]);
}

int dlmod_run_pt_init(s_dlmod *mod)
{
	if (mod == NULL)
	{
		return -1;
	}

	if (mod->init == NULL)
	{
		return -1;
	}

	pthread_create(&pthdmod, (const pthread_attr_t*) NULL, (void* (*)(void*)) &dlmod_run_init, mod->args);

	return 0;
}

void dlmod_run_destory(void *args)
{
	if (args == NULL)
	{
		return;
	}
	void **ags = (void **) args;
	s_dlmod *mod = ags[0];
	if (mod == NULL)
	{
		return;
	}
	mod->destory(ags[1], ags[2]);
}

int dlmod_run_pt_destory(s_dlmod *mod)
{
	if (mod == NULL)
	{
		return -1;
	}

	if (mod->init == NULL)
	{
		return -1;
	}

	pthread_create(&pthdmod, (const pthread_attr_t*) NULL, (void* (*)(void*)) &dlmod_run_destory, mod->args);

	return 0;
}

int dlmod_dlclose(s_dlmod *mod)
{
	if (mod == NULL)
	{
		return -1;
	}

	if (mod->handler == NULL)
	{
		return -1;
	}

	dlclose(mod->handler);

	return 0;
}

int dlmod_free_mod(s_dlmod *mod)
{
	if (mod == NULL)
	{
		return -1;
	}

	if (mod->args != NULL)
	{
		free(mod->args);
	}

	free(mod);
	return 0;
}

int dlmod_init()
{
	list_init(&list, &dlmod_free_mod);

	//准备读取工作目录
	DIR *dir;
	struct dirent *dirinfo;
	//打开工作目录
	if ((dir = opendir("./lib")) == NULL)
	{
		return -1;
	}
	char filename[0x200];
	//循环读入每一个文件
	while ((dirinfo = readdir(dir)) != NULL)
	{
		//如果是普通文件
		if (dirinfo->d_type == 8)
		{
			snprintf(filename, 0x200, "%s/%s", "./lib", dirinfo->d_name);
			//文件名
			printf("%s\n", filename);

			void *handler = dlopen(filename, RTLD_LAZY);
			if (handler == NULL)
			{
				continue;
			}

			int (*init)()= dlsym(handler, "__init");
			if (init == NULL)
			{
				dlclose(handler);
				continue;
			}

			int (*destory)()= dlsym(handler, "__destory");
			if (destory == NULL)
			{
				dlclose(handler);
				continue;
			}

			int (*status)()= dlsym(handler, "__status");
			if (status == NULL)
			{
				dlclose(handler);
				continue;
			}

			s_dlmod *mod = malloc(sizeof(s_dlmod));
			mod->handler = handler;
			mod->init = init;
			mod->destory = destory;
			mod->status = status;

			mod->args = malloc(sizeof(void *) * 4);
			mod->args[0] = mod;
			mod->args[1] = &engine;
			mod->args[2] = &params;
			mod->args[3] = &status;

			list_insert(&list, mod);
		}
	}
	//关闭文件夹
	closedir(dir);

	list_visit(&list, (void *) &dlmod_run_pt_init);

	return 0;
}

int dlmod_mods_status()
{
	s_node *p = list.header;
	while (p != NULL)
	{
		s_dlmod *mod = (s_dlmod *) p->data;
		if (mod->status())
		{
			return 1;
		}
		p = p->next;
	}
	printf("all clear.\n");
	return 0;
}

int dlmod_destory()
{
	list_visit(&list, (void *) &dlmod_run_pt_destory);
	for (int i = 0; i < 3000; i++)
	{
		if (dlmod_mods_status() == 0)
		{
			printf("Done.\n");
			break;
		}

		usleep(1000);
	}
	list_visit(&list, (void *) &dlmod_dlclose);
	list_destroy(&list);

	return 0;
}
