/*
 * config.c
 *
 *  Created on: Dec 28, 2017
 *      Author: lidq
 */

#include <config.h>

int config_env(char* value, char *name)
{
	if (name == NULL || value == NULL)
	{
		return -1;
	}
	char* v = getenv(name);
	memcpy(value, v, strlen(v) + 1);

	return 0;
}

//int config_read(char *path)
//{
//	char line[256];
//	int linenum = 0;
//	while (fgets(line, 256, file) != NULL)
//	{
//		char ip[256], mac[256];
//
//		linenum++;
//		if (line[0] == '#')
//			continue;
//
//		if (sscanf(line, "%s %s", ip, mac) != 2)
//		{
//			fprintf(stderr, "Syntax error, line %d\n", linenum);
//			continue;
//		}
//
//		printf("Line %d:  IP %s MAC %s\n", linenum, ip, mac);
//	}
//}
