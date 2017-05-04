/*
 * list.h
 *
 *  Created on: Sep 27, 2016
 *      Author: lidq
 */

#ifndef INCLUDE_LIST_H_
#define INCLUDE_LIST_H_

#include <typedef.h>

//表节点
typedef struct s_node
{
	void* data;
	struct s_node* next;
} s_node;

//表数据结构
typedef struct
{
	//头节点
	s_node* header;
	//用于释放节点数据的回调函数
	int (*free_node)();
} s_list;

//构建一个顺序线性表
int list_init(s_list* list, int (*free_node)());

//销毁表
int list_destroy(s_list* list);

//清空表中所有元素
int list_clear(s_list* list);

//插入一个新元素
int list_insert(s_list* list, void* data);

//对每个元素执行visit操作
int list_visit(s_list* list, void (*callback)());

#endif /* INCLUDE_LIST_H_ */
