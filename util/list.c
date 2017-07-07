/*
 * list.c
 *
 *  Created on: Sep 27, 2016
 *      Author: lidq
 */

#include <list.h>

//构建一个顺序线性表
s32 list_init(s_list* list, s32 (*free_node)())
{
	if (list == NULL)
	{
		return -1;
	}
	list->free_node = free_node;
	list->header    = NULL;
	return 0;
}

//销毁表
s32 list_destroy(s_list* list)
{
	if (list == NULL)
	{
		return -1;
	}

	if (list->header == NULL)
	{
		return 0;
	}

	list_clear(list);

	return 0;
}

//清空表中所有元素
s32 list_clear(s_list* list)
{
	if (list == NULL)
	{
		return -1;
	}

	//从头节点开始释放
	s_node* p = list->header;
	while (p != NULL)
	{
		s_node* t = p;
		p	 = p->next;
		//释放内存
		list->free_node(t->data);
		free(t);
	}
	list->header = NULL;

	return 0;
}

//在第i个位置前插入一个新元素
s32 list_insert(s_list* list, void* data)
{
	if (list == NULL || data == NULL)
	{
		return -1;
	}

	//申请新节点内存
	s_node* p_new = (s_node*)malloc(sizeof(s_node));
	p_new->data   = data;
	p_new->next   = list->header;
	list->header  = p_new;

	return 0;
}

//对每个元素执行visit操作
s32 list_visit(s_list* list, void (*callback)())
{
	if (list == NULL || list->header == NULL || callback == NULL)
	{
		return -1;
	}

	//顺序访问每一个元素
	s_node* p = list->header;
	while (p != NULL)
	{
		s_node* pvisit = p;
		p	      = p->next;
		callback(pvisit->data);
	}

	return 0;
}
