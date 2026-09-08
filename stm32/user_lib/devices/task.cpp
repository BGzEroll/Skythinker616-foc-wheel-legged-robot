// #include "task.h"

// static task_t *head = NULL, *tail = NULL;		// 定义头尾链表指针与当前链表指针

// /**
//  * @brief 检查是否到达任务运行时间
//  */
// static uint8_t check_time(task_t *t)
// {
// 	uint32_t now = get_ms_tick();
	
// 	if(now - t->last_time >= t->loop_time)
// 	{
// 		t->last_time += t->loop_time;		// 更新时间
// 		return 1;
// 	}
// 	return 0;
// }

// /**
//  * @brief 创建调度器任务
//  */
// static void create(task_t *new_task, void (*task_callback)(uint32_t tick), uint32_t loop_time)
// {
// 	new_task->loop_time = loop_time;		// 设置定时时间
// 	new_task->task_callback = task_callback;		// 设置回调函数
// 	new_task->p_next = NULL;		// 尾节点指向NULL
	
// 	if(head)
// 	{
// 		tail->p_next = new_task;
// 		tail = new_task;
// 	}
// 	else
// 	{
// 		head = new_task;
// 		tail = new_task;
// 	}
// }

// /**
//  * @brief 循环查询任务列表是否到达定时时间
//  */
// static void loop(void)
// {
// 	task_t *p = head;
// 	while(p)
// 	{
// 		if(check_time(p)){p->task_callback(p->loop_time);}        // 到点就调用对应任务的回调函数
// 		p = p->p_next;			// 移动到下一个任务
// 	}
// }

// // 任务操作函数
// const task_ops_t task = {
// 	.create = create,
// 	.loop = loop
// };
