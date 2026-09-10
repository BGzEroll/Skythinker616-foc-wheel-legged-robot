#include "task.h"

#include "stm32f1xx_hal.h"

namespace task
{
    namespace
    {
        task_node *head = nullptr, *tail = nullptr;		// 定义头尾链表指针与当前链表指针
    }

    namespace
    {
        /**
         * @brief 获取系统运行时间（毫秒）
         * 
         * @return uint32_t 系统运行时间（毫秒）
         */
        uint32_t get_ms_tick()
        {
            return HAL_GetTick();
        }

        /**
         * @brief 将任务节点添加到链表末尾
         * 
         * @param task 任务节点指针
         */
        void append(task_node *task)
        {
            task->p_next = nullptr;

            if(tail)
            {
                tail->p_next = task;
            }
            else
            {
                head = task;
            }

            tail = task;
            task->registered = true;
        }
    }

    /**
     * @brief 创建一个定时任务
     * 
     * @param task 任务节点指针
     * @param callback 回调函数指针
     * @param period_ms 定时周期（毫秒）
     * 
     * @return true 创建成功
     * @return false 创建失败
     */
    bool create(task_node *task, void (*callback)(), uint32_t period_ms)
    {
        if(!task || !callback || task->registered)
        {
            return false;
        }

        task->last_time = get_ms_tick();
        task->period_ms = period_ms;
        task->callback = callback;

        append(task);

        return true;
    }

    /**
     * @brief 创建一个定时任务
     * 
     * @param task 任务节点指针
     * @param callback 回调函数指针
     * @param period_ms 定时周期（毫秒）
     * 
     * @return true 创建成功
     * @return false 创建失败
     */
    bool create(task_node *task, void (*callback)(uint32_t), uint32_t period_ms)
    {
        if(!task || !callback || task->registered)
        {
            return false;
        }

        task->last_time = get_ms_tick();
        task->period_ms = period_ms;
        task->callback_tick = callback;

        append(task);

        return true;
    }

    /**
     * @brief 创建一个定时任务
     * 
     * @param task 任务节点指针
     * @param callback 回调函数指针
     * @param period_ms 定时周期（毫秒）
     * @param arg 回调函数参数指针
     * 
     * @return true 创建成功
     * @return false 创建失败
     */
    bool create(task_node *task, void (*callback)(void *), uint32_t period_ms, void *arg)
    {
        if(!task || !callback || task->registered)
        {
            return false;
        }

        task->last_time = get_ms_tick();
        task->period_ms = period_ms;
        task->callback_arg = callback;
        task->arg = arg;

        append(task);

        return true;
    }

    /**
     * @brief 循环查询任务列表是否到达定时时间
     */
    void loop()
    {
        task_node *current = head;

        while(current)
        {
            uint32_t now = get_ms_tick();

            if(current->period_ms == 0)
            {
                if(current->callback){current->callback();}
                else if(current->callback_tick){current->callback_tick(current->period_ms);}
                else if(current->callback_arg){current->callback_arg(current->arg);}
            }
            else if(now - current->last_time >= current->period_ms)
            {
                current->last_time += current->period_ms;

                if(current->callback){current->callback();}
                else if(current->callback_tick){current->callback_tick(current->period_ms);}
                else if(current->callback_arg){current->callback_arg(current->arg);}
            }
            current = current->p_next;
        }
    }
}
