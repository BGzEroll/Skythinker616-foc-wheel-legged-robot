#ifndef TASK_H
#define TASK_H

#include "stm32f1xx_hal.h"

#define get_ms_tick       HAL_GetTick		// 不同平台修改成对应的获取毫秒的函数

struct task_node
{
    uint32_t last_time = 0;
    uint32_t loop_time = 0;

    void (*callback)(uint32_t, void *) = nullptr;
    void *arg = nullptr;

    task_node *p_next = nullptr;
};

namespace task
{
    bool create(
        task_node *new_task,
        void (*callback)(uint32_t, void *),
        uint32_t loop_time,
        void *arg = nullptr);

    void loop();
}

#endif
