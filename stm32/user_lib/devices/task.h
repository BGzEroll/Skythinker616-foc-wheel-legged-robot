#ifndef TASK_H
#define TASK_H

#include <stdint.h>

struct task_node
{
    uint32_t last_time = 0;
    uint32_t period_ms = 0;

    void (*callback)(uint32_t, void *) = nullptr;
    void *arg = nullptr;

    task_node *p_next = nullptr;
    bool registered = false;
};

namespace task
{
    bool create(
        task_node *task,
        void (*callback)(uint32_t, void *),
        uint32_t period_ms,
        void *arg = nullptr);

    void loop();
}

#endif
