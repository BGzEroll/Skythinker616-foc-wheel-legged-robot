#include "task.h"

/**
 * @brief rtos 任务构造函数
 * 
 * @param loop_time 任务周期（ms）
 * @param func 任务回调函数
 * @param stack_depth 任务栈大小（byte 单位）
 * @param priority 任务优先级
 * @param core_id 任务运行核心
 * 
 * @note 任务周期单位为毫秒
 * @note 带 tick 参数
 */
task::task(uint32_t loop_time,
        void (*func)(uint32_t),
        uint32_t stack_depth,
        UBaseType_t priority,
        BaseType_t core_id)
    : type(type_t::rtos),
      loop_time(loop_time),
      task_func(func),
      task_func_void(nullptr),
      stack_depth(stack_depth),
      priority(priority),
      core_id(core_id){}

/**
 * @brief rtos 任务构造函数
 * 
 * @param loop_time 任务周期（ms）
 * @param func 任务回调函数
 * @param stack_depth 任务栈大小（byte 单位）
 * @param priority 任务优先级
 * @param core_id 任务运行核心
 * 
 * @note 任务周期单位为毫秒
 * @note 不带 tick 参数
 */
task::task(uint32_t loop_time,
        void (*func_void)(),
        uint32_t stack_depth,
        UBaseType_t priority,
        BaseType_t core_id)
    : type(type_t::rtos),
      loop_time(loop_time),
      task_func(nullptr),
      task_func_void(func_void),
      stack_depth(stack_depth),
      priority(priority),
      core_id(core_id){}

/**
 * @brief esp_timer 任务构造函数
 * 
 * @param loop_time 任务周期（us）
 * @param func 任务回调函数
 * 
 * @note 任务周期单位为微秒
 * @note 带 tick 参数
 */
task::task(uint32_t loop_time,
        void (*func)(uint32_t))
    : type(type_t::esp_timer),
      loop_time(loop_time),
      task_func(func),
      task_func_void(nullptr){}

/**
 * @brief esp_timer 任务构造函数
 * 
 * @param loop_time 任务周期（us）
 * @param func 任务回调函数
 * 
 * @note 任务周期单位为微秒
 * @note 不带 tick 参数
 */
task::task(uint32_t loop_time,
        void (*func_void)())
    : type(type_t::esp_timer),
      loop_time(loop_time),
      task_func(nullptr),
      task_func_void(func_void){}

/**
 * @brief 启动任务
 * 
 * @note 任务启动后，会根据任务类型（rtos 或 esp_timer）创建或启动任务
 */
void task::start()
{
    switch(type)
    {
        case type_t::rtos:
            xTaskCreatePinnedToCore(
                rtos_entry,     // 任务入口
                "user_task",     // 任务名
                stack_depth,        // 栈大小（byte 单位）
                this,       // 参数
                priority,       // 优先级
                NULL,
                core_id     // 运行核心
            );
            break;

        case type_t::esp_timer:
            const esp_timer_create_args_t timer_args = {
                .callback = timer_callback,
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "user_task",
                .skip_unhandled_events = 1
            };

            esp_timer_create(&timer_args, &xTimer);

            // 启动周期定时器（单位：us）
            esp_timer_start_periodic(xTimer, loop_time);
            break;
    }
}

/**
 * @brief rtos 任务入口函数
 * 
 * @param arg 任务参数
 * 
 */
void task::rtos_entry(void *arg)
{
    task *t = (task *)arg;

    TickType_t last_wake_time = xTaskGetTickCount();
    while(1)
    {
        if(t->task_func)
        {
            t->task_func(t->loop_time);
        }
        else if(t->task_func_void)
        {
            t->task_func_void();
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(t->loop_time));
    }
}

/**
 * @brief esp_timer 任务回调函数
 * 
 * @param arg 任务参数
 */
void task::timer_callback(void *arg)
{
    task *t = (task *)arg;

    if(t->task_func)
	{
		t->task_func(t->loop_time);
    }
    else if(t->task_func_void)
    {
        t->task_func_void();
    }
}
