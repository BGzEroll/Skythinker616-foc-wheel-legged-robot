#include "task.h"
#include "test.h"

extern "C" void app_init(void)
{
    static task_node test_task;
    task::create(&test_task, normal_led_blink, 50, nullptr);
}

extern "C" void app_loop(void)
{
    task::loop();
}
