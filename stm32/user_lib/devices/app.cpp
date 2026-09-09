#include "task.h"
#include "test.h"

void task_list()
{
    static task_node test_task_led;
    task::create(&test_task_led, normal_led_blink, 50, nullptr);

    static task_node test_task_encoder;
    task::create(&test_task_encoder, encoder_test, 1, nullptr);

    static task_node test_task_can_comm;
    task::create(&test_task_can_comm, can_comm_test, 2, nullptr);
}

/**
 * @brief 应用程序初始化函数
 */
extern "C" void app_init(void)
{
    test_init();

    task_list();
}

/**
 * @brief 应用程序循环函数
 */
extern "C" void app_loop(void)
{
    task::loop();
}
