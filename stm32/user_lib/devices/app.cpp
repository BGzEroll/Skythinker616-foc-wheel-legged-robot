#include "task.h"
#include "devices/hw/encoder.h"
#include "devices/hw/motor.h"
#include "devices/io/can_comm.h"
#include "test.h"

/**
 * @brief 任务列表
 */
void task_list()
{
    static task_node led_task;
    task::create(&led_task, normal_led_blink, 50);

    static task_node encoder_task;
    task::create(
        &encoder_task,
        [](){as5600::update();},
        0);

    static task_node can_comm_test_task;
    task::create(&can_comm_test_task, can_comm_test, 2);
}

/**
 * @brief 应用程序初始化函数
 */
extern "C" void app_init(void)
{
    test_init();
    as5600::init();
    motor::init();
    can_comm::init();

    task_list();
}

/**
 * @brief 应用程序循环函数
 */
extern "C" void app_loop(void)
{
    task::loop();
}
