#include "task.h"
#include "devices/hw/led.h"
#include "devices/hw/encoder.h"
#include "devices/hw/motor.h"
#include "devices/io/can_comm.h"

/**
 * @brief 任务列表
 */
void task_list()
{
    static task_node led_task;
    task::create(&led_task, led::leds_proc, 50);

    static task_node encoder_task;
    task::create(
        &encoder_task,
        [](){as5600::update();},
        0);

    static task_node can_comm_task;
    task::create(
        &can_comm_task,
        []()
        {
            static encoder_package package;
            as5600::get_package(package);
            can_comm::send_feedback(package);
        },
        2);
}

/**
 * @brief 应用程序初始化函数
 */
extern "C" void app_init(void)
{
    bool error = false;

    error |= !as5600::init();
    error |= !can_comm::init();
    error |= !motor::init();

    if(error)
    {
        led::set_error(true);
    }

    task_list();
}

/**
 * @brief 应用程序循环函数
 */
extern "C" void app_loop(void)
{
    task::loop();
}
