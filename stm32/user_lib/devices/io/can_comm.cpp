#include "can_comm.h"

static can_bus can0(0);

/**
 * @brief can0 接收数据回调函数
 * 
 * @param id 数据 ID
 * @param data 数据指针
 */
static void can0_receive_cb(uint32_t id, uint8_t *data)
{
    if(id == 0x123)
    {
        ;
    }
}

/**
 * @brief can0 发送数据
 */
static void can0_send()
{
    // uint8_t tx_buf[8];
    // uint8_t idx = 0;

    // float angle = motor_1.sensor->get_angle();
    // float velocity = motor_1.sensor->get_velocity();

    // memcpy(tx_buf, &angle, sizeof(angle));
    // idx += sizeof(angle);
    // memcpy(&tx_buf[idx], &velocity, sizeof(velocity));
    // idx += sizeof(velocity);
    // can0.send(0x100, tx_buf, idx);
}

/**
 * @brief can 初始化
 */
void can_comm_init()
{
    can0.init();
    can0.register_receive_callback(can0_receive_cb);
}

/**
 * @brief can 处理数据进程函数
 */
void can_comm_proc()
{
    can0_send();
}
