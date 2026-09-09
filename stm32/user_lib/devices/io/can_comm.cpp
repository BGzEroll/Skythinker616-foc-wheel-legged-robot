#include "can_comm.h"

#include "devices/hw/encoder.h"

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
    encoder_package package;

    if(!as5600::get_package(package))
    {
        return;
    }

    uint8_t tx_buf[8];

    memcpy(&tx_buf[0], &package.full_angle, sizeof(float));
    memcpy(&tx_buf[4], &package.speed, sizeof(float));

    can0.send(0x100, tx_buf, sizeof(tx_buf));
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
