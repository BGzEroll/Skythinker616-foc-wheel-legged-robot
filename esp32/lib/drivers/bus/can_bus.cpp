#include "can_bus.h"

class can_dev {
    public:
        can_dev(gpio_num_t tx_io, gpio_num_t rx_io)
            : tx_io(tx_io),
              rx_io(rx_io){}

        void init()
        {
            twai_general_config_t twai_config = {
                .mode = TWAI_MODE_NORMAL,
                .tx_io = tx_io,
                .rx_io = rx_io,
                .clkout_io = TWAI_IO_UNUSED,
                .bus_off_io = TWAI_IO_UNUSED,
                .tx_queue_len = 5,
                .rx_queue_len = 10,
                .alerts_enabled = TWAI_ALERT_NONE,
                .clkout_divider = 0,
                .intr_flags = ESP_INTR_FLAG_LEVEL1
            };

            twai_timing_config_t twai_timing_config = TWAI_TIMING_CONFIG_1MBITS();

            twai_filter_config_t twai_filter_config = {
                .acceptance_code = 0x00000000,
                .acceptance_mask = 0xFFFFFFFF,
                .single_filter = true
            };

            twai_driver_install(&twai_config, &twai_timing_config, &twai_filter_config);
            twai_start();
            is_init = true;
        }

        void receive(void (*receive_cb)(uint32_t id, uint8_t *data))
        {
            twai_status_info_t status;
            twai_get_status_info(&status);

            twai_message_t msg;
            for(uint8_t i = 0; i < status.msgs_to_rx; i++)
            {
                if(twai_receive(&msg, 0) == ESP_OK)
                {
                    receive_cb(msg.identifier, msg.data);
                }
            }
        }

        void send(uint32_t id, const uint8_t *data, uint8_t len)
        {
            twai_message_t msg = {
                .flags = 0,
                .identifier = id,
                .data_length_code = len
            };

            memcpy(msg.data, data, len);
            twai_transmit(&msg, 0);
        }

    private:
        gpio_num_t tx_io;
        gpio_num_t rx_io;
        bool is_init = false;
};

// 静态 can 设备表（资源池）
static can_dev can_devs[] = {
    can_dev(GPIO_NUM_33, GPIO_NUM_32)
};
static constexpr uint8_t CAN_DEV_NUM = sizeof(can_devs) / sizeof(can_devs[0]);

/**
 * @brief 根据 bus_id 获取对应底层设备
 * 
 * @param bus_id can 总线编号
 * @return can_dev* can 设备指针
 */
static can_dev *get_dev(uint8_t bus_id)
{
    if(bus_id < CAN_DEV_NUM)
    {
        return &can_devs[bus_id];
    }
    return &can_devs[0];
}

/**
 * @brief can 总线构造函数
 * 
 * @param bus_id can 总线编号
 */
can_bus::can_bus(uint8_t bus_id)
    : bus_id(bus_id){}

/**
 * @brief can 总线初始化函数
 */
void can_bus::init()
{
    get_dev(bus_id)->init();
}

/**
 * @brief can 总线接收函数
 * 
 * @param receive_cb 接收回调函数指针，参数为 can 消息 ID 和数据指针
 */
void can_bus::receive(void (*receive_cb)(uint32_t id, uint8_t *data))
{
    get_dev(bus_id)->receive(receive_cb);
}

/**
 * @brief can 总线发送函数
 * 
 * @param id can 消息 ID
 * @param data can 消息数据指针
 * @param len can 消息数据长度
 * 
 * @note 数据长度必须小于等于 8 字节
 */
void can_bus::send(uint32_t id, const uint8_t *data, uint8_t len)
{
    get_dev(bus_id)->send(id, data, len);
}
