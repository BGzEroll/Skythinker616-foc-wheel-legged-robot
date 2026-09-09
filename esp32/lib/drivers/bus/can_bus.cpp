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

            twai_filter_config_t twai_filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

            if(twai_driver_install(&twai_config, &twai_timing_config, &twai_filter_config) != ESP_OK)
            {
                return;
            }

            is_init = twai_start() == ESP_OK;
        }

        void receive(void (*receive_cb)(uint32_t id, const uint8_t *data, uint8_t len))
        {
            if(!is_init || !receive_cb)
            {
                return;
            }

            twai_message_t msg = {};
            while(twai_receive(&msg, 0) == ESP_OK)
            {
                // Remote frames do not contain a payload. Forward valid CAN
                // data frames with their ID and actual payload length.
                if(!msg.rtr && msg.data_length_code <= TWAI_FRAME_MAX_DLC)
                {
                    receive_cb(msg.identifier, msg.data, msg.data_length_code);
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
 * @param receive_cb 接收回调函数指针，参数为 CAN ID、数据指针和实际长度
 */
void can_bus::receive(void (*receive_cb)(uint32_t id, const uint8_t *data, uint8_t len))
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
