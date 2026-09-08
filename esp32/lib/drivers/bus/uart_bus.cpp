#include "uart_bus.h"

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;

class uart_dev {
    public:
        uart_dev(uint8_t uart_num, uint32_t baudrate)
            : uart_num(uart_num),
              baudrate(baudrate)
        {
            switch(uart_num)
            {
                case 0: serial = &Serial; break;
                case 1: serial = &Serial1; break;
                case 2: serial = &Serial2; break;
                default: serial = nullptr; break;
            }
        }

        void init()
        {
            if(is_init || !serial){return;}
            is_init = true;
            serial->begin(baudrate);
        }

        uint32_t read_bytes(uint8_t *buf, uint32_t max_len)
        {
            if(!serial || !buf || !max_len){return 0;}

            uint32_t len = 0;
            while(serial->available() && len < max_len)
            {
                buf[len++] = serial->read();
            }

            return len;
        }

        void write_bytes(const uint8_t *buf, uint32_t len)
        {
            if(!serial || (!buf && len > 0)){return;}

            serial->write(buf, len);
            serial->flush();
        }

        HardwareSerial *get_HardwareSerial_handle(){return serial;}

    private:
        uint8_t uart_num;
        uint32_t baudrate;
        HardwareSerial *serial = nullptr;
        bool is_init = false;
};

// 静态 uart 设备表（资源池）
static uart_dev uart_devs[] = {
    uart_dev(0, 921600),
    uart_dev(1, 115200),
    uart_dev(2, 1000000)
};
static constexpr uint8_t UART_DEV_NUM = sizeof(uart_devs) / sizeof(uart_devs[0]);

/**
 * @brief 根据 bus_id 获取对应底层设备
 *
 * @note 超出范围时默认返回 bus0；
 * @note 保证始终返回有效指针；
 */
static uart_dev *get_dev(uint8_t bus_id)
{
    if(bus_id < UART_DEV_NUM)
    {
        return &uart_devs[bus_id];
    }
    return &uart_devs[0];
}

/**
 * @brief uart_bus 构造函数
 *
 * @param bus_id UART 总线编号
 */
uart_bus::uart_bus(uint8_t bus_id)
    : bus_id(bus_id){}

/**
 * @brief 初始化 uart 总线
 */
void uart_bus::init()
{
    get_dev(bus_id)->init();
}

/**
 * @brief 连续读取字节
 *
 * @param buf 存放读取数据的缓冲区
 * @param max_len 缓冲区最大长度
 * 
 * @return 实际读取的字节数
 */
uint32_t uart_bus::read_bytes(uint8_t *buf, uint32_t max_len)
{
    return get_dev(bus_id)->read_bytes(buf, max_len);
}

/**
 * @brief 连续写入字节
 *
 * @param buf 待写入的数据缓冲区
 * @param len 写入数据的长度
 */
void uart_bus::write_bytes(const uint8_t *buf, uint32_t len)
{
    get_dev(bus_id)->write_bytes(buf, len);
}

/**
 * @brief 获取底层 HardwareSerial 句柄
 *
 * @return HardwareSerial* 返回底层硬件串口的句柄，供其他库使用
 */
HardwareSerial *uart_bus::get_HardwareSerial_handle() const
{
    return get_dev(bus_id)->get_HardwareSerial_handle();
}
