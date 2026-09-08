#include "i2c_bus.h"

static TwoWire wire_0 = TwoWire(0);
static TwoWire wire_1 = TwoWire(1);

class i2c_dev {
    public:
        i2c_dev(uint8_t i2c_num, uint8_t scl_pin, uint8_t sda_pin, uint32_t freq)
            : i2c_num(i2c_num),
              scl_pin(scl_pin),
              sda_pin(sda_pin),
              freq(freq)
        {
            switch(i2c_num)
            {
                case 0: wire = &wire_0; break;
                case 1: wire = &wire_1; break;
                default: wire = nullptr; break;
            }
        }

        void init()
        {
            if(is_init || !wire){return;}
            is_init = true;
            wire->begin(sda_pin, scl_pin, freq);
        }

        void read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
        {
            if(!wire || !buf || !len){return;}

            wire->beginTransmission(addr);
            wire->write(reg);
            wire->endTransmission(false);
            wire->requestFrom(addr, len);
            wire->readBytes(buf, len);
        }

        void write_bytes(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len)
        {
            if(!wire || (!buf && len > 0)){return;}

            wire->beginTransmission(addr);
            wire->write(reg);
            wire->write(buf, len);
            wire->endTransmission(true);
        }

        TwoWire *get_TwoWire_handle(){return wire;}

    private:
        uint8_t i2c_num;
        TwoWire *wire = nullptr;
        uint8_t scl_pin;
        uint8_t sda_pin;
        uint32_t freq;
        bool is_init = false;
};

// 静态 i2c 设备表（资源池）
static i2c_dev i2c_devs[] = {
    i2c_dev(0, 22, 21, 400000),
    i2c_dev(1, 5, 23, 400000),
};
static constexpr uint8_t I2C_DEV_NUM = sizeof(i2c_devs) / sizeof(i2c_devs[0]);

/**
 * @brief 根据 bus_id 获取对应底层设备
 *
 * @note 超出范围时默认返回 bus0；
 * @note 保证始终返回有效指针；
 */
static i2c_dev *get_dev(uint8_t bus_id)
{
    if(bus_id < I2C_DEV_NUM)
    {
        return &i2c_devs[bus_id];
    }
    return &i2c_devs[0];
}

/**
 * @brief i2c 总线构造函数
 * 
 * @param bus_id i2c 总线编号
 */
i2c_bus::i2c_bus(uint8_t bus_id)
    : bus_id(bus_id){}

/**
 * @brief 初始化 i2c 总线
 */
void i2c_bus::init()
{
    get_dev(bus_id)->init();
}

/**
 * @brief 连续读取寄存器数据
 * 
 * @param addr 设备地址
 * @param reg  起始寄存器地址
 * @param buf  接收缓冲区
 * @param len  读取长度
 */
void i2c_bus::read_bytes(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    get_dev(bus_id)->read_bytes(addr, reg, buf, len);
}

/**
 * @brief 连续写入寄存器数据
 * 
 * @param addr 设备地址
 * @param reg  起始寄存器地址
 * @param buf  数据缓冲区
 * @param len  写入长度
 */
void i2c_bus::write_bytes(uint8_t addr, uint8_t reg, const uint8_t *buf, uint8_t len)
{
    get_dev(bus_id)->write_bytes(addr, reg, buf, len);
}

/**
 * @brief 获取底层 TwoWire 句柄
 */
TwoWire *i2c_bus::get_TwoWire_handle() const
{
    return get_dev(bus_id)->get_TwoWire_handle();
}
