#include "mpu6050.h"

#define DEG2RAD                     (3.1415926f / 180.0f)
#define SQUARE(X)                   ((X) * (X))

#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_CONFIG              0x1A
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_PWR_MGMT_1          0x6B

/**
 * @brief 构造函数
 * 
 * @param i2c_bus_num I2C 总线编号
 * @param addr MPU6050 地址
 * @param acc_coef 加速度系数
 */
mpu6050::mpu6050(uint8_t i2c_bus_num, uint8_t addr, float acc_coef)
    : i2c_bus_num(i2c_bus_num),
      addr(addr),
      acc_coef(acc_coef),
      i2c(i2c_bus_num)
{
    memset(angle, 0, sizeof(angle));
    memset(gyro_angle, 0, sizeof(gyro_angle));
    prev_Ts = 0;
}

/**
 * @brief 初始化 MPU6050
 * 
 * @param cail 是否校准角速度
 */
void mpu6050::init(uint8_t cail)
{
    i2c.init();

    write_cfg(MPU6050_SMPLRT_DIV, 0x00);
    write_cfg(MPU6050_CONFIG, 0x00);
    write_cfg(MPU6050_GYRO_CONFIG, 0x08);
    write_cfg(MPU6050_ACCEL_CONFIG, 0x00);
    write_cfg(MPU6050_PWR_MGMT_1, 0x01);

    if(cail){get_gyro_offset();}
}

/**
 * @brief 更新 MPU6050 数据
 */
void mpu6050::update()
{
    get_raw();
    process_data();
}

/**
 * @brief 读取 MPU6050 原始数据
 */
void mpu6050::get_raw()
{
    i2c.read_bytes(addr, 0x3B, raw, 14);
}

/**
 * @brief 处理 MPU6050 原始数据
 */
void mpu6050::process_data()
{
    // 温度
    temperature = (float)((int16_t)(raw[6] << 8 | raw[7]) + 12412.0f) / 340.0f;

    // 加速度、角速度
    float tmp[6];
    for(uint8_t i = 0; i < 3; i++)
    {
        tmp[i] = (float)((int16_t)((raw[0 + 2 * i] << 8) | raw[1 + 2 * i])) / 16384.0f;
        tmp[3 + i] = (float)((int16_t)((raw[8 + 2 * i] << 8) | raw[9 + 2 * i])) / 65.5f * DEG2RAD - gyro_offset[i];
    }

    // 轴向调整为 xyz 东北天
    acc[0] = tmp[1];
    acc[1] = -tmp[0];
    acc[2] = tmp[2];
    gyro[0] = tmp[4];
    gyro[1] = -tmp[3];
    gyro[2] = tmp[5];

    uint32_t now = (uint32_t)esp_timer_get_time();
    if(prev_Ts)
    {
        float dt = (float)(now - prev_Ts) * 1.0e-6f;
        if(dt > 1.0e-6f)
        {
            // 加速度计算角度，rad
            float acc_angle[2];
            acc_angle[0] = atan2f(acc[1], acc[2]);
            acc_angle[1] = atan2f(-acc[0], sqrtf(SQUARE(acc[1]) + SQUARE(acc[2])));

            // 角速度积分，rad
            for(uint8_t i = 0; i < 3; i++)
            {
                gyro_angle[i] += gyro[i] * dt;
            }

            // 限制 z 轴积分范围
            if(gyro_angle[2] > (float)M_PI){gyro_angle[2] -= 2.0f * (float)M_PI;}
            else if(gyro_angle[2] < -(float)M_PI){gyro_angle[2] += 2.0f * (float)M_PI;}

            // 融合角度
            angle[0] = (1.0f - acc_coef) * (angle[0] + gyro[0] * dt) + acc_coef * acc_angle[0];
            angle[1] = (1.0f - acc_coef) * (angle[1] + gyro[1] * dt) + acc_coef * acc_angle[1];
            angle[2] = gyro_angle[2];
        }
    }

    prev_Ts = now;
}

/**
 * @brief 写入 MPU6050 配置寄存器
 * 
 * @param reg 寄存器地址
 * @param val 寄存器值
 */
void mpu6050::write_cfg(uint8_t reg, uint8_t val)
{
    i2c.write_bytes(addr, reg, &val, 1);
}

/**
 * @brief 获取 MPU6050 角速度偏移
 */
void mpu6050::get_gyro_offset()
{
    // 预读 1000 次
    uint8_t tmp[6];
    for(uint32_t i = 0; i < 1000; i++)
    {
        i2c.read_bytes(addr, 0x43, tmp, 6);
    }

    int32_t sum[3] = {0};
    int16_t raw[3] = {0};
    for(uint32_t i = 0; i < 3000; i++)
    {
        i2c.read_bytes(addr, 0x43, tmp, 6);
        for(uint8_t j = 0; j < 3; j++)
        {
            raw[j] = (int16_t)((tmp[0 + 2 * j] << 8) | tmp[1 + 2 * j]);
            sum[j] += raw[j];
        }
    }

    for(uint8_t i = 0; i < 3; i++)
    {
        gyro_offset[i] = (float)sum[i] / 65.5f * DEG2RAD / 3000.0f;
    }
}
