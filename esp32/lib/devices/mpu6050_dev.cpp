#include "mpu6050_dev.h"

mpu6050 mpu6050_dev(0, 0x68, 0.02f);

/**
 * @brief mpu6050_dev_proc 进程函数
 */
void mpu6050_dev_proc()
{
    mpu6050_dev.update();
}
