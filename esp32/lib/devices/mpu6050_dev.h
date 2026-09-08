#ifndef MPU6050_DEV_H
#define MPU6050_DEV_H

#include "mpu6050.h"
#include "bus/i2c_bus.h"

extern mpu6050 mpu6050_dev;

void mpu6050_dev_proc();

#endif
