#include "encoder.h"

#include "i2c.h"

namespace i2c
{
    inline I2C_HandleTypeDef &handle = ::hi2c1;

    namespace
    {
        volatile bool dma_rx_busy = false;
    }

    bool dma_read_bytes(uint16_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t size)
    {
        if(!data || !size){return false;}
        if(HAL_I2C_GetState(&handle) != HAL_I2C_STATE_READY){return false;}
        if(dma_rx_busy){return false;}

        if(HAL_I2C_Mem_Read_DMA(&handle, dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size) == HAL_OK)
        {
            dma_rx_busy = true;
        }

        return false;
    }
}

namespace as5600
{
    namespace
    {
        constexpr float count_to_rad = 2.0f * 3.14159265358979323846f / 4096.0f;

        encoder_package package;
        uint8_t raw_data[2];

        bool i2c::dma_read_bytes(0x36, 0x0C, raw_data, sizeof(raw_data));
    }

    bool init()
    {
        return true;
    }

    encoder_package read(encoder_package &package)
    {
        ;
    }
}

/**
 * @brief i2c dma 接收完成回调
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c::dma_rx_busy = false;
}
