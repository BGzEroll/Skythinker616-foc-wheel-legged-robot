#include "encoder.h"

#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "sys_time.h"

namespace
{
    namespace i2c
    {
        I2C_HandleTypeDef &handle = ::hi2c1;

        enum class dma_state : uint8_t
        {
            idle,
            busy,
            done,
            error
        };

        volatile dma_state state = dma_state::idle;
        volatile uint32_t dma_complete_time_us = 0;

        /**
         * @brief 使用 DMA 读取 I2C 寄存器数据
         * 
         * @param dev_addr I2C 设备地址
         * @param reg_addr 寄存器地址
         * @param data 数据缓冲区指针
         * @param size 数据长度（字节）
         * 
         * @return true 成功发起 DMA 读取
         * @return false 发起失败（参数错误或 DMA 正在忙碌）
         */
        bool dma_read_bytes(uint16_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t size)
        {
            if(!data || size == 0){return false;}
            if(state != dma_state::idle){return false;}

            state = dma_state::busy;
            if(HAL_I2C_Mem_Read_DMA(&handle, dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size) != HAL_OK)
            {
                state = dma_state::idle;
                return false;
            }

            return true;
        }

        /**
         * @brief 获取 DMA 状态
         * 
         * @return dma_state 当前 DMA 状态
         */
        dma_state get_dma_state()
        {
            return state;
        }

        /**
         * @brief 清除 DMA 状态
         */
        void clear_dma_state()
        {
            state = dma_state::idle;
        }

        /**
         * @brief DMA 读取完成
         */
        void dma_complete()
        {
            dma_complete_time_us = sys_time::get_us_tick();
            state = dma_state::done;
        }

        /**
         * @brief DMA 读取失败
         */
        void dma_fail()
        {
            state = dma_state::error;
        }
    }
}

static encoder_package latest_package = {};

namespace as5600
{
    namespace
    {
        constexpr uint16_t address = 0x36;
        constexpr uint8_t reg_raw_angle = 0x0C;

        constexpr int32_t resolution = 4096;
        constexpr int32_t half_resolution = resolution / 2;
        // 2π / 4096，Q8.24，供速度计算保留更高的角度精度。
        constexpr int32_t count_to_rad_q24 = 25736;
        constexpr uint32_t speed_filter_tf_us = 3000;

        uint8_t raw_data[2];

        bool initialized = false;
        bool package_valid = false;
        bool first_sample = true;

        uint16_t last_raw = 0;
        uint16_t last_time_us = 0;
        fixed::q16_t speed_q16 = 0;
        int32_t full_count = 0;
        uint32_t sequence = 0;
        
        /**
         * @brief 发起一次 DMA 读取
         * 
         * @return true 成功发起 DMA 读取
         * @return false 发起失败（参数错误或 DMA 正在忙碌）
         */
        bool start_read()
        {
            return i2c::dma_read_bytes(
                address,
                reg_raw_angle,
                raw_data,
                sizeof(raw_data)
            );
        }

        /**
         * @brief 处理读取到的数据
         */
        void process_data()
        {
            const uint16_t raw = (((uint16_t)raw_data[0] & 0x0F) << 8) | raw_data[1];
            const uint16_t now_us = (uint16_t)i2c::dma_complete_time_us;

            if(first_sample)
            {
                first_sample = false;
                last_raw = raw;
                last_time_us = now_us;
                full_count = raw;
            }
            else
            {
                int32_t delta = (int32_t)raw - (int32_t)last_raw;

                if(delta > half_resolution){delta -= resolution;}
                else if(delta < -half_resolution){delta += resolution;}

                full_count += delta;

                const uint16_t dt_us = (uint16_t)(now_us - last_time_us);
                if(dt_us != 0)
                {
                    const int64_t raw_speed_q16 =
                        static_cast<int64_t>(delta) *
                        static_cast<int64_t>(count_to_rad_q24) *
                        1000000LL /
                        (static_cast<int64_t>(dt_us) * 256LL);

                    const fixed::q16_t raw_speed =
                        fixed::saturate_q16(raw_speed_q16);

                    const uint32_t alpha_q16 =
                        (static_cast<uint32_t>(dt_us) << 16) /
                        (speed_filter_tf_us + dt_us);

                    const int64_t speed_delta =
                        static_cast<int64_t>(raw_speed) -
                        static_cast<int64_t>(speed_q16);

                    speed_q16 = fixed::saturate_q16(
                        static_cast<int64_t>(speed_q16) +
                        (speed_delta * alpha_q16 >> 16)
                    );
                }
                
                last_raw = raw;
                last_time_us = now_us;
            }

            encoder_package new_package;
            new_package.timestamp_us = (uint16_t)i2c::dma_complete_time_us;
            new_package.timestamp_ms = HAL_GetTick();
            new_package.angle_phase = static_cast<uint16_t>(raw << 4);
            new_package.full_count = full_count;
            new_package.speed_q16 = speed_q16;
            new_package.sequence = ++sequence;

            // 使用临界区保护数据包更新，防止中断导致数据不一致
            const uint32_t primask = __get_PRIMASK();
            __disable_irq();
            latest_package = new_package;
            package_valid = true;
            __set_PRIMASK(primask);
        }
    }

    /**
     * @brief 初始化 AS5600 编码器
     * 
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool init()
    {
        // 确保微秒计时器在 DMA 中断发生前已经运行
        sys_time::get_us_tick();

        // 预先发起第一次 DMA
        initialized = start_read();

        return initialized;
    }

    /**
     * @brief 更新编码器数据包
     * 
     * @return true 数据包已更新（DMA 读取完成）
     * @return false 数据包未更新（DMA 正在忙碌或出错）
     */
    bool update()
    {
        if(!initialized)
        {
            return false;
        }

        switch(i2c::get_dma_state())
        {
            case i2c::dma_state::busy:
                return false;

            case i2c::dma_state::done:
                process_data();
                i2c::clear_dma_state();
                start_read();       // 立即开始下一次读取
                return true;

            case i2c::dma_state::error:
                i2c::clear_dma_state();
                start_read();       // 出错后下一轮重新尝试
                return false;

            case i2c::dma_state::idle:
                start_read();
                return false;
        }

        return false;
    }

    /**
     * @brief 获取最新的编码器数据包
     * 
     * @param package 编码器数据包引用
     * 
     * @return true 成功获取数据包
     * @return false 未能获取数据包（编码器未初始化）
     */
    bool get_package(encoder_package &snapshot)
    {
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        const bool valid = initialized && package_valid;
        if(valid)
        {
            snapshot = latest_package;
        }

        __set_PRIMASK(primask);
        return valid;
    }
}

/**
 * @brief i2c dma 接收完成回调
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c == &i2c::handle)
    {
        i2c::dma_complete();
    }
}

/**
 * @brief i2c dma 接收错误回调
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c == &i2c::handle)
    {
        i2c::dma_fail();
    }
}
