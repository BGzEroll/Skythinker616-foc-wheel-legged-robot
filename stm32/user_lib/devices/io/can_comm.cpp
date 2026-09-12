#include "can_comm.h"

#include "stm32f1xx_hal.h"
#include "can.h"
#include <string.h>
#include "devices/sys_time.h"

namespace can_comm
{
    namespace
    {
        CAN_HandleTypeDef &handle = ::hcan;

        /**
         * @brief ESP32 广播设备发现
         *
         * Standard CAN
         * ID  = 0x700
         * DLC = 0
         */
        constexpr uint16_t discover_id = 0x700;
        
        /**
         * @brief Extended CAN ID 布局
         *
         * bit 28~26 : 消息类型
         * bit 25~0  : device_id
         *
         * 000 : command
         * 001 : feedback
         * 010 : discovery response
         */
        constexpr uint32_t device_id_mask = 0x03FFFFFF;
        constexpr uint32_t command_base  = 0x00000000;
        constexpr uint32_t feedback_base = 0x04000000;
        constexpr uint32_t response_base = 0x08000000;

        // STM32F103 96-bit Unique Device ID 起始地址
        constexpr uintptr_t uid_address = 0x1FFFF7E8UL;

        uint32_t device_id = 0;

        uint32_t command_id = 0;
        uint32_t feedback_id = 0;
        uint32_t response_id = 0;

        volatile int32_t latest_target_mNm = 0;
        volatile uint32_t latest_target_time = 0;
    }

    namespace
    {
        /**
         * @brief 根据 STM32 96-bit UID 生成固定 device_id
         *
         * 使用 FNV-1a Hash，
         * 最终截取为 26 bit。
         */
        uint32_t make_device_id()
        {
            const volatile uint8_t *uid =
                reinterpret_cast<const volatile uint8_t *>(uid_address);

            uint32_t hash = 2166136261u;

            for(uint8_t i = 0; i < 12; ++i)
            {
                hash ^= uid[i];
                hash *= 16777619u;
            }

            uint32_t id = hash & device_id_mask;

            // 0 保留，不作为有效 device_id
            if(id == 0){id = 1;}

            return id;
        }

        /**
         * @brief 配置精确 CAN ID 过滤器
         *
         * @param bank Filter Bank
         * @param id CAN ID
         * @param extended true = Extended CAN
         *
         * @return true 配置成功
         * @return false 配置失败
         */
        bool config_exact_filter(uint8_t bank, uint32_t id, bool extended)
        {
            uint32_t filter_id;
            uint32_t filter_mask;

            if(extended)
            {
                /**
                 * bxCAN 32-bit Extended filter:
                 *
                 * bit31~3 = 29-bit Extended ID
                 * bit2    = IDE
                 * bit1    = RTR
                 */
                filter_id =
                    ((id & 0x1FFFFFFFu) << 3) |
                    (1u << 2);

                filter_mask =
                    (0x1FFFFFFFu << 3) |
                    (1u << 2) |
                    (1u << 1);
            }
            else
            {
                /**
                 * bxCAN 32-bit Standard filter:
                 *
                 * bit31~21 = 11-bit Standard ID
                 * bit2     = IDE
                 * bit1     = RTR
                 */
                filter_id =
                    (id & 0x7FFu) << 21;

                filter_mask =
                    (0x7FFu << 21) |
                    (1u << 2) |
                    (1u << 1);
            }


            CAN_FilterTypeDef filter = {};

            filter.FilterBank = bank;
            filter.FilterMode = CAN_FILTERMODE_IDMASK;
            filter.FilterScale = CAN_FILTERSCALE_32BIT;
            filter.FilterIdHigh = (uint16_t)(filter_id >> 16);
            filter.FilterIdLow = (uint16_t)(filter_id & 0xFFFF);
            filter.FilterMaskIdHigh = (uint16_t)(filter_mask >> 16);
            filter.FilterMaskIdLow = (uint16_t)(filter_mask & 0xFFFF);
            filter.FilterFIFOAssignment = CAN_RX_FIFO0;
            filter.FilterActivation = ENABLE;

            return HAL_CAN_ConfigFilter(&handle, &filter) == HAL_OK;
        }

        /**
         * @brief 发送 CAN 数据帧
         */
        bool send_frame(uint32_t id, bool extended, const uint8_t *data, uint8_t size)
        {
            if(size > 8)
            {
                return false;
            }

            CAN_TxHeaderTypeDef header = {};

            if(extended)
            {
                header.ExtId = id & 0x1FFFFFFFu;
                header.IDE = CAN_ID_EXT;
            }
            else
            {
                header.StdId = id & 0x7FFu;
                header.IDE = CAN_ID_STD;
            }

            header.RTR = CAN_RTR_DATA;
            header.DLC = size;
            header.TransmitGlobalTime = DISABLE;

            uint32_t mailbox;

            const uint32_t primask = __get_PRIMASK();
            __disable_irq();
            const HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&handle, &header, data, &mailbox);
            __set_PRIMASK(primask);

            return status == HAL_OK;
        }

        /**
         * @brief 回复设备发现
         *
         * Extended CAN ID:
         *
         * response_base | device_id
         *
         * CAN ID 本身已经包含设备身份，
         * 因此无需 payload。
         */
        bool send_device_response()
        {
            uint8_t dummy = 0;

            return send_frame(
                response_id,
                true,
                &dummy,
                0
            );
        }

        /**
         * @brief 处理收到的目标扭矩
         */
        void receive_target(const CAN_RxHeaderTypeDef &header, const uint8_t *data)
        {
            if(header.IDE != CAN_ID_EXT ||
               header.RTR != CAN_RTR_DATA ||
               header.ExtId != command_id ||
               header.DLC != sizeof(float))
            {
                return;
            }

            float torque;
            memcpy(&torque, data, sizeof(torque));

            latest_target_mNm = (int32_t)(torque * 1000.0f);
            latest_target_time = sys_time::get_ms_tick();
        }
        
        /**
         * @brief CAN FIFO0 接收处理
         */
        void receive()
        {
            CAN_RxHeaderTypeDef header;
            uint8_t data[8];

            if(HAL_CAN_GetRxMessage(&handle, CAN_RX_FIFO0, &header, data) != HAL_OK)
            {
                return;
            }

            // Device discovery
            if(header.IDE == CAN_ID_STD &&
               header.RTR == CAN_RTR_DATA &&
               header.StdId == discover_id &&
               header.DLC == 0)
            {
                send_device_response();
                return;
            }

            // Target voltage
            if(header.IDE == CAN_ID_EXT &&
               header.ExtId == command_id)
            {
                receive_target(header, data);
                return;
            }
        }
    }

    /**
     * @brief 初始化 CAN 通信
     *
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool init()
    {
        // STM 上电立即读取自己的 UID，生成固定 device_id
        device_id = make_device_id();
        command_id = command_base | device_id;
        feedback_id = feedback_base | device_id;
        response_id = response_base | device_id;

        /**
         * Filter Bank 0
         *
         * 广播设备发现：
         *
         * STD 0x700
         */
        if(!config_exact_filter(0, discover_id, false))
        {
            return false;
        }

        /**
         * Filter Bank 1
         *
         * 只接收发送给自己的控制帧：
         *
         * EXT command_id
         */
        if(!config_exact_filter(1,command_id, true))
        {
            return false;
        }

        if(HAL_CAN_Start(&handle) != HAL_OK)
        {
            return false;
        }

        if(HAL_CAN_ActivateNotification(&handle, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        {
            return false;
        }

        return true;
    }

    /**
     * @brief 获取最新的目标
     *
     * @return 最新的目标
     */
    int32_t get_target_mNm()
    {
        if(sys_time::get_ms_tick() - latest_target_time > 100)
        {
            return 0;
        }

        return latest_target_mNm;
    }

    /**
     * @brief 发送编码器数据反馈
     *
     * @param package 编码器数据包
     *
     * @return true 发送成功
     * @return false 发送失败
     */
    bool send_feedback(const encoder_package &package)
    {
        uint8_t data[8];
        static uint16_t sequence = 0;

        sequence++;

        memcpy(&data[0], &sequence, sizeof(uint16_t));
        memcpy(&data[2], &package.timestamp_us, sizeof(uint16_t));
        memcpy(&data[4], &package.full_count, sizeof(int32_t));

        return send_frame(
            feedback_id,
            true,
            data,
            sizeof(data)
        );
    }
}

/**
 * @brief CAN RX FIFO0 消息接收中断回调
 *
 * @param hcan CAN 句柄
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan == &can_comm::handle)
    {
        can_comm::receive();
    }
}
