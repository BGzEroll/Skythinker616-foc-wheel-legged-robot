#include "can_bus.h"

#include "stm32f1xx_hal.h"
#include "can.h"

class can_dev {
    public:
        can_dev(CAN_HandleTypeDef *can_handle)
            : can_handle(can_handle){}

        void init()
        {
            if(is_init || !can_handle){return;}

            CAN_FilterTypeDef filter;

            filter.FilterBank = 0;      // 筛选器组编号
            filter.FilterMode = CAN_FILTERMODE_IDMASK;      // 筛选器模式
            filter.FilterScale = CAN_FILTERSCALE_32BIT;      // 32位筛选器

            /* 不根据 ID 过滤，接收所有 ID */
            filter.FilterIdHigh = 0x0000;       // 高 16 位ID
            filter.FilterIdLow = 0x0000;        // 低 16 位ID
            filter.FilterMaskIdHigh = 0x0000;       // 高 16 位掩码
            filter.FilterMaskIdLow = 0x0000;        // 低 16 位掩码

            filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;     // FIFO0
            filter.SlaveStartFilterBank = 0;        // 从筛选器组 0 开始（只有一个 can 设备，该参数无意义）
            filter.FilterActivation = CAN_FILTER_ENABLE;        // 筛选器使能

            HAL_CAN_ConfigFilter(can_handle, &filter);
            __HAL_CAN_ENABLE_IT(can_handle, CAN_IT_RX_FIFO0_MSG_PENDING);
            HAL_CAN_Start(can_handle);
            is_init = true;
        }

        void register_receive_callback(void (*receive_cb)(uint32_t id, uint8_t *data))
        {
            this->receive_cb = receive_cb;
        }

        void send(uint32_t id, const uint8_t *data, uint8_t len)
        {
            CAN_TxHeaderTypeDef header = {
                .StdId = id,
                .IDE = CAN_ID_STD,      // 标准帧
                .RTR = CAN_RTR_DATA,        // 数据帧
                .DLC = len,     // 数据长度
                .TransmitGlobalTime = DISABLE       // 不使用时间戳
            };

            uint32_t tx_mailbox;
            HAL_CAN_AddTxMessage(can_handle, &header, data, &tx_mailbox);
        }

    private:
        CAN_HandleTypeDef *can_handle;
        void (*receive_cb)(uint32_t id, uint8_t *data);
        bool is_init = false;

    private:
        friend void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
};

// 静态 can 设备表（资源池）
static can_dev can_devs[] =
{
    can_dev(&hcan)
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
 * @brief can 总线注册接收回调函数
 * 
 * @param receive_cb 接收回调函数指针，参数为 can 消息 ID 和数据指针
 */
void can_bus::register_receive_callback(void (*receive_cb)(uint32_t id, uint8_t *data))
{
    get_dev(bus_id)->register_receive_callback(receive_cb);
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

/**
 * @brief can 总线接收中断回调函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can_dev *p = nullptr;
    if(hcan->Instance == CAN1){p = get_dev(0);}
    if(!p || !p->is_init || !p->receive_cb){return;}

    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

    if(HAL_CAN_GetRxMessage(p->can_handle, CAN_RX_FIFO0, &header, data) == HAL_OK)
    {
        p->receive_cb(header.StdId, data);
    }
}
