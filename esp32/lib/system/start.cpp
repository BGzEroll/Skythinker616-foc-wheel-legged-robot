#include "start.h"
#include "task.h"
#include "led_dev.h"
#include "bus/uart_bus.h"
#include "bus/can_bus.h"

static uart_bus uart0(0);
static can_bus can0(0);

namespace
{
    // 与 STM32 main 分支 can_comm 的扩展 ID 布局保持一致。
    constexpr uint32_t can_discover_id = 0x700;
    constexpr uint32_t can_feedback_base = 0x04000000;
    constexpr uint32_t can_response_base = 0x08000000;
    constexpr uint32_t can_id_type_mask = 0x1C000000;
    constexpr uint32_t can_device_id_mask = 0x03FFFFFF;

    constexpr uint32_t can_discovery_period_ms = 500;
    constexpr float foc_target_value = 0.01f;

    // STM32 的 device_id 由 UID 计算，运行时从反馈/发现响应中学习。
    volatile uint32_t foc_target_device_id = 0;

    static void learn_device_id(uint32_t id)
    {
        const uint32_t type = id & can_id_type_mask;
        if(type != can_feedback_base && type != can_response_base)
        {
            return;
        }

        const uint32_t device_id = id & can_device_id_mask;
        if(device_id != 0)
        {
            foc_target_device_id = device_id;
        }
    }
}

static void process_can_msg(uint32_t id, const uint8_t *data, uint8_t len)
{
    constexpr uint8_t uart_can_sof_0 = 0xA5;
    constexpr uint8_t uart_can_sof_1 = 0x5A;
    constexpr uint8_t max_data_len = 8;

    if(len > max_data_len || (!data && len > 0))
    {
        return;
    }

    learn_device_id(id);

    // UART frame: A5 5A | CAN ID (little-endian) | DLC | data | XOR checksum.
    uint8_t frame[2 + sizeof(id) + 1 + max_data_len + 1];
    uint8_t frame_len = 0;
    frame[frame_len++] = uart_can_sof_0;
    frame[frame_len++] = uart_can_sof_1;
    memcpy(&frame[frame_len], &id, sizeof(id));
    frame_len += sizeof(id);
    frame[frame_len++] = len;

    uint8_t checksum = 0;
    for(uint8_t i = 2; i < frame_len; i++)
    {
        checksum ^= frame[i];
    }

    memcpy(&frame[frame_len], data, len);
    for(uint8_t i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }
    frame_len += len;
    frame[frame_len++] = checksum;

    uart0.write_bytes(frame, frame_len);
}

/**
 * @brief 自动发现 STM32，并持续发送 0.5 目标电压
 *
 * STM32 main 分支的 CAN 协议：
 * - 标准帧 0x700、DLC=0：设备发现
 * - 扩展帧 device_id、DLC=4：little-endian float 目标值
 *
 * STM32 FOC 在 100 ms 内没有新指令会关闭输出，因此这里 20 ms 发送一次。
 */
static void can_target_proc(void)
{
    constexpr uint32_t send_period_ms = 20;
    static uint32_t discovery_elapsed_ms = can_discovery_period_ms;

    const uint32_t device_id = foc_target_device_id;
    if(device_id == 0)
    {
        // 第一次立即发起发现，之后每 500 ms 重试一次。
        if(discovery_elapsed_ms >= can_discovery_period_ms)
        {
            uint8_t dummy = 0;
            can0.send(can_discover_id, &dummy, 0, false);
            discovery_elapsed_ms = 0;
        }
        else
        {
            discovery_elapsed_ms += send_period_ms;
        }
        return;
    }

    discovery_elapsed_ms = 0;

    uint8_t data[sizeof(foc_target_value)];
    memcpy(data, &foc_target_value, sizeof(data));
    can0.send(device_id, data, sizeof(data), true);
}

static void can_recv_proc(void)
{
    can0.receive(process_can_msg);
}

/**
 * @brief 任务列表
 */
static void task_list(void)
{
    static task can_recv_task(1, can_recv_proc, 4096, 4, 1);
    can_recv_task.start();

    static task can_target_task(20, can_target_proc, 2048, 3, 1);
    can_target_task.start();

    static task board_led_dev_task(50, board_led_dev_proc, 2048, 2, 0);
    board_led_dev_task.start();
}

/**
 * @brief 初始化 CAN 到 UART 透传
 */
void start_init_all(void)
{
    uart0.init();
    can0.init();
    board_led.init();
    task_list();
}
