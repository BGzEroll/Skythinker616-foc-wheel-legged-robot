#include "start.h"
#include "task.h"
#include "led_dev.h"
#include "bus/uart_bus.h"
#include "bus/can_bus.h"

static uart_bus uart0(0);
static can_bus can0(0);

static void process_can_msg(uint32_t id, const uint8_t *data, uint8_t len)
{
    constexpr uint8_t uart_can_sof_0 = 0xA5;
    constexpr uint8_t uart_can_sof_1 = 0x5A;
    constexpr uint8_t max_data_len = 8;

    if(len > max_data_len || (!data && len > 0))
    {
        return;
    }

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
