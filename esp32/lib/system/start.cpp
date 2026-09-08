#include "start.h"

#define DEBUG_TEST
#ifdef DEBUG_TEST

#include "bus/uart_bus.h"
#include "bus/can_bus.h"

uart_bus uart0(0);
can_bus can0(0);

static float rad_angle, rad_speed;
static void process_can_msg(uint32_t id, uint8_t *data)
{
    if(id == 0x456)
    {
        printf("can msg id: 0x%08X, ", id);
        for(uint8_t i = 0; i < 8; i++)
        {
            printf("0x%02X ", data[i]);
        }
        printf("\n");
    }

    if(id == 0x100)
    {
        memcpy(&rad_angle, data, sizeof(rad_angle));
        memcpy(&rad_speed, &data[4], sizeof(rad_speed));
    }

    // if(id == 0x102)
    // {
    //     memcpy(&debug_Uq, data, sizeof(debug_Uq));
    //     memcpy(&debug_Ud, &data[4], sizeof(debug_Ud));
    // }
}

static void can_recv_proc(void)
{
    can0.receive(process_can_msg);
}

static void debug_test_proc(uint32_t tick)
{
    // static uint32_t can_send_cnt = 0;
    // if((can_send_cnt += tick) >= 1000 && (can_send_cnt = 0, 1))
    // {
    //     uint8_t tx_buf[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38};
    //     can0.send(0x123, tx_buf, sizeof(tx_buf));
    // }

    static uint32_t printf_cnt = 0;
    if((printf_cnt += tick) >= 5 && (printf_cnt = 0, 1))
    {
        uint8_t tx_buf[256];
        uint32_t tx_len;

        // tx_len = sprintf((char *)tx_buf,
        //     "angle_x: %f\tangle_y: %f\tangle_z: %f\n",
        //     mpu6050_dev.angle[0] / (float)PI * 180.0f, mpu6050_dev.angle[1] / (float)PI * 180.0f, mpu6050_dev.angle[2] / (float)PI * 180.0f);

        // tx_len = sprintf((char *)tx_buf,
        //     "acc_x: %f\tacc_y: %f\tacc_z: %f\n",
        //     mpu6050_dev.acc[0], mpu6050_dev.acc[1], mpu6050_dev.acc[2]);

        tx_len = sprintf((char *)tx_buf,
            // "rad_angle: %f\trad_speed: %f\n",
            // rad_angle, rad_speed);
            // "%f,%f,%f,%f,%f,%f\n",
            // rad_angle, rad_speed, debug_e_angle, direction_corrected_velocity, debug_Uq, debug_Ud);
            "%f,%f\n",
            rad_angle, rad_speed);

        // printf("gyro_x: %f\tgyro_y: %f\tgyro_z: %f\n",
        //     mpu6050_dev.gyro[0], mpu6050_dev.gyro[1], mpu6050_dev.gyro[2]);

        // const auto &fb = ctrl.feedback();
        // printf("pitch_angle: %f\tpitch_rate: %f\tavg_linear_pos: %f\tavg_linear_vel: %f\tyaw_angle: %f\tyaw_rate: %f\n",
        //     fb.pitch_angle / (float)PI * 180.0f,
        //     fb.pitch_rate / (float)PI * 180.0f,
        //     fb.avg_linear_pos,
        //     fb.avg_linear_vel,
        //     fb.yaw_angle / (float)PI * 180.0f,
        //     fb.yaw_rate / (float)PI * 180.0f
        // );

        // printf("Buttons: ");
        // for(int8_t i = 15; i >= 0; i--)
        // {
        //     printf("%d", (gamepad.buttons >> i) & 0x1);
        // }
        // printf("\t");
        // printf("joysticks: LX:%.3f\tLY:%.3f\tRX:%.3f\tRY:%.3f\tLT:%.3f\t",
        //     gamepad.axes[0],
        //     gamepad.axes[1],
        //     gamepad.axes[2],
        //     gamepad.axes[3],
        //     gamepad.axes[4],
        //     gamepad.axes[5]
        // );
        // printf("triggers: LT:%.3f\tRT:%.3f\n",
        //     gamepad.axes[4],
        //     gamepad.axes[5]
        // );

        uart0.write_bytes(tx_buf, tx_len);
    }
}

#endif

/**
 * @brief 任务列表
 */
static void task_list(void)
{
#ifdef DEBUG_TEST
    static task debug_test_task(1, debug_test_proc, 4096, 3, 0);
    debug_test_task.start();

    static task can_recv_task(1, can_recv_proc, 4096, 4, 1);
    can_recv_task.start();
#endif

    static task board_led_dev_task(50, board_led_dev_proc, 2048, 2, 0);
    board_led_dev_task.start();

    // static task mpu6050_dev_task(5, mpu6050_dev_proc, 4096, 4, 0);
    // mpu6050_dev_task.start();
}

/**
 * @brief 初始化所有模块
 */
void start_init_all(void)
{
    delay(1000);

#ifdef DEBUG_TEST
    uart0.init();
    can0.init();
#endif

    board_led.init();
    // mpu6050_dev.init(1);

	task_list();
}



