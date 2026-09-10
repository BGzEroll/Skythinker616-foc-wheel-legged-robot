#ifndef MOTOR_H
#define MOTOR_H

namespace motor
{
    bool init();

    // 不提供 move api，目标电压和电角度在 TIM2 更新中断中直接处理。
}

#endif
