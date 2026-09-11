#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <stdint.h>

namespace fixed
{
    // Q15 用于归一化电压、正弦和占空比；Q16.16 用于物理量。
    using q15_t = int32_t;
    using q16_t = int32_t;

    constexpr q15_t q15_one = 1 << 15;
    constexpr q16_t q16_one = 1 << 16;
    constexpr q16_t q16_max = 0x7FFFFFFF;
    constexpr q16_t q16_min = -q16_max - 1;

    inline q16_t saturate_q16(int64_t value)
    {
        if(value > q16_max){return q16_max;}
        if(value < q16_min){return q16_min;}
        return static_cast<q16_t>(value);
    }

    inline q16_t add_q16(q16_t left, q16_t right)
    {
        return saturate_q16(
            static_cast<int64_t>(left) + static_cast<int64_t>(right)
        );
    }

    inline q16_t mul_q16(q16_t left, q16_t right)
    {
        return saturate_q16(
            (static_cast<int64_t>(left) * static_cast<int64_t>(right)) >> 16
        );
    }

    inline q15_t mul_q15(q15_t left, q15_t right)
    {
        return static_cast<q15_t>(
            (static_cast<int64_t>(left) * static_cast<int64_t>(right)) >> 15
        );
    }
}

#endif
