/* 机器人物理参数：腿长（cm）、舵机引脚、零位偏移（度） */
#pragma once

namespace robot {
    // 腿长（cm）。默认站立时小腿竖直并与最小圆（内圆）相切，脚落在髋节下方，
    // 故需满足 大腿 ≤ 小腿：LENA = 3.0，LENB = 4.0。
    constexpr double LENA = 3.0;   // 大腿长
    constexpr double LENB = 4.0;   // 小腿长

    // 6 条腿 × 3 个舵机（髋、大腿、小腿）的引脚
    constexpr int SERVO_PINS[6][3] = {
        {2, 3, 4}, {5, 6, 7}, {8, 9, 10},
        {11, 12, 13}, {14, 15, 16}, {17, 18, 19}
    };
    // 各舵机零位偏移（度），ServoManager 写入时叠加：最终角度 = 计算角度 + 偏移
    constexpr float ANGLE_OFFSET[6][3] = {
        {-90, 0, 0}, {90, 0, 0}, {-90, 0, 0},
        {90, 0, 0}, {-90, 0, 0}, {90, 0, 0}
    };
}
