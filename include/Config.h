/* 机器人物理参数：腿长（cm）、PCA9685 通道、零位偏移（度）
 * 硬件：每条腿 2 个舵机（大腿 + 小腿），两自由度在同一竖直平面内（无髋节旋转）。
 *
 * 舵机驱动：改用 PCA9685（I2C 16 路 PWM 驱动板），12 个舵机接其 ch0~ch11。
 *   - PCA9685.SDA → ESP32 GPIO21
 *   - PCA9685.SCL → ESP32 GPIO22
 *   - PCA9685.VCC → 5V（板载逻辑供电）
 *   - PCA9685 舵机电源 V+ → 独立 5V 电源（12 个舵机电流较大，务必外接电源，
 *     不能从 PCA9685 板载逻辑供电口取电，GND 与 ESP32 共地）
 */
#pragma once

namespace robot {
    // 腿长（cm），实测硬件：大腿 8cm、小腿 13cm。
    // 默认站立时小腿竖直并与最小圆（内圆，半径 |LENA-LENB| = 5cm）负侧相切，
    // 脚落在膝正下方（髋外侧 5cm、下方 6.755cm）。
    constexpr double LENA = 6.5;   // 大腿长
    constexpr double LENB = 12.5;  // 小腿长

    // 6 条腿 × 2 个舵机（大腿、小腿）对应的 PCA9685 输出通道（ch0~ch11）：
    //   leg0=ch0/1, leg1=ch2/3, leg2=ch4/5, leg3=ch6/7, leg4=ch8/9, leg5=ch10/11
    // 按你实际的舵机接线调整这些通道编号即可，无需改上层代码。
    constexpr int SERVO_PINS[6][2] = {
        {0,  1 }, {2,  3 }, {4,  5 },
        {6,  7 }, {8,  9 }, {10, 11}
    };
    // 各舵机零位偏移（度），ServoManager 写入时叠加：最终角度 = 计算角度 + 偏移
    // 目前全部为 0，上电校准后按实际安装填值
    constexpr float ANGLE_OFFSET[6][2] = {
        {0, 0}, {0, 0}, {0, 0},
        {0, 0}, {0, 0}, {0, 0}
    };
}

// ================= PCA9685 板级常量（全局） =================
// I2C 引脚（ESP32 接 PCA9685）
constexpr int PCA9685_SDA_PIN = 21;
constexpr int PCA9685_SCL_PIN = 22;

// PCA9685 I2C 地址（默认 0x40，改跳线后这里也改）
constexpr int PCA9685_I2C_ADDR = 0x40;

// 舵机 PWM 参数（按你的舵机微调！）
// PCA9685 用 50Hz、4096 级，每级 ≈ 4.9us：
//   0.5ms (0°)   ≈ 102
//   1.5ms (90°)  ≈ 307
//   2.5ms (180°) ≈ 512
constexpr float  PCA9685_PWM_FREQ_HZ  = 50.0f;   // 舵机标准频率
constexpr int    PCA9685_OSC_FREQ_HZ  = 25000000;// PCA9685 内部振荡器频率（默认值）
constexpr int    SERVO_MIN_PULSE_CNT  = 102;     // 0°   对应 0.5ms
constexpr int    SERVO_MAX_PULSE_CNT  = 512;     // 180° 对应 2.5ms
