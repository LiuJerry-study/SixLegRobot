/* 机器人物理参数：腿长（cm）、PCA9685 通道、零位偏移（度）
 * 硬件：每条腿 2 个舵机（大腿 + 小腿），两自由度在同一竖直平面内（无髋节旋转）。
 *
 * 舵机驱动：改用 PCA9685（I2C 16 路 PWM 驱动板），12 个舵机接其 ch0~ch11。
 *   - PCA9685.SDA → ESP32 GPIO21
 *   - PCA9685.SCL → ESP32 GPIO22
 *   - PCA9685.VCC → 与 ESP32 I²C 电平匹配（带 VCC 上拉的模块使用 3.3V 或电平转换）
 *   - PCA9685 舵机电源 V+ → 独立 5V 电源（12 个舵机电流较大，务必外接电源，
 *     不能从 PCA9685 板载逻辑供电口取电，GND 与 ESP32 共地）
 */
#pragma once

#include "RobotModel.h"

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
