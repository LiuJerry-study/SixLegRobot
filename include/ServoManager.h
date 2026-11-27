/* 舵机硬件抽象：集中管理 6腿 × 2关节 = 12 个舵机（大腿 + 小腿，同一竖直平面）。
 *
 * 本实现改为通过 PCA9685（I2C 接口的 16 路 PWM 驱动板）驱动 12 个舵机，
 * 只需占用 ESP32 的 SDA/SCL 两个引脚即可控制全部舵机。
 *
 * 职责：初始化 PCA9685、写入时叠加零位偏移、统一限幅 0~180°。
 * 通道与偏移分别取自 Config.h 的 SERVO_PINS / ANGLE_OFFSET
 * （此处 SERVO_PINS 表示 PCA9685 的输出通道号 ch0~ch11）。
 * 单例风格：全部静态方法，无需创建对象；SingleLeg/Gait/主程序都不直接碰舵机驱动。
 *
 * 接线：PCA9685.SDA→ESP32.GPIO21、PCA9685.SCL→ESP32.GPIO22，
 *       逻辑供电与 I²C 电平匹配，舵机电源独立供电并共地（见 Config.h）。
 *
 * 使用：setup 里无需单独调 begin()，SingleLeg::init() 内部会自动调用（幂等）。
 *      写入示例：ServoManager::write(legId, ServoManager::FEMUR, 90.0f);
 */
#ifndef SERVO_MANAGER_H
#define SERVO_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Config.h"
#include "WalkMotion.h"

class ServoManager {
public:
    // 关节索引，对应 SERVO_PINS[6][2] / ANGLE_OFFSET[6][2] 的第二维
    enum Joint {
        FEMUR = 0,   // 大腿
        TIBIA = 1    // 小腿（膝盖）
    };

    static void begin();   // 初始化 PCA9685（50Hz），不驱动任何舵机（上电不动）；幂等，可重复调用

    // 写入关节角度（度）：angleDeg + 偏移 → 限幅 0~180 → 计算脉冲 → 写 PCA9685。
    // 偏移符号按实际安装校准：角度偏了就翻转 Config.h 里对应值的正负。
    // 每次真正驱动某路舵机都会【节流回传一行数字孪生遥测 @SERVO:<ch>:<实际到达角>】
    // （ch = PCA9685 输出通道；节流 ≥0.5° 且 ≥50ms），供 gait-sim「悬空·舵机孪生」等
    // 上位机 1:1 镜像 —— 任何经 write() 写舵机的模式（单舵机/单腿/六腿同步/步态…）通用。
    // 注：舵机无位置反馈，遥测是"固件下发的指令角"，不代表真机一定到位。
    static void write(int leg, Joint joint, float angleDeg);

    // 数字孪生遥测开关（默认开）。不需要遥测（如嫌串口输出刷屏）时可在 setup 里关闭。
    static void setTelemetry(bool on);
    static const motion::JointCommand* lastCommand(int channel);
    static bool takeWriteFailure();

    // 便捷写法（可选）
    static void writeFemur(int leg, float deg) { write(leg, FEMUR, deg); }
    static void writeTibia(int leg, float deg) { write(leg, TIBIA, deg); }

    // 角度(0~180) → PCA9685 脉冲计数（102~512，对应 0.5ms~2.5ms @50Hz）
    static int angleToPulse(float deg);

private:
    // [leg][joint] → PCA9685 通道 ch = leg*2 + joint（见 Config.h SERVO_PINS 注释）
    static Adafruit_PWMServoDriver pwm_;
    static bool initialized_;
};

#endif
