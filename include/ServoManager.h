/* 舵机硬件抽象：集中管理 6腿 × 3关节 = 18 个舵机（方案 A）。
 *
 * 职责：attach 全部舵机、写入时叠加零位偏移、统一限幅 0~180°。
 * 引脚与偏移分别取自 Config.h 的 SERVO_PINS / ANGLE_OFFSET。
 * 单例风格：全部静态方法，无需创建对象；SingleLeg/Gait/主程序都不直接碰 Servo。
 *
 * 使用：setup 里无需单独调 begin()，SingleLeg::init() 内部会自动调用（幂等）。
 *      写入示例：ServoManager::write(legId, ServoManager::BASE, 90.0f);
 *
 * 注意：ESP32Servo 库默认最多支持 16 路舵机（timer 限制），
 *       18 路需先调大库配置，上板前务必确认。
 */
#ifndef SERVO_MANAGER_H
#define SERVO_MANAGER_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "Config.h"

class ServoManager {
public:
    // 关节索引，对应 SERVO_PINS[6][3] / ANGLE_OFFSET[6][3] 的第二维
    enum Joint {
        BASE  = 0,   // 髋（基节）：整条腿水平旋转
        FEMUR = 1,   // 大腿
        TIBIA = 2    // 小腿（膝盖）
    };

    static void begin();   // attach 全部 18 个舵机；幂等，可重复调用

    // 写入关节角度（度）：angleDeg + 偏移 → 限幅 0~180 → 舵机.write
    // 偏移符号按实际安装校准：角度偏了就翻转 Config.h 里对应值的正负。
    static void write(int leg, Joint joint, float angleDeg);

    // 便捷写法（可选）
    static void writeBase(int leg, float deg)  { write(leg, BASE, deg); }
    static void writeFemur(int leg, float deg) { write(leg, FEMUR, deg); }
    static void writeTibia(int leg, float deg) { write(leg, TIBIA, deg); }

private:
    static Servo servos_[6][3];
    static bool attached_;
};

#endif
