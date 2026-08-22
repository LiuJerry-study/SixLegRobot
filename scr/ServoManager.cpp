/* 舵机初始化与角度写入（偏移补偿 + 限幅） */

#include "ServoManager.h"

Servo ServoManager::servos_[6][2];
bool  ServoManager::attached_ = false;

// attach 全部 12 个舵机（引脚表 Config.h SERVO_PINS）；幂等
void ServoManager::begin() {
    if (attached_) return;

    for (int leg = 0; leg < 6; leg++) {
        for (int j = 0; j < 2; j++) {
            servos_[leg][j].attach(robot::SERVO_PINS[leg][j]);
        }
    }
    attached_ = true;
}

void ServoManager::write(int leg, Joint joint, float angleDeg) {
    // 防御：未初始化时写入会静默失败，打一次警告
    if (!attached_) {
        static bool warned = false;
        if (!warned) {
            Serial.println("[ServoManager] 未调用 begin()，忽略写入");
            warned = true;
        }
        return;
    }

    float target = angleDeg + robot::ANGLE_OFFSET[leg][joint];  // 叠加零位偏移
    target = constrain(target, 0.0f, 180.0f);                    // 限幅防堵转
    servos_[leg][joint].write(target);
}
