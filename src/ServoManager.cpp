/* 舵机初始化与角度写入（PCA9685 版本 + 偏移补偿 + 限幅）
 *
 * 所有舵机通过 PCA9685（I2C）驱动，12 路输出通道 ch0~ch11。
 * 通道映射：channel = leg*2 + joint（Config.h SERVO_PINS[6][2]）。
 */

#include "ServoManager.h"

Adafruit_PWMServoDriver ServoManager::pwm_ = Adafruit_PWMServoDriver(PCA9685_I2C_ADDR);
bool ServoManager::initialized_ = false;

// 初始化 PCA9685（50Hz）并复位全部 12 通道；幂等
void ServoManager::begin() {
    if (initialized_) return;

    // 初始化 I2C（SDA/SCL 由 Config.h 提供）
    Wire.begin(PCA9685_SDA_PIN, PCA9685_SCL_PIN);

    // 初始化 PCA9685 并设为舵机标准 50Hz
    pwm_.begin();
    pwm_.setOscillatorFrequency(PCA9685_OSC_FREQ_HZ);   // 常用内部振荡 25MHz，换晶振板才需要调
    pwm_.setPWMFreq(PCA9685_PWM_FREQ_HZ);               // 50Hz

    // 复位全部 12 个舵机到 90°（中间位），避免上电前乱转
    for (int leg = 0; leg < 6; leg++) {
        for (int j = 0; j < 2; j++) {
            pwm_.setPWM(robot::SERVO_PINS[leg][j], 0, angleToPulse(90.0f));
        }
    }

    initialized_ = true;
}

void ServoManager::write(int leg, Joint joint, float angleDeg) {
    // 防御：未初始化时写入会静默失败，打一次警告
    if (!initialized_) {
        static bool warned = false;
        if (!warned) {
            Serial.println("[ServoManager] 未调用 begin()，忽略写入");
            warned = true;
        }
        return;
    }

    float target = angleDeg + robot::ANGLE_OFFSET[leg][joint];  // 叠加零位偏移
    target = constrain(target, 0.0f, 180.0f);                    // 限幅防堵转

    int ch = robot::SERVO_PINS[leg][joint];   // PCA9685 输出通道（0~11）
    pwm_.setPWM(ch, 0, angleToPulse(target));
}

// 角度(0~180) → 脉冲计数（数值见 Config.h，默认 102~512 @50Hz）
int ServoManager::angleToPulse(float deg) {
    // 夹紧输入范围，避免越界
    deg = constrain(deg, 0.0f, 180.0f);
    int pulse = (int)(SERVO_MIN_PULSE_CNT +
                  (SERVO_MAX_PULSE_CNT - SERVO_MIN_PULSE_CNT) * (deg / 180.0f));
    return pulse;
}
