/* 舵机初始化与角度写入（PCA9685 版本 + 偏移补偿 + 限幅 + 数字孪生遥测）
 *
 * 所有舵机通过 PCA9685（I2C）驱动，12 路输出通道 ch0~ch11。
 * 通道映射：channel = leg*2 + joint（Config.h SERVO_PINS[6][2]）。
 *
 * 上电行为：begin() 只初始化 Wire + PCA9685（50Hz），**不向任何通道写脉冲**
 *          → 上电瞬间 12 路舵机全部无力不动，只有被 write() 显式写过的那一路才动。
 * 写入行为：write() = 角度 + 零位偏移 → 限幅 0~180 → 换算脉冲 → setPWM，
 *          同时节流回传一行 @SERVO 遥测（见下方「数字孪生遥测」）。
 */

#include "ServoManager.h"
#include <math.h>   // fabsf


Adafruit_PWMServoDriver ServoManager::pwm_ = Adafruit_PWMServoDriver(PCA9685_I2C_ADDR);
bool ServoManager::initialized_ = false;

// ===== 数字孪生遥测（本文件内私有状态，匿名命名空间 = 仅本编译单元可见） =====
// 所有经 ServoManager::write() 的舵机驱动（任意测试模式通用）都会节流回传一行：
//     @SERVO:<ch>:<实际到达角>    例： @SERVO:0:157.4
// ch = PCA9685 输出通道；实际到达角 = 逻辑角 + 零位偏移（限幅 0~180）。
// gait-sim「悬空模式」解析该行即可 1:1 镜像真机 12 路舵机姿态。
// 遥测只反映"固件想让舵机去哪"，不代表舵机真的到位（无位置反馈）。
namespace {
    bool telemetryOn = true;                 // 默认开；可用 ServoManager::setTelemetry(false) 关
    float lastEchoDeg[12] = { 0 };           // 最近一次回传的最终角（用于 ≥0.5° 变化判断）
    unsigned long lastEchoMs[12] = { 0 };    // 最近一次回传时刻（用于 ≥50ms 节流；0=尚未回传）
    motion::JointCommand commands[robot::CHANNEL_COUNT] = {};
    bool known[robot::CHANNEL_COUNT] = {};
    bool writeFailed = false;
}

// 初始化 PCA9685（50Hz）；幂等。
// 注意：本函数【不再驱动任何舵机】——上电后所有通道保持无脉冲输出，
// 舵机处于无力状态、完全不动。只有显式调用 write() 的那一路才会动，
// 从而把电流峰值限制在"单台舵机"，避免电源不足时 12 路同时驱动的冲击。
void ServoManager::begin() {
    if (initialized_) return;

    // 初始化 I2C（SDA/SCL 由 Config.h 提供）
    Wire.begin(PCA9685_SDA_PIN, PCA9685_SCL_PIN);

    // 初始化 PCA9685 并设为舵机标准 50Hz
    pwm_.begin();
    pwm_.setOscillatorFrequency(PCA9685_OSC_FREQ_HZ);   // 常用内部振荡 25MHz，换晶振板才需要调
    pwm_.setPWMFreq(PCA9685_PWM_FREQ_HZ);               // 50Hz

    // 此处不再写任何 setPWM：PCA9685 复位后输出为低电平，舵机不收脉冲 → 不动。
    // 需要让某路舵机去某个角度时，由调用方显式 ServoManager::write() 完成。

    initialized_ = true;
}

void ServoManager::setTelemetry(bool on) {
    telemetryOn = on;
}

const motion::JointCommand* ServoManager::lastCommand(int channel) {
    if (channel < 0 || channel >= robot::CHANNEL_COUNT || !known[channel]) return nullptr;
    return &commands[channel];
}

bool ServoManager::takeWriteFailure() {
    const bool failed = writeFailed;
    writeFailed = false;
    return failed;
}

void ServoManager::write(int leg, Joint joint, float angleDeg) {
    if (leg < 0 || leg >= robot::LEG_COUNT || joint < FEMUR || joint > TIBIA || !isfinite(angleDeg)) return;
    // 防御：未初始化时写入会静默失败，打一次警告
    if (!initialized_) {
        static bool warned = false;
        if (!warned) {
            Serial.println("[ServoManager] 未调用 begin()，忽略写入");
            warned = true;
        }
        return;
    }

    const int ch = robot::SERVO_PINS[leg][joint];
    const auto next = motion::command(leg, joint, angleDeg);
    const float target = next.applied;
    const int pulse = angleToPulse(target);
    if (!known[ch] || pulse != angleToPulse(commands[ch].applied)) {
        if (pwm_.setPWM(ch, 0, pulse) != 0) {
            writeFailed = true;
            return;
        }
    }
    const bool first = !known[ch];
    commands[ch] = next;
    known[ch] = true;

    // 数字孪生遥测：实际到达角变化 ≥0.5° 才回传，且每通道 ≥50ms 一条（防刷屏）。
    // 角度不变或微小抖动时不回传，避免静止时也刷行。
    if (telemetryOn && ch >= 0 && ch < 12) {
        unsigned long now = millis();
        if (first || (fabsf(target - lastEchoDeg[ch]) >= 0.5f && now - lastEchoMs[ch] >= 50)) {
            lastEchoDeg[ch] = target;
            lastEchoMs[ch] = now;
            Serial.printf("@SERVO:%d:%.1f\n", ch, target);
        }
    }
}

// 角度(0~180) → 脉冲计数（数值见 Config.h，默认 102~512 @50Hz）
int ServoManager::angleToPulse(float deg) {
    // 夹紧输入范围，避免越界
    deg = constrain(deg, 0.0f, 180.0f);
    int pulse = (int)(SERVO_MIN_PULSE_CNT +
                  (SERVO_MAX_PULSE_CNT - SERVO_MIN_PULSE_CNT) * (deg / 180.0f));
    return pulse;
}
