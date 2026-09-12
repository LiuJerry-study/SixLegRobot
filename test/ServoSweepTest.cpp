/* 单舵机转动测试（Arduino sketch）
 *
 * 目的：测试某个舵机是否能正常转动（通电 + 信号是否正常）。
 *
 * 用法：
 *   1. 用常量 TARGET_LEG / TARGET_JOINT 指定要测的舵机
 *      - TARGET_LEG   = 0~5，对应 6 条腿
 *      - TARGET_JOINT = FEMUR(大腿) 或 TIBIA(小腿)
 *   2. 上传到 ESP32 后，舵机应不断在 30°↔150° 之间来回摆动
 *      （SWEEP_MS=500 是单程时长，一个来回约 1 秒）。
 *   3. 观察：舵机能跟着摆动 → 正常；不动/抖动 → 接线或供电问题。
 *
 * 注意：
 *   - 只有被 write() 显式写到的那一路（TARGET_LEG*2 + TARGET_JOINT 对应的 PCA9685
 *     通道）会输出脉冲，其余 11 路在 begin() 之后一直无脉冲、舵机无力不动 ——
 *     PCA9685 上电不会把舵机"复位"到某个角度，所以不必担心别的腿乱动。
 *   - 本文件里的 30°~150° 是【逻辑角】：ServoManager::write() 会自动叠加 Config.h 的
 *     零位偏移 ANGLE_OFFSET 并限幅 0~180，即舵机实际到达角 = 逻辑角 + 偏移。
 *     偏移大的通道可能顶到 0°/180° 边界（例如小腿通道偏移 +45°~+63°）。
 *   - 供电务必足够（独立 5V 电源，避免板载供电带不动时舵机无力或抖动）。
 *   - 若测到某腿的小腿(TIBIA)方向与预期相反，检查 ANGLE_OFFSET 或机械安装。
 *
 * 本文件是保留的独立实现（自含 setup/loop），不在 src/ 下，`pio run` 不编译它，
 * 也不是 PlatformIO 单元测试；等价的平台模式见 src/main.cpp 的 -DMODE_SERVO_SWEEP
 * （同一套常量与摆动逻辑，只是改走 ServoManager）。
 * 要在 Arduino IDE 里单独编译，需与以下源文件一起编译：
 *   include/Config.h、include/ServoManager.h、src/ServoManager.cpp
 */

#include <Arduino.h>
#include "ServoManager.h"

// ====== 要测试的舵机 ======
const int TARGET_LEG   = 0;   // 0~5：哪条腿
const int TARGET_JOINT = ServoManager::FEMUR;  // FEMUR(大腿) 或 TIBIA(小腿)

// ====== 摆动幅度与速度 ======
const float SWEEP_MIN = 30.0f;   // 最小角度（度）
const float SWEEP_MAX = 150.0f;  // 最大角度（度）
const unsigned long SWEEP_MS = 500;  // 单程（MIN→MAX 或 MAX→MIN）时长（毫秒）

// 内部状态
bool heading_to_max = true;      // 当前是否朝 MAX 方向摆动
unsigned long last_sweep_time = 0;
float current_angle = SWEEP_MIN;

void setup() {
    Serial.begin(115200);
    delay(200);

    // 初始化 PCA9685（begin() 只做 Wire + 驱动板初始化，不驱动任何舵机；
    // 幂等，可重复调用）；之后只有下面这一路被 write() 写到
    ServoManager::begin();

    Serial.printf("ServoTest: leg=%d joint=%s sweep %d°~%d°\n",
                  TARGET_LEG,
                  (TARGET_JOINT == ServoManager::FEMUR) ? "FEMUR" : "TIBIA",
                  (int)SWEEP_MIN, (int)SWEEP_MAX);

    // 先写到最小角度（逻辑角 SWEEP_MIN，叠加偏移后才是舵机实际角），再开始摆动
    ServoManager::write(TARGET_LEG, (ServoManager::Joint)TARGET_JOINT, SWEEP_MIN);
    current_angle = SWEEP_MIN;
    last_sweep_time = millis();
}

void loop() {
    unsigned long now = millis();
    unsigned long dt  = now - last_sweep_time;

    if (dt >= SWEEP_MS) {
        // 到达端点，反转方向
        heading_to_max = !heading_to_max;
        last_sweep_time = now;
        current_angle = heading_to_max ? SWEEP_MIN : SWEEP_MAX;
    }

    // 线性插值从一端摆到另一端（单程 SWEEP_MS，来回约 1 秒）
    float t = (float)(now - last_sweep_time) / (float)SWEEP_MS;
    t = constrain(t, 0.0f, 1.0f);

    if (heading_to_max) {
        current_angle = SWEEP_MIN + (SWEEP_MAX - SWEEP_MIN) * t;
    } else {
        current_angle = SWEEP_MAX - (SWEEP_MAX - SWEEP_MIN) * t;
    }

    ServoManager::write(TARGET_LEG, (ServoManager::Joint)TARGET_JOINT, current_angle);

    // 每 200ms 打印一次当前角度，方便观察（打印的是逻辑角，不含零位偏移）
    static unsigned long last_print = 0;
    if (now - last_print > 200) {
        last_print = now;
        Serial.printf("angle=%.1f\n", current_angle);
    }

    // PCA9685 一直输出最后一次脉冲，重写只是保持；ServoManager 的 @SERVO 遥测自带
    // 「角度变化 ≥0.5° 且每通道 ≥50ms 一条」节流，所以这里高频写入也不会刷屏
    delay(10);  // 控制更新频率（约 10ms 一次）
}
