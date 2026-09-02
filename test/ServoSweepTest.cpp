/* 单舵机转动测试（Arduino sketch）
 *
 * 目的：测试某个舵机是否能正常转动（通电 + 信号是否正常）。
 *
 * 用法：
 *   1. 用常量 TARGET_LEG / TARGET_JOINT 指定要测的舵机
 *      - TARGET_LEG   = 0~5，对应 6 条腿
 *      - TARGET_JOINT = FEMUR(大腿) 或 TIBIA(小腿)
 *   2. 上传到 ESP32 后，舵机应不断在 30°↔150° 之间来回摆动（一个来回约 1 秒）。
 *   3. 观察：舵机能跟着摆动 → 正常；不动/抖动 → 接线或供电问题。
 *
 * 注意：
 *   - 仅会给指定舵机发信号，其余 11 个舵机不通电（不 attach），无需担心误动。
 *   - 供电务必足够（独立 5V 电源，避免板载供电带不动时舵机无力或抖动）。
 *   - 若测到某腿的小腿(TIBIA)方向与预期相反，检查 ANGLE_OFFSET 或机械安装。
 *
 * 本文件为自包含版，直接在 Arduino IDE 里与以下源文件一起编译：
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

    // 初始化舵机（只 attach 指定这一个，其它 11 个不动）
    ServoManager::begin();

    Serial.printf("ServoTest: leg=%d joint=%s sweep %d°~%d°\n",
                  TARGET_LEG,
                  (TARGET_JOINT == ServoManager::FEMUR) ? "FEMUR" : "TIBIA",
                  (int)SWEEP_MIN, (int)SWEEP_MAX);

    // 先回到最小角度，再开始摆动
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

    // 线性插值从一端摆到另一端
    float t = (float)(now - last_sweep_time) / (float)SWEEP_MS;
    t = constrain(t, 0.0f, 1.0f);

    if (heading_to_max) {
        current_angle = SWEEP_MIN + (SWEEP_MAX - SWEEP_MIN) * t;
    } else {
        current_angle = SWEEP_MAX - (SWEEP_MAX - SWEEP_MIN) * t;
    }

    ServoManager::write(TARGET_LEG, (ServoManager::Joint)TARGET_JOINT, current_angle);

    // 每 200ms 打印一次当前角度，方便观察
    static unsigned long last_print = 0;
    if (now - last_print > 200) {
        last_print = now;
        Serial.printf("angle=%.1f\n", current_angle);
    }

    delay(10);  // 控制更新频率，避免写入过密
}
