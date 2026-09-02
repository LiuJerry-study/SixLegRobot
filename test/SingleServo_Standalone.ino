/*
 * 单舵机转动测试（ESP32 独立版）
 *
 * 目的：确认「一个舵机」能否正常转动（通电 + 信号）。
 *
 * 接线：
 *   舵机棕线(GND) ──► ESP32 扩展板 GND 引脚
 *   舵机信号(橙黄) ─► ESP32 扩展板 D13 (GPIO13)
 *   舵机红线(5V) ──► 外接 5V 电源正极
 *   外接电源 GND ──► ESP32 扩展板 GND 引脚   ← 必须共地！
 *
 * 效果：舵机在 30°↔150° 之间来回摆动，串口每 200ms 打印角度。
 */

#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_PIN 13        // 信号线接的 GPIO（可改成 D14 等其他 PWM 脚）

Servo myservo;

const float SWEEP_MIN = 30.0f;   // 最小角度
const float SWEEP_MAX = 150.0f;  // 最大角度
const unsigned long SWEEP_MS = 500;  // 单程时长(毫秒)

bool heading_to_max = true;
unsigned long last_sweep_time = 0;
float current_angle = SWEEP_MIN;

void setup() {
    Serial.begin(115200);
    delay(300);

    myservo.attach(SERVO_PIN);         // 绑定舵机到引脚
    myservo.write(SWEEP_MIN);           // 先归到最小角度
    current_angle = SWEEP_MIN;
    last_sweep_time = millis();

    Serial.printf("ServoTest: pin=%d, sweep %d~%d degrees\n",
                  SERVO_PIN, (int)SWEEP_MIN, (int)SWEEP_MAX);
}

void loop() {
    unsigned long now = millis();
    unsigned long dt  = now - last_sweep_time;

    if (dt >= SWEEP_MS) {              // 到端点，反向
        heading_to_max = !heading_to_max;
        last_sweep_time = now;
        current_angle = heading_to_max ? SWEEP_MIN : SWEEP_MAX;
    }

    float t = constrain((float)(now - last_sweep_time) / SWEEP_MS, 0.0f, 1.0f);
    if (heading_to_max) {
        current_angle = SWEEP_MIN + (SWEEP_MAX - SWEEP_MIN) * t;
    } else {
        current_angle = SWEEP_MAX - (SWEEP_MAX - SWEEP_MIN) * t;
    }

    myservo.write(current_angle);

    static unsigned long last_print = 0;
    if (now - last_print > 200) {
        last_print = now;
        Serial.printf("angle=%.1f\n", current_angle);
    }
    delay(10);
}
