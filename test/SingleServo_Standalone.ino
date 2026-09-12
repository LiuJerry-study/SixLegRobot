/*
 * 单舵机转动测试（ESP32 独立版：单路 GPIO + ESP32Servo 库）
 *
 * 目的：确认「一个舵机」能否正常转动（通电 + 信号）。
 *       本文件与六足工程的 PCA9685 方案无关：不经 ServoManager、没有通道映射、
 *       也不叠加 Config.h 的零位偏移 ANGLE_OFFSET —— 就是一根 GPIO 出 PWM 试舵机。
 *       （src/main.cpp #include 的是 test/SingleServoTest.cpp，那是 PCA9685 版，
 *         对应 -DMODE_SINGLE_SERVO_TEST；`pio run` 只编译 src/，.ino 也不参与编译。）
 *       要在 Arduino IDE 里跑本文件，需先装库 "ESP32Servo"。
 *
 * 接线：
 *   舵机棕线(GND) ──► ESP32 扩展板 GND 引脚
 *   舵机信号(橙黄) ─► ESP32 扩展板 D13 (GPIO13)
 *   舵机红线(5V) ──► 外接 5V 电源正极
 *   外接电源 GND ──► ESP32 扩展板 GND 引脚   ← 必须共地！
 *
 * 效果：舵机在 30°↔150° 之间来回摆动（SWEEP_MS=500 为单程时长，来回约 1 秒），
 *       串口每 200ms 打印一次当前插值角度。
 */

#include <Arduino.h>
#include <ESP32Servo.h>

#define SERVO_PIN 13        // 信号线接的 GPIO（可改成 D14 等其他 PWM 脚）

Servo myservo;

const float SWEEP_MIN = 30.0f;   // 最小角度（度，直接给舵机，无偏移补偿）
const float SWEEP_MAX = 150.0f;  // 最大角度（度）
const unsigned long SWEEP_MS = 500;  // 单程时长(毫秒)，来回约 1 秒

// heading_to_max：true = 当前朝 SWEEP_MAX 方向摆
// last_sweep_time：本次单程的起始时刻
// current_angle：当前插值角度（setup 里先写到 SWEEP_MIN）
bool heading_to_max = true;
unsigned long last_sweep_time = 0;
float current_angle = SWEEP_MIN;

void setup() {
    Serial.begin(115200);
    delay(300);

    myservo.attach(SERVO_PIN);         // 绑定舵机到引脚（ESP32Servo 内部用 LEDC 出 50Hz PWM）
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

    // ⚠ write() 的形参是整数度，传 float 会被隐式截断（如 30.7°→30°），本测试无影响
    myservo.write(current_angle);

    static unsigned long last_print = 0;
    if (now - last_print > 200) {
        last_print = now;
        // 打印的是内部插值角度（不是从舵机读回的角）
        Serial.printf("angle=%.1f\n", current_angle);
    }
    // 控制循环频率：LEDC 会持续输出，无需高频重写
    delay(10);
}
