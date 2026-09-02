/* ============================================================
 *  ESP32 + PCA9685 + 2 个舵机  独立测试代码
 * ------------------------------------------------------------
 *  用途：快速验证 PCA9685 驱动板与 2 个舵机是否正常工作。
 *       两个舵机将在 0°~180° 之间来回摆动。
 *
 *  本文件是"独立测试"入口，可单独编译烧录。
 *  编译/烧录方式见文件末尾说明。
 * ============================================================ */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ================= PCB 接线常量（见文字说明） =================
// PCA9685 通过 I2C 连接 ESP32
//   SDA  → 引脚 21   （ESP32 默认 I2C 数据线）
//   SCL  → 引脚 22   （ESP32 默认 I2C 时钟线）
// 使用 I2C 默认引脚，也可改成你实际用的
#define SDA_PIN 21
#define SCL_PIN 22

// PCA9685 的 I2C 地址（默认 0x40，未改跳线则保持）
#define PCA9685_ADDR 0x40

// 两个舵机接在 PCA9685 的哪些输出通道（ch0~ch15）
#define SERVO0_CH 0
#define SERVO1_CH 1

// ---------------- 舵机 PWM 参数 ----------------
// 这些要根据你舵机的规格调整！
// 常见 180° 舵机（如 SG90/MG996R）：
//   SERVO_MIN = 脉冲宽度（单位 0.5ms？）见下方换算说明
// 用 Adafruit 库时，填入的是"时钟周期计数"：
//   为了简单，这里用 setPWM(ch, 0, pulse)，pulse 直接用 0~4095 计数。
//   0 对应 0% 占空比，约等于舵机转 0° 并不准。
// ---- 更精确做法：直接写"脉冲宽度计数" ----
// 假设 PCA9685 时钟 50Hz（周期 20ms），4096 级 → 每级 ≈ 4.9us
//   0.5ms (0°)  ≈  102
//   1.5ms (90°) ≈  307
//   2.5ms (180°)≈  512
#define PWM_FREQ_HZ 50          // 舵机标准 50Hz（20ms 周期）
#define SERVO_MIN_CNT 102       // 0°  对应脉冲 0.5ms
#define SERVO_MAX_CNT 512       // 180° 对应脉冲 2.5ms

// ================= 全局对象 =================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// 把 0~180 角度映射到 102~512 计数
int angleToPulse(float deg) {
    // 注意：很多舵机不是严格的 0.5~2.5ms，装好了可微调
    float cnt = SERVO_MIN_CNT + (SERVO_MAX_CNT - SERVO_MIN_CNT) * (deg / 180.0f);
    return (int)cnt;
}

// 摆动相关变量
float angle = 0;
bool up = true;
unsigned long lastMove = 0;
const unsigned long STEP_MS = 15;   // 每 15ms 步进 2°（扫个来回大约几十秒，方便目测）

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("ESP32 + PCA9685 + 2 Servo Test");

    // 初始化 I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.printf("I2C begin: SDA=%d SCL=%d\n", SDA_PIN, SCL_PIN);

    // 初始化 PCA9685
    pwm.begin();
    pwm.setPWMFreq(PWM_FREQ_HZ);   // 50Hz
    Serial.println("PCA9685 initialized at 50Hz");

    // 上电先让两个舵机回到 0°（做初始位置，避免上电乱转）
    pwm.setPWM(SERVO0_CH, 0, angleToPulse(90));
    pwm.setPWM(SERVO1_CH, 0, angleToPulse(90));
    Serial.println("Both servos set to 90°");
}

void loop() {
    unsigned long now = millis();
    if (now - lastMove >= STEP_MS) {
        lastMove = now;

        // 角度往复 0~180
        if (up) {
            angle += 2.0f;
            if (angle >= 180.0f) up = false;
        } else {
            angle -= 2.0f;
            if (angle <= 0.0f) up = true;
        }

        int pulse = angleToPulse(angle);
        // 两个舵机同步摆动，验证输出一致
        pwm.setPWM(SERVO0_CH, 0, pulse);
        pwm.setPWM(SERVO1_CH, 0, pulse);

        // 每 500ms 打印一次角度，方便观察
        if (now / 500 != (now - STEP_MS) / 500) {
            Serial.printf("angle=%.1f pulse=%d\n", angle, pulse);
        }
    }
}

/*
 * ============================================================
 *  编译 / 烧录说明
 * ------------------------------------------------------------
 *  方式A（推荐，用独立环境）：
 *    在 platformio.ini 里新增一个环境（见 platformio.ini 中注释掉的示例环境），
 *    或直接把本文件拷成 src/main.cpp 后 Upload。
 *
 *  方式B（最快）：把本文件内容整体替换到 src/main.cpp 后 Upload 即可。
 *                （src/ 下其他 .cpp 会一并编译但互不冲突——入口只有 main.cpp；
 *                  项目已弃用 scr/ 目录与 build_src_filter，无需再改 platformio.ini。）
 * ============================================================
 */
