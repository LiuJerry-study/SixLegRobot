/* ============================================================
 *  ESP32 + PCA9685 + 2 个舵机  独立测试代码
 * ------------------------------------------------------------
 *  用途：快速验证 PCA9685 驱动板与 2 个舵机是否正常工作。
 *       setup 先把两路都写到 90° 作为初始位置，随后两个舵机在
 *       0°~180° 之间【同步】来回摆动（同一角度、同一脉冲）。
 *
 *  硬件：PCA9685 走 I2C（SDA=GPIO21、SCL=GPIO22、地址 0x40、PWM 50Hz），
 *       两个舵机接 ch0 / ch1 —— 与六足固件 Config.h 的 SERVO_PINS 口径一致
 *       （ch0 = leg0 大腿、ch1 = leg0 小腿）。
 *
 *  ⚠ 注意：本文件直接调 Adafruit 库的 pwm.setPWM()，【不经过 ServoManager】：
 *     既不会叠加 Config.h 的零位偏移 ANGLE_OFFSET，也不做 0~180 限幅，
 *     更不会回传 @SERVO 数字孪生遥测 —— 与 SingleServoTest / SingleLegTest
 *     那些走 ServoManager 的用例写法不同（这里角度就是实打实的脉冲映射）。
 *
 *  本文件是"独立测试"入口（自含 setup/loop）。它不在 src/ 下，`pio run`
 *  只编译 src/，所以它默认不参与编译、也不是 PlatformIO 单元测试。
 *  要单独烧录请看文件末尾的"编译 / 烧录说明"。
 * ============================================================ */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ================= PCB 接线常量（见文字说明） =================
// PCA9685 通过 I2C 连接 ESP32
//   SDA  → 引脚 21   （ESP32 默认 I2C 数据线）
//   SCL  → 引脚 22   （ESP32 默认 I2C 时钟线）
// 使用 I2C 默认引脚，也可改成你实际用的
// （与 include/Config.h 的 PCA9685_SDA_PIN / PCA9685_SCL_PIN 取的是同一组引脚）
#define SDA_PIN 21
#define SCL_PIN 22

// PCA9685 的 I2C 地址（默认 0x40，未改跳线则保持）
#define PCA9685_ADDR 0x40

// 两个舵机接在 PCA9685 的哪些输出通道（ch0~ch15）
// 这里取 0 / 1，正好就是六足固件里 leg0 的大腿 / 小腿通道（Config.h SERVO_PINS[0]）
#define SERVO0_CH 0
#define SERVO1_CH 1

// ---------------- 舵机 PWM 参数 ----------------
// 这些要根据你舵机的规格调整（常见 180° 舵机如 SG90/MG996R）！
// 本文件用 setPWM(ch, 0, cnt) 直接写"脉冲宽度计数"，
// 而不是 0~4095 的裸占空比（裸计数 0 = 0% 占空比，对舵机没有意义，角度也不准）。
// 换算：PCA9685 时钟 50Hz（周期 20ms）、4096 级 → 每级 ≈ 4.9us
//   0.5ms (0°)  ≈  102
//   1.5ms (90°) ≈  307
//   2.5ms (180°)≈  512
// （include/Config.h 的 SERVO_MIN/MAX_PULSE_CNT 就是这两个数）
#define PWM_FREQ_HZ 50          // 舵机标准 50Hz（20ms 周期）
#define SERVO_MIN_CNT 102       // 0°  对应脉冲 0.5ms
#define SERVO_MAX_CNT 512       // 180° 对应脉冲 2.5ms

// ================= 全局对象 =================
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// 把 0~180 角度线性映射到 102~512 计数（调用处传的都在 0~180 内）
int angleToPulse(float deg) {
    // 注意：很多舵机不是严格的 0.5~2.5ms，装好了可微调（此处不叠任何偏移）
    float cnt = SERVO_MIN_CNT + (SERVO_MAX_CNT - SERVO_MIN_CNT) * (deg / 180.0f);
    return (int)cnt;
}

// 摆动相关变量
float angle = 0;
bool up = true;
unsigned long lastMove = 0;
const unsigned long STEP_MS = 15;   // 每 15ms 步进 2°：0→180° 约 1.35s，来回约 2.7s，方便目测

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

    // 上电先把两个舵机都写到 90°（中位，做初始位置，避免上电乱转）
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

        // 大致每 500ms 打印一次角度：比较 now 与 (now-STEP_MS) 落在哪个 500ms 分桶里，
        // 只有本次步进恰好跨过分桶边界时才打印（偶尔漏一两条，仅影响日志、不影响摆动）
        if (now / 500 != (now - STEP_MS) / 500) {
            Serial.printf("angle=%.1f pulse=%d\n", angle, pulse);
        }
    }
}

/*
 * ============================================================
 *  编译 / 烧录说明
 * ------------------------------------------------------------
 *  前置：本文件不在 src/ 下，`pio run` 只编译 src/，所以它默认不会被编译，
 *        也不会和 src/main.cpp 里的 setup()/loop() 撞名。
 *
 *  方式A（PlatformIO）：把本文件内容整份替换 src/main.cpp 后 Upload 即可
 *                      （src/ 下其他 .cpp 会一并编译但互不冲突——入口只有 main.cpp；
 *                        项目已弃用 scr/ 目录与 build_src_filter）。
 *  方式B（Arduino IDE）：当作 .ino sketch 打开本文件烧录，需先装库
 *                      "Adafruit PWM Servo Driver Library"（会连带安装 Adafruit BusIO）。
 *
 *  注：platformio.ini 末尾注释掉的示例环境只是骨架，真要用得自己补全（依赖等）。
 * ============================================================
 */
