/* =====================================================================
 * 实时角度控制测试（PCA9685 + ESP32 / Arduino）
 * ---------------------------------------------------------------------
 * 【功能】在串口监视器输入一个角度数值，选中的舵机就【实时】转到该角度。
 *
 *   - 默认控制【通道 0】；用命令 "ch <编号>" 切换要控制的通道。
 *   - 输入一个角度数字（如 45）→ 当前通道立即转向 45°。
 *   - 舵机会以设定速度【平滑插值】转到目标角（避免瞬间猛跳撞到限位）。
 *   - 支持连续输入：每输入一个角度就转到哪里，适合边动手推边用角度尺标定。
 *
 * 【串口命令语法】
 *   - "45"       → 当前通道转到 45°
 *   - "0"        → 转到 0°
 *   - "ch 3"     → 切换控制通道为 3
 *   - "speed 500" → 设转速（单位"度/秒"，如 500=接近全速；0=立刻到）
 *   - "center"   → 当前通道回中间位 90°
 *   - "help"     → 打印帮助
 *
 * 【硬件接线】
 *   PCA9685 模块：
 *     - PCA9685.SDA → ESP32.GPIO21   （Arduino UNO/Nano 则接 A4）
 *     - PCA9685.SCL → ESP32.GPIO22   （Arduino UNO/Nano 则接 A5）
 *     - PCA9685.VCC → 5V（板载逻辑供电）
 *     - PCA9685 V+  → 独立 5V 电源（舵机供电，电流较大务必外接！）
 *     - GND         → 主控 GND（共地）
 *   舵机接到 PCA9685 的 CH0~CH11（按你的接线改 SERVO_COUNT）
 *
 * 【Arduino IDE 烧录步骤】
 *   1. 装 ESP32 板支持（若用 Arduino UNO 则不用装，开发板选对即可）
 *   2. 工具 → 管理库，安装 "Adafruit PWM Servo Driver Library"
 *   3. 开发板选对应板子 → 点"上传" → 串口监视器（115200）
 *
 *   ★ PlatformIO 也可编译：把本文件复制为 src/main.cpp 即可用
 * ===================================================================== */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================== 可配置参数 ====================
#define PCA9685_ADDR       0x40

// 主控型号自动识别 I2C 引脚
#if defined(ESP32) || defined(ESP8266)
  #define PCA9685_SDA      21
  #define PCA9685_SCL      22
  #define USE_I2C_PINS     1
#else
  #define USE_I2C_PINS     0
#endif

// 舵机 PWM 参数
#define SERVO_FREQ_HZ      50
#define SERVO_MIN_PULSE    102   // 0°   ≈ 0.5ms
#define SERVO_MAX_PULSE    512   // 180° ≈ 2.5ms

// 测试的舵机数量（最多 16）
#define SERVO_COUNT        12

// 角度限幅（防止误输入撞到机械限位，按你的舵机改）
#define ANGLE_MIN          0.0f
#define ANGLE_MAX          180.0f

// 默认转速：单位"度/秒"（0=直接跳到目标，不插值）
// 常见舵机最快约 550~650 度/秒；默认 500 接近全速、又留有观察时间
#define DEFAULT_SPEED      500.0f

// 平滑更新周期（ms），越小越流畅；建议 10~25
#define UPDATE_INTERVAL_MS 20
// ====================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

int   g_channel = 0;
float g_speed   = DEFAULT_SPEED;      // 度/秒
float g_curAng[SERVO_COUNT];          // 每通道当前位置
float g_target[SERVO_COUNT];          // 每通道目标角度

// ---- 函数原型声明 ----
int  angleToPulse(float deg);
void writeServo(int channel, float deg);
void setTarget(int channel, float targetAngle);
void updateSmooth(unsigned long dtMs);
bool isNumeric(const String& s);
void handleCommand(const String& line);
void printHelp();

// 角度 → 脉冲
int angleToPulse(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return (int)(SERVO_MIN_PULSE +
         (SERVO_MAX_PULSE - SERVO_MIN_PULSE) * (deg / 180.0f));
}

// 直接写到指定通道
void writeServo(int channel, float deg) {
  if (channel < 0 || channel > 15) return;
  pwm.setPWM(channel, 0, angleToPulse(deg));
}

// 设定某通道的目标角度
void setTarget(int channel, float targetAngle) {
  g_target[channel] = constrain(targetAngle, ANGLE_MIN, ANGLE_MAX);
  // 速度为 0 时直接跳到目标
  if (g_speed <= 0) {
    g_curAng[channel] = g_target[channel];
    writeServo(channel, g_curAng[channel]);
  }
}

// 每次更新调用：按真实经过时间把当前通道向目标平滑推进（g_speed 单位度/秒）
void updateSmooth(unsigned long dtMs) {
  if (g_speed <= 0) {              // 速度为 0 = 直接跳转（已在 setTarget 处理）
    writeServo(g_channel, g_curAng[g_channel]);
    return;
  }

  float diff = g_target[g_channel] - g_curAng[g_channel];

  // 单帧最多走 g_speed 度/秒 × 实际间隔
  float maxMove = g_speed * ((float)dtMs / 1000.0f);

  if (fabs(diff) <= maxMove) {
    g_curAng[g_channel] = g_target[g_channel];       // 到位
  } else {
    g_curAng[g_channel] += (diff > 0 ? maxMove : -maxMove);
  }
  writeServo(g_channel, g_curAng[g_channel]);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  #if USE_I2C_PINS
    Wire.begin(PCA9685_SDA, PCA9685_SCL);
  #else
    Wire.begin();
  #endif

  pwm.begin();
  pwm.setOscillatorFrequency(25000000);
  pwm.setPWMFreq(SERVO_FREQ_HZ);

  // 初始化：所有通道回中间 90°
  for (int c = 0; c < SERVO_COUNT; c++) {
    g_curAng[c] = 90.0f;
    g_target[c] = 90.0f;
    writeServo(c, 90.0f);
  }

  Serial.println("AngleCtlTest ready.");
  printHelp();
  Serial.print("> 当前控制通道: CH"); Serial.println(g_channel);
}

void loop() {
  // 读串口
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) handleCommand(line);
    delay(5);
  }

  // 按固定周期平滑推进（用真实间隔时间算步长，速度稳定）
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < UPDATE_INTERVAL_MS) return;
  unsigned long dt = now - last;
  last = now;
  updateSmooth(dt);
}

// 解析并执行串口命令
void handleCommand(const String& line) {
  // "help"
  if (line.equalsIgnoreCase("help")) { printHelp(); return; }

  // "center"
  if (line.equalsIgnoreCase("center")) {
    setTarget(g_channel, 90.0f);
    Serial.println("> 回中间位 90°");
    return;
  }

  // "ch <编号>"
  if (line.length() >= 3 && line.substring(0, 2).equalsIgnoreCase("ch")) {
    String num = line.substring(2); num.trim();
    if (isNumeric(num)) {
      int c = num.toInt();
      if (c >= 0 && c < SERVO_COUNT) {
        g_channel = c;
        Serial.print("> 切换到 CH"); Serial.println(c);
      } else Serial.println("! 通道越界");
    }
    return;
  }

  // "speed <数值>"
  if (line.length() >= 5 && line.substring(0, 5).equalsIgnoreCase("speed")) {
    String num = line.substring(5); num.trim();
    if (isNumeric(num)) {
      g_speed = num.toFloat();
      Serial.print("> 转速设为 "); Serial.print(g_speed); Serial.println(" 度/秒");
    }
    return;
  }

  // 纯数字 → 当前通道转到该角度
  if (isNumeric(line)) {
    float target = line.toFloat();
    setTarget(g_channel, target);
    Serial.print("> CH"); Serial.print(g_channel);
    Serial.print(" -> "); Serial.println(constrain(target, ANGLE_MIN, ANGLE_MAX));
    return;
  }

  Serial.println("! 无法识别的命令，输入 help 查看用法");
}

// 判断字符串是否纯数字
bool isNumeric(const String& s) {
  if (s.length() == 0) return false;
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (!(c >= '0' && c <= '9') && c != '-' && c != '.') return false;
  }
  return true;
}

void printHelp() {
  Serial.println("=== AngleCtlTest Help ===");
  Serial.println("输入角度数字(如 45) → 当前通道转到该角度");
  Serial.println("  ch <编号>  切换控制通道(0~11)");
  Serial.println("  speed <n>  设转速(度/秒, 如500≈全速; 0=直接跳)");
  Serial.println("  center     回中间位90°");
  Serial.println("  help       帮助");
}
