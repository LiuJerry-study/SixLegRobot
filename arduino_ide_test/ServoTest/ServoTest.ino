/* =====================================================================
 * 通用舵机测试代码（PCA9685 + ESP32 / Arduino）
 * ---------------------------------------------------------------------
 * 【功能】通过 PCA9685（I2C 16 路 PWM 驱动板）驱动舵机做测试，三种模式：
 *
 *   mode 1  AUTO_SWEEP  自动扫描：所有通道按顺序来回摆动，快速逐只检查舵机。
 *   mode 2  ALL_SWEEP   同步扫描：所有舵机一起摆动，用于检查电源负载能力。
 *   mode 3  SERIAL      串口控制：输入 <通道> <角度> 精确控制任意一只舵机，
 *                                 适合用来标定角度/零位偏移。
 *
 * 【串口命令语法】（仅 mode 3）
 *   - "0 90"    → 通道 0 转到 90°
 *   - "5 45"    → 通道 5 转到 45°
 *   - "sweep"   → 切到自动扫描模式
 *   - "all"     → 切到同步扫描模式
 *   - "mode 3"  → 切回串口控制模式
 *
 * 【硬件接线】
 *   PCA9685 模块：
 *     - PCA9685.SDA → ESP32.GPIO21   （Arduino UNO/Nano 则接 A4）
 *     - PCA9685.SCL → ESP32.GPIO22   （Arduino UNO/Nano 则接 A5）
 *     - PCA9685.VCC → 5V（板载逻辑供电）
 *     - PCA9685 V+  → 独立 5V 电源（舵机供电，电流较大务必外接！）
 *     - GND         → 主控 GND（共地）
 *   舵机接到 PCA9685 的 CH0~CH11（按你的接线改 SERVO_CHANNELS 数量）
 *
 * 【Arduino IDE 烧录步骤】
 *   1. 装 ESP32 板支持（若用 Arduino UNO 则不用装，开发板选对即可）
 *   2. 工具 → 管理库，安装 "Adafruit PWM Servo Driver Library"（会带 BusIO）
 *   3. 开发板选对应板子 → 点"上传" → 打开串口监视器（115200）
 *
 *   ★ PlatformIO 也可编译：把本文件复制为 src/main.cpp 即可用
 *     （需在 platformio.ini lib_deps 增加 Adafruit PWM Servo Driver Library）
 * ===================================================================== */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ==================== 可配置参数 ====================
// PCA9685 默认地址 0x40
#define PCA9685_ADDR       0x40

// 主控型号自动识别：
//   - ESP32 / ESP8266 用指定 I2C 引脚（ESP32 默认 GPIO21/22）
//   - 其它 Arduuo（UNO/Nano/Mega 等）用默认 I2C 引脚（A4/A5 或 20/21）
#if defined(ESP32) || defined(ESP8266)
  #define PCA9685_SDA      21
  #define PCA9685_SCL      22
  #define USE_I2C_PINS     1
#else
  #define USE_I2C_PINS     0
#endif

// 舵机 PWM 参数（按你的舵机微调！）
#define SERVO_FREQ_HZ      50    // 舵机标准 50Hz
#define SERVO_MIN_PULSE    102   // 0°   ≈ 0.5ms
#define SERVO_MAX_PULSE    512   // 180° ≈ 2.5ms

// 扫描用的角度范围（别超过机械限位！）
#define SWEEP_MIN_ANGLE    30    // 扫描下限
#define SWEEP_MAX_ANGLE    150   // 扫描上限
#define SWEEP_STEP_MS      15    // 每步间隔 ms，越小转越快

// 测试的舵机数量（最多 16，按你的接线改；12=6腿*2关节）
#define SERVO_COUNT        12

// 运行模式（上电初始模式）
#define MODE_AUTO_SWEEP    1
#define MODE_ALL_SWEEP     2
#define MODE_SERIAL        3
#define INITIAL_MODE       MODE_AUTO_SWEEP
// ====================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

int g_mode = INITIAL_MODE;

// 每个通道当前的扫描方向（+1 增大 / -1 减小）
int8_t g_goingUp[SERVO_COUNT];

// ---- 函数原型声明（兼容非 .ino 编译环境，如 PlatformIO 的 main.cpp）----
int  angleToPulse(float deg);
void writeServo(int channel, float deg);
void sweepStep(int channel, int step);
void centerAll();
void printHelp();
void handleCommand(const String& line);
bool isNumeric(const String& s);

// 角度(0~180) → PCA9685 脉冲计数
int angleToPulse(float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  return (int)(SERVO_MIN_PULSE +
         (SERVO_MAX_PULSE - SERVO_MIN_PULSE) * (deg / 180.0f));
}

// 写入角度到指定通道（0~15）
void writeServo(int channel, float deg) {
  if (channel < 0 || channel > 15) return;
  pwm.setPWM(channel, 0, angleToPulse(deg));
}

// 让某个通道做一步摆动（基于上次角度推进 +step）
void sweepStep(int channel, int step) {
  static int angle[SERVO_COUNT];   // 保存每通道当前角度
  static bool inited = false;
  if (!inited) {
    for (int c = 0; c < SERVO_COUNT; c++) {
      angle[c] = SWEEP_MIN_ANGLE;
      g_goingUp[c] = 1;
    }
    inited = true;
  }
  angle[channel] += step * g_goingUp[channel];
  if (angle[channel] >= SWEEP_MAX_ANGLE) { angle[channel] = SWEEP_MAX_ANGLE; g_goingUp[channel] = -1; }
  if (angle[channel] <= SWEEP_MIN_ANGLE) { angle[channel] = SWEEP_MIN_ANGLE; g_goingUp[channel] = 1; }
  writeServo(channel, angle[channel]);
}

// 定位所有舵机到中间位 90°
void centerAll() {
  for (int c = 0; c < SERVO_COUNT; c++) writeServo(c, 90.0f);
}

// 打印帮助
void printHelp() {
  Serial.println("=== ServoTest Help ===");
  Serial.println("mode 1 : auto sweep (逐个扫描)");
  Serial.println("mode 2 : all sweep  (同步扫描)");
  Serial.println("mode 3 : serial    (串口控制: <ch> <angle>)");
  Serial.println("示例:");
  Serial.println("  0 90    -> 通道0转到90°");
  Serial.println("  sweep   -> 自动扫描");
  Serial.println("  all     -> 同步扫描");
  Serial.println("  mode 3  -> 切回串口控制");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // 初始化 I2C（不同平台引脚不同）
  #if USE_I2C_PINS
    Wire.begin(PCA9685_SDA, PCA9685_SCL);
  #else
    Wire.begin();
  #endif

  // 初始化 PCA9685 并设为 50Hz
  pwm.begin();
  pwm.setOscillatorFrequency(25000000);  // 内部振荡 25MHz
  pwm.setPWMFreq(SERVO_FREQ_HZ);

  // 上电把所有舵机摆到中间，避免突然乱动
  centerAll();

  Serial.println("ServoTest ready. 12 channels @50Hz.");
  printHelp();
  Serial.println(g_mode == MODE_AUTO_SWEEP ? "> Mode: AUTO SWEEP"
              : g_mode == MODE_ALL_SWEEP  ? "> Mode: ALL SWEEP"
              :                             "> Mode: SERIAL");
}

void loop() {
  // ---------- 处理串口命令 ----------
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) handleCommand(line);
    // 命令处理后延迟一点，避免同一批数据被误处理
    delay(5);
  }

  // ---------- 根据当前模式执行 ----------
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < SWEEP_STEP_MS) return;
  last = now;

  if (g_mode == MODE_AUTO_SWEEP) {
    // 逐个通道轮流扫描：按顺序每步推进一档
    static int chan = 0;
    sweepStep(chan, 1);
    chan = (chan + 1) % SERVO_COUNT;   // 轮流到下一个通道
  }
  else if (g_mode == MODE_ALL_SWEEP) {
    // 所有通道同步摆动
    for (int c = 0; c < SERVO_COUNT; c++) sweepStep(c, 1);
  }
  // MODE_SERIAL 不做自动动作，等串口命令
}

// 解析并执行一条串口命令
void handleCommand(const String& line) {
  // 模式切换命令
  if (line.equalsIgnoreCase("sweep")) {
    g_mode = MODE_AUTO_SWEEP;
    centerAll();
    Serial.println("> Mode: AUTO SWEEP");
    return;
  }
  if (line.equalsIgnoreCase("all")) {
    g_mode = MODE_ALL_SWEEP;
    centerAll();
    Serial.println("> Mode: ALL SWEEP");
    return;
  }
  if (line.equalsIgnoreCase("mode 3")) {
    g_mode = MODE_SERIAL;
    Serial.println("> Mode: SERIAL (type <ch> <angle>)");
    return;
  }
  if (line.equalsIgnoreCase("help")) {
    printHelp();
    return;
  }

  // 否则解析 "<通道> <角度>" 两个数字
  int spaceIdx = line.indexOf(' ');
  if (spaceIdx <= 0) {
    Serial.println("! 格式: <通道> <角度>, 例如: 0 90");
    return;
  }
  String chStr = line.substring(0, spaceIdx);
  String angStr = line.substring(spaceIdx + 1);
  chStr.trim(); angStr.trim();

  // 校验数字
  if (!isNumeric(chStr) || !isNumeric(angStr)) {
    Serial.println("! 请输入数字，例如: 0 90");
    return;
  }

  int channel = chStr.toInt();
  float angle = angStr.toFloat();
  if (channel < 0 || channel >= SERVO_COUNT) {
    Serial.print("! 通道越界(0~"); Serial.print(SERVO_COUNT - 1); Serial.println(")");
    return;
  }
  angle = constrain(angle, 0.0f, 180.0f);
  writeServo(channel, angle);
  Serial.print("> CH"); Serial.print(channel);
  Serial.print(" -> "); Serial.println(angle);
}

// 判断字符串是否为合法数字
bool isNumeric(const String& s) {
  if (s.length() == 0) return false;
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (!(c >= '0' && c <= '9') && c != '-' && c != '.') return false;
  }
  return true;
}
