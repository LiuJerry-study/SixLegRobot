/* 平台单舵机测试（串口交互式，PCA9685 版）
 *
 * 目的：在六足平台上逐个验证 12 个舵机（6腿 × 大腿/小腿）是否正常。
 *       全程通过串口命令选择通道、设置角度、来回摆动 —— 不用改代码重新编译。
 *
 * ── 运行方式（PlatformIO）──────────────────────────────
 *   1. platformio.ini 的 build_flags 设为：-DMODE_SINGLE_SERVO_TEST
 *   2. Upload 烧录，打开串口监视器（115200 波特率）
 *   3. 发送命令（每行一条，回车结尾）：
 *
 *        help                → 打印命令帮助
 *        map                 → 打印 通道 ↔ (腿, 关节) 对照表
 *        center              → 全部 12 路写逻辑角 90°（"安全位"；实际角 = 90° + 偏移，
 *                              偏移表当前全 0 ⇒ 就是 90°）
 *        sweep <ch>          → 通道 ch 在 30°~150° 来回摆动（如 sweep 3）
 *        sweep leg <l> <j>   → (腿l, 关节j) 来回摆动（如 sweep leg 0 1）
 *        stop                → 停止摆动，保持当前角度
 *        <ch> <angle>        → 通道 ch 转到指定角度（如 3 90）
 *        leg <l> <j> <ang>   → (腿l, 关节j) 转到指定角度（如 leg 0 0 45）
 *
 *   通道编号 = 腿号×2 + 关节号（关节 0=大腿FEMUR，1=小腿TIBIA）：
 *     ch0=leg0大腿  ch1=leg0小腿  ch2=leg1大腿  ch3=leg1小腿 ...
 *     ch10=leg5大腿 ch11=leg5小腿
 *
 *   角度说明：命令与 sweep 里的角度是「逻辑角」；ServoManager 写入时自动叠加
 *   Config.h 的零位偏移 ANGLE_OFFSET（实际到达角 = 逻辑角 + 偏移，超 0~180 限幅）。
 *   ⚠ 该表当前【整表为 0，待按实物重新标定】（旧值按错误的大腿映射标定，已作废）
 *     ⇒ 现在实际到达角 = 逻辑角；重新标定后差值才会体现。
 *   map 会打印每通道的偏移，center / 单通道写入 / sweep 会打印实际到达角，
 *   便于在实物上核对 ANGLE_OFFSET 是否填得正确（角度偏了改对应值的正负）。
 *
 * ── 数字孪生遥测（供 gait-sim「悬空模式」镜像）────────────
 *   本测试【不自带】遥测代码：所有写舵机统一走 ServoManager::write()，
 *   而该函数已在固件层内置节流回传一行机器可读遥测：
 *
 *        @SERVO:<ch>:<角度>      例： @SERVO:0:67.4（默认站姿的大腿角；偏移全 0 即逻辑角）
 *
 *   ch = PCA9685 通道号；角度 = 叠加零位偏移并限幅(0~180)后的最终舵机角
 *   （偏移表当前整表为 0 ⇒ 就是逻辑角）。
 *   因此烧录单舵机/单腿/六腿同步等【任意经 ServoManager 写舵机的模式】，
 *   gait-sim「悬空模式」都能解析 @SERVO 并 1:1 镜像 12 路舵机（见 src/ServoManager.cpp）。
 *
 * ── 安全建议 ──────────────────────────────────────────
 *   - 上电时 ServoManager::begin() 只初始化驱动板、不会驱动任何舵机，
 *     机器人上电瞬间保持不动；只有你发命令指定的那一路舵机会动。
 *     若希望手动转动舵机臂，可先断开舵机电源或卸下舵机臂再操作；
 *   - 舵机电源用独立 5V，并与 ESP32 共地（接线见 Config.h 注释）；
 *   - 首次测试建议把舵机臂取下，先确认转向与角度范围。
 *
 * 本文件【不是 PlatformIO 单元测试】：src/main.cpp 里有
 *   #elif defined(MODE_SINGLE_SERVO_TEST) … #include "../test/SingleServoTest.cpp"
 * 即它被直接包含进 main.cpp 所在的同一个编译单元（把 platformio.ini 的 build_flags
 * 改成 -DMODE_SINGLE_SERVO_TEST 即生效 —— 本文件里的 PI 也就是 Arduino.h 的宏）。
 * 也可在 Arduino IDE 里与以下源文件一起编译：
 *   include/Config.h、include/ServoManager.h、src/ServoManager.cpp
 *
 * ✅ 编译状态（2026-09-10 实测）：pio run 通过。此前 src/ServoManager.cpp 里的 `const double PI` 与
 *   Arduino.h 的 PI 宏同名相撞（error: expected unqualified-id before numeric constant）曾让整个工程
 *   编译不过，该行已删除；别再往 include 了 <Arduino.h> 的文件作用域写 `const double PI`。
 */

#include <Arduino.h>
#include "ServoManager.h"
#include "Config.h"

// ================= 测试参数 =================
const float SWEEP_MIN = 30.0f;         // 摆动下限（度）
const float SWEEP_MAX = 150.0f;        // 摆动上限（度）
const unsigned long SWEEP_MS = 500;    // 单程时长（毫秒）

// ================= 内部状态 =================
int sweepChannel = -1;                 // -1 = 不在摆动
bool sweepUp = true;
unsigned long sweepStartMs = 0;
float sweepAngle = SWEEP_MIN;

// ================= 小工具 =================
int chToLeg(int ch)      { return ch / 2; }
int chToJoint(int ch)    { return ch % 2; }
const char* jointName(int j) {
    return (j == ServoManager::FEMUR) ? "FEMUR(大腿)" : "TIBIA(小腿)";
}

// ================= 零位偏移小工具 =================
// 命令角度是逻辑角；实际到达角 = 逻辑角 + Config.h ANGLE_OFFSET，再限幅 0~180，
// 与 ServoManager::write 内部规则完全一致（仅供打印显示，写入仍走 write()）。
float servoFinalAngle(int l, int j, float deg) {
    return constrain(deg + robot::ANGLE_OFFSET[l][j], 0.0f, 180.0f);
}

void printHelp() {
    Serial.println("--- 单舵机测试命令 ---");
    Serial.println("  help                : 本帮助");
    Serial.println("  map                 : 通道与(腿,关节)对照表(含零位偏移)");
    Serial.println("  center              : 全部 12 个舵机回 90°");
    Serial.println("  sweep <ch>          : 通道 ch 来回摆动 (ch=0~11)");
    Serial.println("  sweep leg <l> <j>   : (腿 l, 关节 j) 来回摆动");
    Serial.println("  stop                : 停止摆动，保持当前角度");
    Serial.println("  <ch> <angle>        : 通道 ch 转到角度 (0~180)");
    Serial.println("  leg <l> <j> <angle> : (腿 l, 关节 j) 转到角度");
    Serial.println("  (命令角度=逻辑角；实际到达角=角度+Config偏移，写入/center/sweep 会打印)");
}

void printMap() {
    Serial.println("通道 → (腿, 关节, 零位偏移°)  实际到达角 = 命令角 + 偏移：");
    for (int ch = 0; ch < 12; ch++) {
        int l = chToLeg(ch), j = chToJoint(ch);
        Serial.printf("  ch%2d → leg%d %s  偏移 %+6.1f°\n",
                      ch, l, jointName(j), robot::ANGLE_OFFSET[l][j]);
    }
    Serial.println("  (偏移取自 Config.h ANGLE_OFFSET；center/写入/sweep 会打印实际到达角)");
}

void stopSweep();

void centerAll() {
    stopSweep();
    for (int l = 0; l < 6; l++) {
        for (int j = 0; j < 2; j++) {
            ServoManager::write(l, (ServoManager::Joint)j, 90.0f);
        }
    }
    Serial.println("全部舵机 → 90°（逻辑角；带零位偏移的通道实际角如下）");
    for (int ch = 0; ch < 12; ch++) {
        int l = chToLeg(ch), j = chToJoint(ch);
        if (robot::ANGLE_OFFSET[l][j] != 0.0f) {
            Serial.printf("    ch%2d (leg%d %s): 90° + %+.1f° → 实际 %.1f°\n",
                          ch, l, jointName(j), robot::ANGLE_OFFSET[l][j],
                          servoFinalAngle(l, j, 90.0f));
        }
    }
}

void startSweep(int ch) {
    if (ch < 0 || ch > 11) { Serial.println("! 通道范围 0~11"); return; }
    stopSweep();
    sweepChannel = ch;
    sweepUp = true;
    sweepAngle = SWEEP_MIN;
    ServoManager::write(chToLeg(ch), (ServoManager::Joint)chToJoint(ch), sweepAngle);
    sweepStartMs = millis();
    int l = chToLeg(ch), j = chToJoint(ch);
    Serial.printf("sweep ch%d (leg%d %s): 逻辑 %d°~%d° + 偏移 %+.1f° → 实际 %.1f°~%.1f°, %lu ms/程\n",
                  ch, l, jointName(j),
                  (int)SWEEP_MIN, (int)SWEEP_MAX, robot::ANGLE_OFFSET[l][j],
                  servoFinalAngle(l, j, SWEEP_MIN), servoFinalAngle(l, j, SWEEP_MAX),
                  (unsigned long)SWEEP_MS);
}

void stopSweep() {
    if (sweepChannel >= 0) {
        int ch = sweepChannel;
        Serial.printf("stop sweep ch%d @ 逻辑 %.1f° (实际 %.1f°)\n",
                      ch, sweepAngle,
                      servoFinalAngle(chToLeg(ch), chToJoint(ch), sweepAngle));
        sweepChannel = -1;   // PCA9685 持续输出最后脉冲，舵机保持不动
    }
}

void setAngleByChannel(int ch, float deg) {
    if (ch < 0 || ch > 11) { Serial.println("! 通道范围 0~11"); return; }
    stopSweep();
    deg = constrain(deg, 0.0f, 180.0f);
    int l = chToLeg(ch), j = chToJoint(ch);
    ServoManager::write(l, (ServoManager::Joint)j, deg);
    Serial.printf("ch%d (leg%d %s): %.1f° + 偏移 %+.1f° → 实际 %.1f°\n",
                  ch, l, jointName(j), deg,
                  robot::ANGLE_OFFSET[l][j], servoFinalAngle(l, j, deg));
}

void setAngleByLeg(int l, int j, float deg) {
    if (l < 0 || l > 5 || j < 0 || j > 1) { Serial.println("! 腿 0~5，关节 0~1"); return; }
    setAngleByChannel(l * 2 + j, deg);
}

// 解析 sweep 后面的参数：sweep <ch> 或 sweep leg <l> <j>
int parseSweepTarget(const String& rest) {
    String r = rest; r.trim();
    if (r.length() == 0) return -1;

    if (r.startsWith("leg")) {
        int l = -1, j = -1;
        if (sscanf(r.c_str(), "leg %d %d", &l, &j) == 2 && l >= 0 && l <= 5 && j >= 0 && j <= 1) {
            return l * 2 + j;
        }
        return -1;
    }
    int ch = -1;
    if (sscanf(r.c_str(), "%d", &ch) == 1 && ch >= 0 && ch <= 11) return ch;
    return -1;
}

void handleCommand(const String& line) {
    String cmd = line; cmd.trim();
    if (cmd.length() == 0) return;

    int sp = cmd.indexOf(' ');
    String first = (sp < 0) ? cmd : cmd.substring(0, sp);
    String rest  = (sp < 0) ? ""  : cmd.substring(sp + 1);
    first.toLowerCase();
    rest.trim();

    if (first == "help")            { printHelp(); return; }
    if (first == "map")             { printMap();  return; }
    if (first == "center")          { centerAll(); return; }
    if (first == "stop")            { stopSweep(); return; }

    if (first == "sweep") {
        int ch = parseSweepTarget(rest);
        if (ch >= 0) startSweep(ch);
        else Serial.println("! 用法: sweep <ch> 或 sweep leg <l> <j>");
        return;
    }

    if (first == "leg") {
        int l = -1, j = -1; float ang = -1;
        if (sscanf(rest.c_str(), "%d %d %f", &l, &j, &ang) == 3) {
            setAngleByLeg(l, j, ang);
        } else {
            Serial.println("! 用法: leg <腿0~5> <关节0大腿/1小腿> <角度0~180>");
        }
        return;
    }

    // 兜底：<ch> <angle>（纯数字开头）
    int ch = -1; float ang = -1;
    if (sscanf(cmd.c_str(), "%d %f", &ch, &ang) == 2) {
        setAngleByChannel(ch, ang);
        return;
    }

    Serial.println("! 无法识别的命令，输入 help 查看帮助");
}

// ================= 主程序 =================
void setup() {
    Serial.begin(115200);
    delay(200);

    ServoManager::begin();            // 初始化 PCA9685（不驱动任何舵机，上电不动）
    Serial.println("=== 平台单舵机测试 (PCA9685, 12路) ===");
    printMap();
    printHelp();
}

void loop() {
    // 1) 串口命令
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) handleCommand(line);
        delay(5);
    }

    // 2) 摆动推进（若在摆动）
    if (sweepChannel >= 0) {
        unsigned long now = millis();
        if (now - sweepStartMs >= SWEEP_MS) {
            sweepUp = !sweepUp;
            sweepStartMs = now;
        }
        float t = constrain((float)(now - sweepStartMs) / SWEEP_MS, 0.0f, 1.0f);
        sweepAngle = sweepUp
                   ? SWEEP_MIN + (SWEEP_MAX - SWEEP_MIN) * t
                   : SWEEP_MAX - (SWEEP_MAX - SWEEP_MIN) * t;
        ServoManager::write(chToLeg(sweepChannel), (ServoManager::Joint)chToJoint(sweepChannel), sweepAngle);
    }

    delay(10);
}
