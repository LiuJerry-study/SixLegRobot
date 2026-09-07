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
 *        center              → 全部 12 个舵机回到 90°（安全位）
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
 * ── 安全建议 ──────────────────────────────────────────
 *   - 上电时 ServoManager::begin() 会让全部 12 个舵机回 90°，务必先把
 *     机器人架起来（腿悬空）或确认周围无障碍，避免舵机堵转/扫到东西；
 *   - 舵机电源用独立 5V，并与 ESP32 共地（接线见 Config.h 注释）；
 *   - 首次测试建议把舵机臂取下，先确认转向与角度范围。
 *
 * 本文件为自包含版，也可在 Arduino IDE 里与以下源文件一起编译：
 *   include/Config.h、include/ServoManager.h、src/ServoManager.cpp
 */

#include <Arduino.h>
#include "ServoManager.h"

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

void printHelp() {
    Serial.println("--- 单舵机测试命令 ---");
    Serial.println("  help                : 本帮助");
    Serial.println("  map                 : 通道与(腿,关节)对照表");
    Serial.println("  center              : 全部 12 个舵机回 90°");
    Serial.println("  sweep <ch>          : 通道 ch 来回摆动 (ch=0~11)");
    Serial.println("  sweep leg <l> <j>   : (腿 l, 关节 j) 来回摆动");
    Serial.println("  stop                : 停止摆动，保持当前角度");
    Serial.println("  <ch> <angle>        : 通道 ch 转到角度 (0~180)");
    Serial.println("  leg <l> <j> <angle> : (腿 l, 关节 j) 转到角度");
}

void printMap() {
    Serial.println("通道 → (腿, 关节)：");
    for (int ch = 0; ch < 12; ch++) {
        Serial.printf("  ch%2d → leg%d %s\n", ch, chToLeg(ch), jointName(chToJoint(ch)));
    }
}

void stopSweep();

void centerAll() {
    stopSweep();
    for (int l = 0; l < 6; l++) {
        for (int j = 0; j < 2; j++) {
            ServoManager::write(l, (ServoManager::Joint)j, 90.0f);
        }
    }
    Serial.println("全部舵机 → 90°");
}

void startSweep(int ch) {
    if (ch < 0 || ch > 11) { Serial.println("! 通道范围 0~11"); return; }
    stopSweep();
    sweepChannel = ch;
    sweepUp = true;
    sweepAngle = SWEEP_MIN;
    ServoManager::write(chToLeg(ch), (ServoManager::Joint)chToJoint(ch), sweepAngle);
    sweepStartMs = millis();
    Serial.printf("sweep ch%d (leg%d %s): %d°~%d°, %lu ms/程\n",
                  ch, chToLeg(ch), jointName(chToJoint(ch)),
                  (int)SWEEP_MIN, (int)SWEEP_MAX, (unsigned long)SWEEP_MS);
}

void stopSweep() {
    if (sweepChannel >= 0) {
        Serial.printf("stop sweep ch%d @ %.1f°\n", sweepChannel, sweepAngle);
        sweepChannel = -1;   // PCA9685 持续输出最后脉冲，舵机保持不动
    }
}

void setAngleByChannel(int ch, float deg) {
    if (ch < 0 || ch > 11) { Serial.println("! 通道范围 0~11"); return; }
    stopSweep();
    deg = constrain(deg, 0.0f, 180.0f);
    ServoManager::write(chToLeg(ch), (ServoManager::Joint)chToJoint(ch), deg);
    Serial.printf("ch%d (leg%d %s) -> %.1f°\n",
                  ch, chToLeg(ch), jointName(chToJoint(ch)), deg);
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

    ServoManager::begin();            // 初始化 PCA9685，全部通道回 90°
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
