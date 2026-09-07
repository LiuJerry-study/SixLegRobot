/* 单腿测试（串口交互式）
 *
 * 目的：验证一条腿的完整功能 —— 逆运动学（足端→关节角）、摆动相/支撑相
 *       轨迹、两个舵机联动。全程通过串口命令控制，不用改代码重新编译。
 *
 * ── 运行方式（PlatformIO）──────────────────────────────
 *   1. platformio.ini 的 build_flags 设为：-DMODE_SINGLE_LEG_TEST
 *   2. Upload 烧录，打开串口监视器（115200 波特率）
 *   3. 发送命令（每行一条，回车结尾）：
 *
 *        help              → 打印命令帮助
 *        cycle             → 连续行走循环（摆动相抬腿前迈 + 支撑相贴地收回），无限循环
 *        one               → 只走一个完整周期，然后自动回站立位
 *        home              → 平滑回到默认站立位
 *        goto <x> <y>      → 足端平滑移动到绝对坐标 (x, y)（cm，原点=髋关节）
 *        lift <cm>         → 设置弧线抬腿高度（默认 3.0）
 *        dur <ms>          → 设置每个动作的时长（默认 600）
 *        pos               → 打印当前足端坐标、IK 关节角与舵机角
 *        stop              → 停止循环，平滑回站立位
 *
 *   修改下方 LEG_ID 可测试其它腿（0~5）；腿与 PCA9685 通道的映射见 Config.h。
 *
 * ── 安全建议 ──────────────────────────────────────────
 *   - 上电时 ServoManager::begin() 会让全部 12 个舵机回 90°，
 *     务必先把机器人架起来（被测腿悬空）再发命令；
 *   - 舵机电源用独立 5V，并与 ESP32 共地（接线见 Config.h 注释）；
 *   - goto 的坐标要落在可达范围（内圆 |8-13|=5cm，外圆 8+13=21cm）内。
 *
 * 本文件为自包含版，也可在 Arduino IDE 里与以下源文件一起编译：
 *   include/Config.h、include/LegKinematics.h、include/ServoManager.h、
 *   include/SingleLeg.h、src/LegKinematics.cpp、src/ServoManager.cpp、src/SingleLeg.cpp
 */

#include <Arduino.h>
#include "SingleLeg.h"
#include "LegKinematics.h"
#include "Config.h"

// ================= 测试参数 =================
const int LEG_ID = 0;                // 要测试的腿（0~5）
const float STAND_HEIGHT = 10.0f;    // 站立高度（cm）
const unsigned long DEFAULT_DURATION_MS = 600;   // 单个动作默认时长
const float DEFAULT_ARC_LIFT = 3.0f;             // 弧线默认抬腿高度（cm）

SingleLeg leg(LEG_ID);

// 一个完整行走周期：摆动相抬腿前迈 → 支撑相贴地收回
const int CYCLE_STEPS = 2;
const LegPrimitive CYCLE[CYCLE_STEPS] = {
    PRIM_ARC_EXTEND,    // 摆动相：抬腿弧线伸出（脚悬空前抬）
    PRIM_LINE_RETRACT   // 支撑相：贴地直线收回（脚掌抓地/推地）
};

// ================= 内部状态 =================
unsigned long primDuration = DEFAULT_DURATION_MS;
bool cycling = false;          // 连续循环模式
bool oneShot = false;          // 只跑一个周期模式
int  stepsDone = 0;            // 本周期已完成的步数
int  cycleStep = 0;            // 当前周期步

const char* primName(LegPrimitive p) {
    switch (p) {
        case PRIM_ARC_EXTEND:   return "ARC_EXTEND(摆动相抬腿前迈)";
        case PRIM_ARC_RETRACT:  return "ARC_RETRACT(摆动相抬腿收回)";
        case PRIM_LINE_EXTEND:  return "LINE_EXTEND(支撑相贴地伸出)";
        case PRIM_LINE_RETRACT: return "LINE_RETRACT(支撑相贴地收回)";
    }
    return "?";
}

void printHelp() {
    Serial.println("--- 单腿测试命令 ---");
    Serial.println("  help             : 本帮助");
    Serial.println("  cycle            : 连续行走循环（无限）");
    Serial.println("  one              : 只走一个周期，然后回站立位");
    Serial.println("  home             : 平滑回默认站立位");
    Serial.println("  goto <x> <y>     : 足端平滑移到绝对坐标 (cm)");
    Serial.println("  lift <cm>        : 设置弧线抬腿高度 (默认 3.0)");
    Serial.println("  dur <ms>         : 设置动作时长 (默认 600)");
    Serial.println("  pos              : 打印足端坐标/关节角");
    Serial.println("  stop             : 停止循环，回站立位");
}

// 平滑回到默认站立位（走直线、不抬腿，避免中途打断时跳变）
void goHome() {
    cycling = false;
    oneShot = false;
    double hx, hy;
    SingleLeg::defaultStandingFoot(hx, hy);
    leg.startGrab(leg.footX(), leg.footY(), (float)hx, (float)hy,
                  0.0f, MODE_SWING, primDuration);
    Serial.println("→ home (回站立位)");
}

void startCycle() {
    oneShot = false;
    cycling = true;
    stepsDone = 0;
    cycleStep = 0;
    leg.startPrimitive(CYCLE[0], primDuration);
    Serial.printf("→ cycle 开始: %s (%lu ms)\n", primName(CYCLE[0]), (unsigned long)primDuration);
}

void startOneShot() {
    cycling = false;
    oneShot = true;
    stepsDone = 0;
    cycleStep = 0;
    leg.startPrimitive(CYCLE[0], primDuration);
    Serial.printf("→ one 开始: %s (%lu ms)\n", primName(CYCLE[0]), (unsigned long)primDuration);
}

void doGoto(float x, float y) {
    cycling = false;
    oneShot = false;
    leg.startGrab(leg.footX(), leg.footY(), x, y, 0.0f, MODE_SWING, primDuration);
    Serial.printf("→ goto (%.1f, %.1f)\n", x, y);
}

void printPos() {
    // 用与 SingleLeg 相同的腿长参数做一次正向显示计算
    LegKinematics kin(robot::LENA, robot::LENB);
    LegAngles ang = kin.ik(leg.footX(), leg.footY());
    float thighDeg = ang.thigh * 180.0 / PI;
    float kneeDeg  = ang.knee  * 180.0 / PI;
    // 与 SingleLeg::writeServosFromRad 相同的舵机映射（仅显示用）
    float servoThigh = 90.0f - thighDeg;
    float servoKnee  = 180.0f - kneeDeg;
    Serial.printf("leg%d foot=(%.2f, %.2f) | thigh=%.1f° knee=%.1f° | servo: thigh=%.1f° knee=%.1f° | moving=%s\n",
                  LEG_ID,
                  leg.footX(), leg.footY(),
                  thighDeg, kneeDeg,
                  servoThigh, servoKnee,
                  leg.isMoving() ? "YES" : "no");
}

void handleCommand(const String& line) {
    String cmd = line; cmd.trim();
    if (cmd.length() == 0) return;

    int sp = cmd.indexOf(' ');
    String first = (sp < 0) ? cmd : cmd.substring(0, sp);
    String rest  = (sp < 0) ? ""  : cmd.substring(sp + 1);
    first.toLowerCase();
    rest.trim();

    if (first == "help")     { printHelp(); return; }
    if (first == "cycle")    { startCycle(); return; }
    if (first == "one")      { startOneShot(); return; }
    if (first == "home")     { goHome(); return; }
    if (first == "stop")     { goHome(); return; }
    if (first == "pos")      { printPos(); return; }

    if (first == "goto") {
        float x = 0, y = 0;
        if (sscanf(rest.c_str(), "%f %f", &x, &y) == 2) doGoto(x, y);
        else Serial.println("! 用法: goto <x> <y>");
        return;
    }
    if (first == "lift") {
        float v = 0;
        if (sscanf(rest.c_str(), "%f", &v) == 1) {
            leg.setArcLift(v);
            Serial.printf("抬腿高度 = %.1f cm\n", v);
        } else Serial.println("! 用法: lift <cm>");
        return;
    }
    if (first == "dur") {
        unsigned long v = 0;
        if (sscanf(rest.c_str(), "%lu", &v) == 1 && v >= 50) {
            primDuration = v;
            Serial.printf("动作时长 = %lu ms\n", (unsigned long)primDuration);
        } else Serial.println("! 用法: dur <ms> (≥50)");
        return;
    }

    Serial.println("! 无法识别的命令，输入 help 查看帮助");
}

// ================= 主程序 =================
void setup() {
    Serial.begin(115200);
    delay(200);

    leg.init(STAND_HEIGHT);          // 初始化（内部会初始化 PCA9685）并回站立位
    leg.setArcLift(DEFAULT_ARC_LIFT);

    Serial.printf("=== 单腿测试: leg%d, 站立高度 %.1fcm ===\n", LEG_ID, STAND_HEIGHT);
    printHelp();
    printPos();
}

void loop() {
    // 1) 串口命令
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) handleCommand(line);
        delay(5);
    }

    // 2) 推进当前动作
    leg.update();

    // 3) 当前动作完成 → 按模式安排下一个
    if (!leg.isMoving()) {
        if (cycling) {
            // 连续循环：永远走下一个周期步
            cycleStep = (cycleStep + 1) % CYCLE_STEPS;
            leg.startPrimitive(CYCLE[cycleStep], primDuration);
        } else if (oneShot) {
            // 单周期：走完 CYCLE_STEPS 步后回站立位
            stepsDone++;
            if (stepsDone >= CYCLE_STEPS) {
                oneShot = false;
                Serial.println("one 周期完成 → home");
                goHome();
            } else {
                cycleStep++;
                leg.startPrimitive(CYCLE[cycleStep], primDuration);
            }
        }
    }

    delay(10);
}
