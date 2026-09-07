/* 六腿同步测试（串口交互式）
 *
 * 目的：把单腿测试的动作扩展到 6 条腿同步执行 —— 所有腿同一时刻做同一个动作，
 *       用于验证：
 *         - 6 条腿 / 12 个舵机的装配与运动学一致性（对称性检查）；
 *         - 12 路舵机同时动作时的供电能力（电流峰值）；
 *         - 整机基础运动是否正确。
 *
 * ── 运行方式（PlatformIO）──────────────────────────────
 *   1. platformio.ini 的 build_flags 设为：-DMODE_SIX_LEGS_SYNC
 *   2. Upload 烧录，打开串口监视器（115200 波特率）
 *   3. 发送命令（每行一条，回车结尾，全部作用于 6 条腿）：
 *
 *        help              → 打印命令帮助
 *        cycle             → 6 腿同步连续行走循环（摆动相抬腿前迈 + 支撑相贴地收回）
 *        one               → 只走一个完整周期，然后自动回站立位
 *        home              → 平滑回默认站立位
 *        goto <x> <y>      → 6 腿足端同步移动到绝对坐标 (x, y)（cm，各腿自身坐标系）
 *        lift <cm>         → 设置弧线抬腿高度（默认 3.0）
 *        dur <ms>          → 设置每个动作的时长（默认 300，越小越快；过快舵机跟不上）
 *        0 <角度>          → 6 条大腿(FEMUR)舵机同步转到角度 (0~180)，如 0 90
 *        1 <角度>          → 6 条小腿(TIBIA)舵机同步转到角度 (0~180)，如 1 45
 *        pos               → 打印 6 条腿的足端坐标与状态
 *        stop              → 停止循环，平滑回站立位
 *
 *   舵机直控（0 x / 1 x）说明：
 *     - 发送后进入"直控模式"，轨迹运动（cycle/goto 等）会先被冻结，防止打架；
 *     - 直控模式中再发任意运动命令，会自动先瞬间回默认站位并同步内部状态
 *       （此时打印会提示），之后正常运动；
 *     - 注意：直控只发角度、不动内部坐标，故直控期间 pos 打印的坐标不可信。
 *
 *   同步方式：所有腿同时 startPrimitive，每步等 6 条腿全部完成后才一起进入
 *   下一步（以最后完成的那条腿为准），保证全程严格同步、不互相拉扯。
 *
 * ── 安全建议 ──────────────────────────────────────────
 *   - 上电时 ServoManager::begin() 会让全部 12 个舵机回 90°，务必架起机器人；
 *   - 12 个舵机同时动作电流很大（峰值可达数安培），舵机电源务必用独立
 *     大电流 5V（如 5A 以上开关电源/电池+UBEC），且与 ESP32 共地；
 *   - 先跑"单舵机/单腿"测试确认每路都正常，再跑本测试，便于定位问题。
 *
 * 本文件为自包含版，也可在 Arduino IDE 里与以下源文件一起编译：
 *   include/Config.h、include/LegKinematics.h、include/ServoManager.h、
 *   include/SingleLeg.h、src/LegKinematics.cpp、src/ServoManager.cpp、src/SingleLeg.cpp
 */

#include <Arduino.h>
#include "SingleLeg.h"
#include "ServoManager.h"
#include "LegKinematics.h"
#include "Config.h"

// ================= 6 条腿 =================
SingleLeg legs[6] = {
    SingleLeg(0), SingleLeg(1), SingleLeg(2),
    SingleLeg(3), SingleLeg(4), SingleLeg(5)
};

// ================= 测试参数 =================
const float STAND_HEIGHT = 10.0f;             // 站立高度（cm）
const unsigned long DEFAULT_DURATION_MS = 300; // 单个动作默认时长（越小越快，太快舵机会跟不上）
const float DEFAULT_ARC_LIFT = 3.0f;           // 弧线默认抬腿高度（cm）

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
bool rawDirty = false;         // 舵机直控模式标记（发过 0 x / 1 x 后为 true）

const char* primName(LegPrimitive p) {
    switch (p) {
        case PRIM_ARC_EXTEND:   return "ARC_EXTEND(摆动相抬腿前迈)";
        case PRIM_ARC_RETRACT:  return "ARC_RETRACT(摆动相抬腿收回)";
        case PRIM_LINE_EXTEND:  return "LINE_EXTEND(支撑相贴地伸出)";
        case PRIM_LINE_RETRACT: return "LINE_RETRACT(支撑相贴地收回)";
    }
    return "?";
}

// 让 6 条腿同时启动同一个原语 —— 同步的关键入口
void startPrimitiveAll(LegPrimitive p, unsigned long dur) {
    for (int i = 0; i < 6; i++) legs[i].startPrimitive(p, dur);
}

// 6 条腿是否全部停稳
bool allLegsStopped() {
    for (int i = 0; i < 6; i++) {
        if (legs[i].isMoving()) return false;
    }
    return true;
}

void printHelp() {
    Serial.println("--- 六腿同步测试命令 ---");
    Serial.println("  help             : 本帮助");
    Serial.println("  cycle            : 6腿同步连续行走循环（无限）");
    Serial.println("  one              : 只走一个周期，然后回站立位");
    Serial.println("  home             : 平滑回默认站立位");
    Serial.println("  goto <x> <y>     : 6腿足端同步移到绝对坐标 (cm)");
    Serial.println("  lift <cm>        : 设置弧线抬腿高度 (默认 3.0)");
    Serial.println("  dur <ms>         : 设置动作时长 (默认 300，越小越快)");
    Serial.println("  0 <角度>         : 6条大腿(FEMUR)舵机同步转到角度 (0~180)");
    Serial.println("  1 <角度>         : 6条小腿(TIBIA)舵机同步转到角度 (0~180)");
    Serial.println("  pos              : 打印6腿足端坐标/状态");
    Serial.println("  stop             : 停止循环，回站立位");
}

// ========== 舵机直控（0 x / 1 x） ==========

// 直控命令：所有腿的同一关节舵机转到同一角度（0=大腿FEMUR，1=小腿TIBIA）
void servoAllByJoint(int joint, float deg) {
    cycling = false;
    oneShot = false;

    // 若腿正在轨迹运动中，先把内部状态冻结在当前位置（取消运动），
    // 否则 update() 会在下一帧用插值角度覆盖刚写入的直控角度
    for (int i = 0; i < 6; i++) {
        legs[i].setFootPositionImmediate(legs[i].footX(), legs[i].footY());
    }

    for (int i = 0; i < 6; i++) {
        ServoManager::write(i, (ServoManager::Joint)joint, deg);
    }
    rawDirty = true;
    Serial.printf("直控: 6条%s舵机 → %.1f° (此后发运动命令会自动先回默认站位)\n",
                  (joint == ServoManager::FEMUR) ? "大腿(FEMUR)" : "小腿(TIBIA)", deg);
}

// 退出直控模式：6 腿瞬间回默认站位并同步内部坐标（供后续轨迹命令使用）
void takeControlBack() {
    if (!rawDirty) return;
    rawDirty = false;
    double hx, hy;
    SingleLeg::defaultStandingFoot(hx, hy);
    for (int i = 0; i < 6; i++) {
        legs[i].setFootPositionImmediate((float)hx, (float)hy);
    }
    Serial.println("↑ 直控结束：6腿瞬间回默认站位，内部状态已同步");
}

// 平滑回默认站立位（走直线、不抬腿，避免中途打断时跳变）
void goHome() {
    cycling = false;
    oneShot = false;
    if (rawDirty) { takeControlBack(); return; }   // 直控中 → 已瞬间回位
    double hx, hy;
    SingleLeg::defaultStandingFoot(hx, hy);
    for (int i = 0; i < 6; i++) {
        legs[i].startGrab(legs[i].footX(), legs[i].footY(),
                          (float)hx, (float)hy, 0.0f, MODE_SWING, primDuration);
    }
    Serial.println("→ 6腿 home (回站立位)");
}

void startCycle() {
    oneShot = false;
    cycling = true;
    stepsDone = 0;
    cycleStep = 0;
    takeControlBack();           // 直控模式先回站位，从站位开始走
    startPrimitiveAll(CYCLE[0], primDuration);
    Serial.printf("→ 6腿同步 cycle 开始: %s (%lu ms)\n",
                  primName(CYCLE[0]), (unsigned long)primDuration);
}

void startOneShot() {
    cycling = false;
    oneShot = true;
    stepsDone = 0;
    cycleStep = 0;
    takeControlBack();           // 直控模式先回站位，从站位开始走
    startPrimitiveAll(CYCLE[0], primDuration);
    Serial.printf("→ 6腿同步 one 开始: %s (%lu ms)\n",
                  primName(CYCLE[0]), (unsigned long)primDuration);
}

void doGoto(float x, float y) {
    cycling = false;
    oneShot = false;
    takeControlBack();           // 直控模式先回站位再平滑 goto
    for (int i = 0; i < 6; i++) {
        legs[i].startGrab(legs[i].footX(), legs[i].footY(),
                          x, y, 0.0f, MODE_SWING, primDuration);
    }
    Serial.printf("→ 6腿同步 goto (%.1f, %.1f)\n", x, y);
}

void printPos() {
    for (int i = 0; i < 6; i++) {
        Serial.printf("  leg%d foot=(%.2f, %.2f) %s\n",
                      i, legs[i].footX(), legs[i].footY(),
                      legs[i].isMoving() ? "[moving]" : "[stop]");
    }
    if (rawDirty) {
        Serial.println("  (注: 舵机直控模式中，以上坐标不代表实际足端位置)");
    }
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
            for (int i = 0; i < 6; i++) legs[i].setArcLift(v);
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

    // 舵机直控：0 <角度> → 6条大腿；1 <角度> → 6条小腿
    if (first == "0" || first == "1") {
        int joint = first.toInt();
        float ang = -1;
        if (sscanf(rest.c_str(), "%f", &ang) == 1) {
            servoAllByJoint(joint, constrain(ang, 0.0f, 180.0f));
        } else {
            Serial.println("! 用法: 0 <角度0~180>(大腿)  或  1 <角度0~180>(小腿)");
        }
        return;
    }

    Serial.println("! 无法识别的命令，输入 help 查看帮助");
}

// ================= 主程序 =================
void setup() {
    Serial.begin(115200);
    delay(200);

    for (int i = 0; i < 6; i++) {
        legs[i].init(STAND_HEIGHT);      // 初始化（内部会初始化 PCA9685）并回站立位
        legs[i].setArcLift(DEFAULT_ARC_LIFT);
    }

    Serial.printf("=== 六腿同步测试: 站立高度 %.1fcm ===\n", STAND_HEIGHT);
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

    // 2) 推进全部 6 条腿（每帧都 update）
    for (int i = 0; i < 6; i++) legs[i].update();

    // 3) 全部停稳后，按模式安排下一步（以最慢的腿为准，保证严格同步）
    if (allLegsStopped()) {
        if (cycling) {
            cycleStep = (cycleStep + 1) % CYCLE_STEPS;
            startPrimitiveAll(CYCLE[cycleStep], primDuration);
        } else if (oneShot) {
            stepsDone++;
            if (stepsDone >= CYCLE_STEPS) {
                oneShot = false;
                Serial.println("one 周期完成 → home");
                goHome();
            } else {
                cycleStep++;
                startPrimitiveAll(CYCLE[cycleStep], primDuration);
            }
        }
    }

    delay(10);
}
