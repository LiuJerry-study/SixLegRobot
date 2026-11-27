/* 六足主控台：串口方向行走、选腿和单舵机控制。
 * 步态内核走 20 ms 控制周期，网页可用 snapshot 请求完整命令帧。
 * 实物首次测试请先架空，校对腿序、零位和供电。
 */

#include <Arduino.h>
#include "SingleLeg.h"
#include "ServoManager.h"
#include "LegKinematics.h"
#include "MotionFrame.h"
#include "Config.h"
#include <math.h>

// ============================================================
// 0. 上电默认值 / 可调常量
// ============================================================
static const char          BOOT_SELECTION[] = "all";   // 上电选中的腿（all / 0 / 0 1 / 0-3 …）
static const float         STAND_HEIGHT_CM  = -robot::STAND_Y;
static const float         DEFAULT_LIFT_CM  = 3.0f;    // 抬腿高度（cm）
static const unsigned long DEFAULT_DUR_MS   = 600;     // 动作时长 / 每拍时长（ms）

// ============================================================
// 1. 全局状态
// ============================================================
// 硬件上的 6 条腿（0~5 → PCA9685 ch = leg*2 + joint，见 Config.h::SERVO_PINS）
static SingleLeg gLegs[6] = {
    SingleLeg(0), SingleLeg(1), SingleLeg(2),
    SingleLeg(3), SingleLeg(4), SingleLeg(5)
};
static bool gInited[6] = { false, false, false, false, false, false };  // 是否已被 init() 过（未上电的腿坐标无意义）

// 选中集合（B 组命令全部只作用于选中的腿）
static bool gSel[6]     = { false, false, false, false, false, false };
static int  gSelList[6] = { 0, 0, 0, 0, 0, 0 };
static int  gSelCount   = 0;

static unsigned long gDur  = DEFAULT_DUR_MS;
static float         gLift = DEFAULT_LIFT_CM;

// 舵机直控（绕过轨迹）：gRaw = 有直控发生过（此时内部足端坐标不再代表实际姿态）
static bool  gRaw           = false;
static float gRawDeg[6][2]  = { { 0, 0 } };            // 直控时下发的逻辑角（大腿/小腿）

// 方向（腿序制式 + 方向档 0~5 → 0°/60°/…/300°）
static int gAzSign = 1;    // +1 = 俯视逆时针（leg0 正前、leg1 左前…）；−1 = 镜像
static int gDir    = 0;

static motion::Walk gGait;
static motion::Settings gSettings;
static bool gWalk = false;
static bool gWalkCont = false;
static bool gLeased = false;
static uint32_t gLeaseAt = 0;
static constexpr uint32_t LEASE_MS = 750;
static bool gStreaming = false;
static uint32_t gFrameSequence = 0;
static uint32_t gFrameDropped = 0;
static uint32_t gMaxLateMs = 0;

// 选中腿的动作模式（同一时刻只有一个）
static bool gCycle = false, gOne = false, gEach = false, gEachPending = false;
static int  gStepIdx = 0, gStepsDone = 0, gEachIdx = 0, gEachPhase = 0;

// 一个"行走周期"= 摆动相抬腿前迈 + 支撑相贴地收回（与其它测试一致）
static const LegPrimitive WALK_CYCLE[2] = { PRIM_ARC_EXTEND, PRIM_LINE_RETRACT };
static const int          WALK_STEPS    = 2;

// 相位表（按"槽位"0~5 的原语；数字=腿号，a=弧线张 b=弧线收 c=直线张 d=直线收）
// 与 TripodGait.cpp 完全对应：slot i 由物理腿 (i + sqo) % 6 来跑
static const char SLOT_A[7] = "dacbca";   // 第一拍：0d 1a 2c 3b 4c 5a
static const char SLOT_B[7] = "adbcbd";   // 第二拍：0a 1d 2b 3c 4b 5d

// ============================================================
// 2. 前置声明（定义在后面，互相调用用）
// ============================================================
static void printState();
static void emitState();
static void printHelp();
static void printDirTable();
static void printMap();
static void printPos();
static void printPhase();
static void verifyDirections();
static void applySel(const String& spec);
static void cancelWalk();
static void yieldAll();                 // 停步态 + 停选中腿动作 + 退出直控
static void walkStart(bool cont, int steps);
static void startCycle();
static void startOne();
static void startEach();
static void stepEach();
static void stepSelected();
static void goHomeSel();
static void goHomeAll();
static void doGoto(float x, float y);
static void rawSet(int leg, int joint, float deg);
static void setDirIndex(int k, bool startIfIdle);
static bool haveSel();
static void ensureInited(int leg);

// ============================================================
// 3. 小工具
// ============================================================
static bool isAllDigits(const String& s) {
    if (s.length() == 0) return false;
    for (unsigned int i = 0; i < s.length(); i++) {
        if (s[i] < '0' || s[i] > '9') return false;
    }
    return true;
}

static float norm360(float a) {
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

static float norm180(float a) {
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a - 180.0f;
}

static const char* jointName(int j) {
    return (j == ServoManager::FEMUR) ? "大腿(FEMUR)" : "小腿(TIBIA)";
}

static const char* dirName(int k) {
    static const char* N[6] = { "前", "左前", "左后", "后", "右后", "右前" };
    return N[((k % 6) + 6) % 6];
}

static const char* dirKey(int k) {
    static const char* K[6] = { "f", "fl", "bl", "b", "br", "fr" };
    return K[((k % 6) + 6) % 6];
}

// 第 id 条腿的方位角（度，0 = 机头方向）
static float legAzimuthDeg(int id) {
    return norm360(60.0f * (float)id * (float)gAzSign);
}

// 方向档 → 三角步态的 SequentialOffset（实物腿序镜像时取 (6−k)%6）
static int gaitSqo() {
    return (gAzSign > 0) ? (gDir % 6) : ((6 - gDir % 6) % 6);
}

// 某物理腿此刻占的槽位（与 TripodGait::legIndices 互逆）
static int slotOfLeg(int id) {
    int sqo = gaitSqo();
    return ((id - sqo) % 6 + 6) % 6;
}

// 选中集合 → 紧凑字符串（给 @STATE 用）
static String selSpec() {
    if (gSelCount == 0) return String("none");
    if (gSelCount == 6) return String("all");
    String s;
    for (int i = 0; i < gSelCount; i++) {
        if (i) s += ',';
        s += String(gSelList[i]);
    }
    return s;
}

// ============================================================
// 4. 选中集合（解析 / 应用 / 打印）
// ============================================================
static void selAdd(int out[6], int& n, int id) {
    if (id < 0 || id > 5) return;
    for (int i = 0; i < n; i++) if (out[i] == id) return;   // 去重
    out[n++] = id;
}

static void selAddRange(int out[6], int& n, int a, int b) {
    if (a > b) { int t = a; a = b; b = t; }
    if (a < 0) a = 0;
    if (b > 5) b = 5;
    for (int i = a; i <= b; i++) selAdd(out, n, i);
}

// 解析 "all" / "none" / "0 1 2" / "0-3"；返回选中数量，−1 = 语法错误
static int parseSel(const String& spec, int out[6]) {
    int n = 0;
    String s = spec; s.trim();
    if (s.length() == 0) return 0;

    int pos = 0;
    while (true) {
        int sp = s.indexOf(' ', pos);
        String tok = (sp < 0) ? s.substring(pos) : s.substring(pos, sp);
        tok.trim();
        if (tok.length() > 0) {
            if (tok.equalsIgnoreCase("all")) {
                for (int i = 0; i < 6; i++) selAdd(out, n, i);
            } else if (tok.equalsIgnoreCase("none")) {
                n = 0;
            } else {
                int dash = tok.indexOf('-');
                if (dash > 0 && isAllDigits(tok.substring(0, dash)) && isAllDigits(tok.substring(dash + 1))) {
                    selAddRange(out, n, tok.substring(0, dash).toInt(), tok.substring(dash + 1).toInt());
                } else if (isAllDigits(tok)) {
                    int id = tok.toInt();
                    if (id > 5) return -1;              // 腿号只有 0~5
                    selAdd(out, n, id);
                } else {
                    return -1;                          // 非法 token
                }
            }
        }
        if (sp < 0) break;
        pos = sp + 1;
    }
    return n;
}

static void printSel() {
    Serial.print("当前选中: ");
    if (gSelCount == 0) { Serial.println("(无 — 用 legs 选择，如 legs 0 1)"); return; }
    for (int i = 0; i < gSelCount; i++) Serial.printf("leg%d ", gSelList[i]);
    Serial.printf("(共 %d 条)\n", gSelCount);
}

static bool haveSel() {
    if (gSelCount > 0) return true;
    Serial.println("! 当前未选中任何腿：先用 legs 选择，如 legs all / legs 0 / legs 0 1 2");
    return false;
}

// 切换腿集合：停掉正在跑的东西 → 被移出的腿回位 → 新入选的腿首次上电回站位
static void applySel(const String& spec) {
    int nl[6];
    int n = parseSel(spec, nl);
    if (n < 0) {
        Serial.println("! legs 语法: legs all | legs none | legs <n n ...> | legs <a-b>   例: legs 0 / legs 0 1 2 / legs 0-3");
        return;
    }
    yieldAll();                                  // 停步态/动作，直控退出（6 腿瞬间回站位）

    // 被移出且仍在动的腿：平滑回位
    for (int i = 0; i < 6; i++) {
        bool still = false;
        for (int k = 0; k < n; k++) if (nl[k] == i) { still = true; break; }
        if (gSel[i] && !still && gInited[i] && gLegs[i].isMoving()) {
            double hx, hy; SingleLeg::defaultStandingFoot(hx, hy);
            gLegs[i].startGrab(gLegs[i].footX(), gLegs[i].footY(), (float)hx, (float)hy, 0.0f, MODE_SWING, gDur);
        }
    }

    // 写入选集（升序）
    for (int i = 0; i < 6; i++) gSel[i] = false;
    gSelCount = n;
    for (int k = 0; k < n; k++) { gSelList[k] = nl[k]; gSel[nl[k]] = true; }
    for (int i = 0; i < gSelCount - 1; i++)
        for (int j = i + 1; j < gSelCount; j++)
            if (gSelList[j] < gSelList[i]) { int t = gSelList[i]; gSelList[i] = gSelList[j]; gSelList[j] = t; }

    // 新入选且从未上电的腿：首次上电回默认站位（会写舵机）
    for (int k = 0; k < gSelCount; k++) {
        int id = gSelList[k];
        if (!gInited[id]) {
            Serial.printf("↑ leg%d 首次上电：回默认站立位\n", id);
            ensureInited(id);
        }
    }
    printSel();
    printState();
}

// ============================================================
// 5. 一致性：轨迹 ↔ 直控（两边不能同时说话）
// ============================================================
static void ensureInited(int leg) {
    if (leg < 0 || leg > 5 || gInited[leg]) return;
    gLegs[leg].init(STAND_HEIGHT_CM);            // 内部会自动 ServoManager::begin()（幂等）
    gLegs[leg].setArcLift(gLift);
    gInited[leg] = true;
}

static void ensureAllInited() {
    for (int i = 0; i < 6; i++) ensureInited(i);
}

// 平滑回默认站位（贴地直线、不抬腿，中途被打断也不跳变）
static void homeLegSmooth(int leg) {
    ensureInited(leg);
    double hx, hy;
    SingleLeg::defaultStandingFoot(hx, hy);
    const motion::Path path{{gLegs[leg].footX(), gLegs[leg].footY()}, {hx, hy}, 0, 1, false, false};
    double peak = 0;
    if (!motion::feasible(leg, path, 0, peak)) {
        gLegs[leg].trackFootPosition(path.from.x, path.from.y);
        Serial.printf("! leg%d 回位轨迹超出限位，保持最后指令\n", leg);
        return;
    }
    const unsigned long duration = max(gDur, motion::durationForPeak(peak));
    gLegs[leg].startGrab(gLegs[leg].footX(), gLegs[leg].footY(),
                         (float)hx, (float)hy, 0.0f, MODE_SWING, duration);
}

static void goHomeAll() {
    for (int i = 0; i < 6; i++) homeLegSmooth(i);
}

static void goHomeSel() {
    if (!haveSel()) return;
    for (int i = 0; i < gSelCount; i++) homeLegSmooth(gSelList[i]);
    Serial.printf("→ %d 条腿平滑回站立位\n", gSelCount);
    printState();
}

static bool selAllStopped() {
    for (int i = 0; i < gSelCount; i++) if (gLegs[gSelList[i]].isMoving()) return false;
    return true;
}

// 用最后下发的命令恢复几何状态。没有反馈的通道由首次初始化建立基准。
static void takeControlBackAll() {
    if (!gRaw) return;
    gRaw = false;
    for (int i = 0; i < 6; ++i) {
        const auto* f = ServoManager::lastCommand(robot::SERVO_PINS[i][0]);
        const auto* t = ServoManager::lastCommand(robot::SERVO_PINS[i][1]);
        if (f && t) {
            const FootPos foot = motion::footFromCommands(i, f->applied, t->applied);
            gLegs[i].trackFootPosition(foot.x, foot.y);
        } else {
            gInited[i] = false;
        }
    }
}

static void cancelWalk() {
    if (!gWalk) return;
    gGait.cancel();
    gWalk = gWalkCont = gLeased = false;
}

// 任何"开新动作"前的统一入口：停步态 → 停选中腿动作 → 退出直控
static void yieldAll() {
    cancelWalk();
    gCycle = gOne = gEach = gEachPending = false;
    gStepIdx = gStepsDone = gEachIdx = gEachPhase = 0;
    takeControlBackAll();
}

// 直控前的让位：停步态 + 停选中腿动作（**不**退出直控状态，否则刚发的直控角度会被回位覆盖）
static void yieldMotionsForRaw() {
    cancelWalk();
    gCycle = gOne = gEach = gEachPending = false;
    gStepIdx = gStepsDone = gEachIdx = gEachPhase = 0;
}

// ============================================================
// 6. 方向行走（三角步态）
// ============================================================
// 相位表 → 字符串（按腿号 0..5 排，便于与文档中的经典表对照）
static String phaseStr(bool firstTable) {
    const char* tab = firstTable ? SLOT_A : SLOT_B;
    String s;
    for (int leg = 0; leg < 6; leg++) {
        s += String(leg);
        s += tab[slotOfLeg(leg)];
        if (leg < 5) s += ' ';
    }
    return s;
}

static void walkStart(bool cont, int steps) {
    yieldAll();
    ensureAllInited();
    FootPos feet[6];
    for (int i = 0; i < 6; ++i) {
        feet[i] = {gLegs[i].footX(), gLegs[i].footY()};
        gLegs[i].trackFootPosition(feet[i].x, feet[i].y);
    }
    gGait.start(feet, gSettings, cont ? 0 : (steps < 1 ? 1 : steps));
    gWalk = true;
    gWalkCont = cont;
    gLeased = false;
    printState();
}

static bool stepWalk(uint32_t now) {
    if (!gWalk) return false;
    if (gLeased && now - gLeaseAt >= LEASE_MS) {
        gLeased = false;
        gGait.stop();
    }
    const auto priorStage = gGait.stage();
    const unsigned priorBeat = gGait.completed();
    bool busFault = ServoManager::takeWriteFailure();
    if (gGait.tick(now)) {
        for (int i = 0; i < 6 && !busFault; ++i) {
            const FootPos foot = gGait.foot(i);
            gLegs[i].setFootPositionImmediate(foot.x, foot.y);
            busFault = ServoManager::takeWriteFailure();
        }
    }
    if (busFault) {
        gGait.hold(priorBeat);
        gRaw = true;
        takeControlBackAll();
    }
    if (!gGait.active()) {
        gWalk = gWalkCont = gLeased = false;
        if (busFault) Serial.println("! PCA9685 写入失败，步态已停止");
        else if (gGait.stage() == motion::Stage::Hold) Serial.println("! 轨迹超过校准行程，保持最后指令");
    }
    const bool changed = priorStage != gGait.stage() || priorBeat != gGait.completed();
    if (changed) emitState();
    return changed;
}

static void setDirection(float angle, bool startIfIdle) {
    angle = norm360(angle);
    const int nearest = int(lroundf(angle / robot::LEG_SPACING_DEG)) % 6;
    if (gSettings.gait == motion::Gait::Legacy) angle = nearest * robot::LEG_SPACING_DEG;
    const bool changed = angle != gSettings.direction || gSettings.mirror != gAzSign;
    gDir = nearest;
    gSettings.direction = angle;
    gSettings.mirror = gAzSign;
    if (gWalk && changed) gGait.configure(gSettings);
    if (!gWalk && startIfIdle) walkStart(true, 0);
    else printState();
}

static void setDirIndex(int k, bool startIfIdle) {
    setDirection(k * robot::LEG_SPACING_DEG, startIfIdle);
}

// 方向自检（纯数学，不需要硬件）：sqo = k ⇒ 两拍净推力都应指向 60°×k
static void verifyDirections() {
    Serial.println("--- 方向自检：SequentialOffset → 净推力方向（纯数学，无需硬件）---");
    Serial.println("  规则: d(支撑相收)→推向该腿朝外 +ê   c(支撑相伸)→推向 −ê   a/b(摆动相)→不产生推力");
    float worst = 0.0f;
    int   worstSqo = 0, worstPh = 0;

    for (int sqo = 0; sqo < 6; sqo++) {
        float dirDeg[2] = { 0.0f, 0.0f };
        float mag[2]    = { 0.0f, 0.0f };
        for (int ph = 0; ph < 2; ph++) {
            const char* tab = (ph == 0) ? SLOT_A : SLOT_B;
            float sx = 0.0f, sy = 0.0f;
            for (int slot = 0; slot < 6; slot++) {
                float sgn = (tab[slot] == 'd') ? 1.0f : ((tab[slot] == 'c') ? -1.0f : 0.0f);
                if (sgn == 0.0f) continue;
                int leg = (slot + sqo) % 6;
                float a = legAzimuthDeg(leg) * DEG_TO_RAD;
                sx += sgn * cosf(a);
                sy += sgn * sinf(a);
            }
            mag[ph]    = sqrtf(sx * sx + sy * sy);
            dirDeg[ph] = (mag[ph] < 1e-6f) ? 0.0f : atan2f(sy, sx) * RAD_TO_DEG;
            float expect = 60.0f * (float)sqo * (float)gAzSign;
            float err = norm180(dirDeg[ph] - expect);
            if (fabsf(err) > fabsf(worst)) { worst = err; worstSqo = sqo; worstPh = ph; }
        }
        Serial.printf("  sqo=%d → 期望 %3d°(%-4s) | 第一拍 %6.1f°(净%.2f×行程)  第二拍 %6.1f°(净%.2f×行程)\n",
                      sqo, 60 * sqo, dirName(sqo),
                      norm360(dirDeg[0]), mag[0], norm360(dirDeg[1]), mag[1]);
    }
    Serial.printf("  最大方向偏差: %+.2f°（sqo=%d 的第%d拍）\n", worst, worstSqo, worstPh + 1);
    Serial.printf("  当前制式(%s)下 6 个方向用的 sqo:\n",
                  (gAzSign > 0) ? "逆时针 leg1=左前" : "顺时针镜像 leg1=右前");
    for (int k = 0; k < 6; k++) {
        int sqo = (gAzSign > 0) ? k : ((6 - k) % 6);
        Serial.printf("    %-2s %3d° %s → sqo=%d\n", dirKey(k), 60 * k, dirName(k), sqo);
    }
    Serial.println("  ⚠ 真机方向不对时先查: ①腿序/方位角是否镜像（发 mirror）②该腿舵机是否装反");
}

// ============================================================
// 7. 选中腿的动作（cycle / one / each / goto）
// ============================================================
static void startPrimitiveSel(LegPrimitive p, unsigned long dur) {
    for (int i = 0; i < gSelCount; i++) gLegs[gSelList[i]].startPrimitive(p, dur);
}

static void startCycle() {
    if (!haveSel()) return;
    yieldAll();
    gCycle = true; gStepIdx = 0; gStepsDone = 0;
    startPrimitiveSel(WALK_CYCLE[0], gDur);
    Serial.printf("→ %d 条腿同步连续行走循环（每段 %lu ms）\n", gSelCount, (unsigned long)gDur);
    printState();
}

static void startOne() {
    if (!haveSel()) return;
    yieldAll();
    gOne = true; gStepIdx = 0; gStepsDone = 0;
    startPrimitiveSel(WALK_CYCLE[0], gDur);
    Serial.printf("→ %d 条腿走一个完整周期（每段 %lu ms）\n", gSelCount, (unsigned long)gDur);
    printState();
}

static void startEach() {
    if (!haveSel()) return;
    yieldAll();
    if (gSelCount == 1) Serial.println("（只选中 1 条腿：each 与 one 等效）");
    gEach = true; gEachPending = true; gEachIdx = 0; gEachPhase = 0;
    for (int i = 0; i < gSelCount; i++) homeLegSmooth(gSelList[i]);   // 先全部回位，回位后逐条测
    Serial.printf("→ 逐腿巡检开始：依次测 %d 条腿（每条走一个完整周期再回位）\n", gSelCount);
    printState();
}

static void stepEach() {
    if (!gEach) return;
    if (gEachPending) {
        if (!selAllStopped()) return;            // 等全部选中腿回位完成
        gEachPending = false;
    }
    if (gEachIdx >= gSelCount) {
        gEach = false;
        Serial.println("逐腿巡检完成（全部回站立位）");
        printState();
        return;
    }

    int id = gSelList[gEachIdx];
    if (gLegs[id].isMoving()) return;            // 等本阶段走完

    switch (gEachPhase) {
        case 0:
            gLegs[id].startPrimitive(WALK_CYCLE[0], gDur);
            gEachPhase = 1;
            Serial.printf("  leg%d: 摆动相（抬腿前迈）%lu ms\n", id, (unsigned long)gDur);
            break;
        case 1:
            gLegs[id].startPrimitive(WALK_CYCLE[1], gDur);
            gEachPhase = 2;
            Serial.printf("  leg%d: 支撑相（贴地收回）%lu ms\n", id, (unsigned long)gDur);
            break;
        case 2:
            homeLegSmooth(id);
            gEachPhase = 3;
            Serial.printf("  leg%d: 回站立位\n", id);
            break;
        default:
            gEachIdx++;
            gEachPhase = 0;
            if (gEachIdx < gSelCount) Serial.printf("-- 下一条: leg%d --\n", gSelList[gEachIdx]);
            break;
    }
}

// 选中腿的模式推进（loop 每帧调用）：以"最慢的腿"为准，保证严格同步
static void stepSelected() {
    if (!gCycle && !gOne) return;
    if (!selAllStopped()) return;

    if (gCycle) {
        gStepIdx = (gStepIdx + 1) % WALK_STEPS;
        startPrimitiveSel(WALK_CYCLE[gStepIdx], gDur);
    } else if (gOne) {
        gStepsDone++;
        if (gStepsDone >= WALK_STEPS) {
            gOne = false;
            Serial.println("一个完整周期完成 → 回站立位");
            goHomeSel();
        } else {
            gStepIdx++;
            startPrimitiveSel(WALK_CYCLE[gStepIdx], gDur);
        }
    }
}

static void doGoto(float x, float y) {
    if (!haveSel()) return;
    const FootPos target{x, y};
    // Validate every selected leg before changing any outputs or canceling the current action.
    double peak = 0;
    for (int i = 0; i < gSelCount; ++i) {
        const int leg = gSelList[i];
        const auto* f = ServoManager::lastCommand(robot::SERVO_PINS[leg][0]);
        const auto* t = ServoManager::lastCommand(robot::SERVO_PINS[leg][1]);
        const FootPos from = gRaw && f && t
            ? motion::footFromCommands(leg, f->applied, t->applied)
            : FootPos{gLegs[leg].footX(), gLegs[leg].footY()};
        if (!motion::feasible(leg, {from, target, 0, 1, false, false}, 0, peak)) {
            Serial.printf("! leg%d 的 goto 轨迹超出几何或舵机限位\n", leg);
            return;
        }
    }
    yieldAll();
    const unsigned long duration = max(gDur, motion::durationForPeak(peak));
    for (int i = 0; i < gSelCount; i++) {
        int id = gSelList[i];
        gLegs[id].startGrab(gLegs[id].footX(), gLegs[id].footY(), x, y, 0.0f, MODE_SWING, duration);
    }
    Serial.printf("→ %d 条腿同步 goto (%.2f, %.2f)，%lu ms\n", gSelCount, x, y, duration);
    printState();
}

// ============================================================
// 8. 单舵机直控（绕过轨迹，直接下发角度）
// ============================================================
// 只负责"写 + 记录"，不打印；调用方负责提示
static void rawSet(int leg, int joint, float deg) {
    if (leg < 0 || leg > 5 || joint < 0 || joint > 1 || !isfinite(deg)) return;
    ensureInited(leg);                       // 保证 PCA9685 已 begin 且内部坐标有意义
    if (gLegs[leg].isMoving()) {
        // 正在跑轨迹：先把内部状态冻结在当前位置（否则下一帧 update() 会用插值角度覆盖刚写的直控角）
        gLegs[leg].trackFootPosition(gLegs[leg].footX(), gLegs[leg].footY());
    }
    deg = constrain(deg, 0.0f, 180.0f);
    gRawDeg[leg][joint]   = deg;
    gRaw = true;
    ServoManager::write(leg, (ServoManager::Joint)joint, deg);   // 叠零位偏移 + 限幅 + @SERVO 遥测
}

// "逻辑角 + 零位偏移 → 实际写入角"的显示串
static String appliedStr(int leg, int joint, float deg) {
    float off = (float)robot::ANGLE_OFFSET[leg][joint];
    float fin = constrain(deg + off, 0.0f, 180.0f);
    char buf[48];
    snprintf(buf, sizeof(buf), "%.1f°(偏移%+.1f→%.1f°)", deg, off, fin);
    return String(buf);
}

// sv <leg> <joint> <角度>
static void cmdServoSingle(int leg, int joint, float deg) {
    if (!isfinite(deg)) { Serial.println("! 角度必须是有限数"); return; }
    if (leg < 0 || leg > 5) { Serial.println("! leg 只能是 0~5"); return; }
    if (joint < 0 || joint > 1) { Serial.println("! joint 只能是 0/f(大腿) 或 1/t(小腿)"); return; }
    yieldMotionsForRaw();
    rawSet(leg, joint, deg);
    Serial.printf("→ 直控 leg%d ch%d %s = %s\n", leg, robot::SERVO_PINS[leg][joint],
                  jointName(joint), appliedStr(leg, joint, gRawDeg[leg][joint]).c_str());
    emitState();      // 直控命令可能被上位机滑条连续调用 → 只回机器可读状态，不刷人读状态行
}

// ch <通道> <角度>（按 PCA9685 通道直控，映射回 腿/关节）
static void cmdServoChannel(int ch, float deg) {
    if (!isfinite(deg)) { Serial.println("! 角度必须是有限数"); return; }
    if (ch < 0 || ch > 15) { Serial.println("! 通道只能是 0~15"); return; }
    int leg = -1, joint = -1;
    for (int l = 0; l < 6 && leg < 0; l++)
        for (int j = 0; j < 2; j++)
            if (robot::SERVO_PINS[l][j] == ch) { leg = l; joint = j; break; }
    if (leg < 0) {
        Serial.printf("! ch%d 没有接舵机（本机只有 ch0~ch11 = leg0~5 的大腿/小腿，接线见 Config.h::SERVO_PINS）\n", ch);
        return;
    }
    yieldMotionsForRaw();
    rawSet(leg, joint, deg);
    Serial.printf("→ 直控 ch%d = leg%d %s = %s\n", ch, leg, jointName(joint),
                  appliedStr(leg, joint, gRawDeg[leg][joint]).c_str());
    emitState();      // 同上：滑条连续调用时不刷人读状态行
}

// lsv <leg> <大腿角> <小腿角>
static void cmdServoLegPair(int leg, float femurDeg, float tibiaDeg) {
    if (!isfinite(femurDeg) || !isfinite(tibiaDeg)) { Serial.println("! 角度必须是有限数"); return; }
    if (leg < 0 || leg > 5) { Serial.println("! leg 只能是 0~5"); return; }
    yieldMotionsForRaw();
    rawSet(leg, ServoManager::FEMUR, femurDeg);
    rawSet(leg, ServoManager::TIBIA, tibiaDeg);
    Serial.printf("→ 直控 leg%d: 大腿 = %s | 小腿 = %s\n", leg,
                  appliedStr(leg, ServoManager::FEMUR, gRawDeg[leg][ServoManager::FEMUR]).c_str(),
                  appliedStr(leg, ServoManager::TIBIA, gRawDeg[leg][ServoManager::TIBIA]).c_str());
    emitState();      // 同上
}

// asv <joint> <角度>（选中腿的同一关节一起下发）
static void cmdServoSelectedJoint(int joint, float deg) {
    if (!isfinite(deg)) { Serial.println("! 角度必须是有限数"); return; }
    if (!haveSel()) return;
    if (joint < 0 || joint > 1) { Serial.println("! joint 只能是 0/f(大腿) 或 1/t(小腿)"); return; }
    yieldMotionsForRaw();
    for (int i = 0; i < gSelCount; i++) rawSet(gSelList[i], joint, deg);
    Serial.printf("→ 直控 %d 条腿的%s = %.1f°（逻辑角）\n", gSelCount, jointName(joint),
                  gRawDeg[gSelList[0]][joint]);
    emitState();      // 同上
}

// center：12 路全回逻辑角 90°（安全位，不是站姿）
static void cmdCenter() {
    yieldMotionsForRaw();
    for (int l = 0; l < 6; l++) {
        rawSet(l, ServoManager::FEMUR, 90.0f);
        rawSet(l, ServoManager::TIBIA, 90.0f);
    }
    Serial.println("→ 12 路舵机全部回逻辑角 90°（安全位；不是站姿，站姿是 67.4°/112.6°）");
    emitState();      // 同上
}

// ============================================================
// 9. 打印：状态 / 位置 / 通道表 / 方向表 / 相位 / 帮助
// ============================================================
// 机器可读状态行（gait-sim 主控面板解析它同步按钮与滑条；不发人读文本，避免刷屏）
static void emitState() {
    Serial.printf("@STATE dir=%.1f sqo=%d walk=%d cont=%d steps=%u sel=%s raw=%d dur=%lu lift=%.1f"
                  " model=%u gait=%s stage=%s issued=%u stride=%.2f height=%.1f duty=%.3f"
                  " usedStride=%.2f usedLift=%.2f usedMs=%lu mirror=%d lease=%lu dropped=%lu late=%lu\\n",
                  gSettings.direction, gaitSqo(), gWalk, gWalkCont, gGait.completed(),
                  selSpec().c_str(), gRaw, gDur, gLift, robot::MODEL_VERSION, motion::gaitName(gSettings.gait),
                  motion::stageName(gGait.stage()), gGait.issued(), gSettings.stride, gSettings.height,
                  gSettings.duty, gGait.stride(), gGait.lift(), (unsigned long)gGait.duration(), gAzSign,
                  (unsigned long)LEASE_MS, (unsigned long)gFrameDropped, (unsigned long)gMaxLateMs);
}

// 人读状态 + 机器可读状态（命令执行后调用）
static void printState() {
    Serial.printf("[状态] 方向 %.1f°(%s) sqo=%d | 步态 %s%s | 选腿 %s | 直控 %s | dur %lu ms | lift %.1f cm\n",
                  gSettings.direction, dirName(gDir), gaitSqo(),
                  gWalk ? "运行中" : "停",
                  gWalk ? (gWalkCont ? "(连续)" : "(有限拍)") : "",
                  selSpec().c_str(),
                  gRaw ? "开" : "关",
                  (unsigned long)gDur, gLift);
    emitState();
}

static void printPos() {
    if (!haveSel()) return;
    LegKinematics kin(robot::LENA, robot::LENB);
    for (int i = 0; i < gSelCount; i++) {
        int id = gSelList[i];
        if (!gInited[id]) { Serial.printf("  leg%d 尚未上电（还没有写过它的舵机）\n", id); continue; }
        double fx = gLegs[id].footX(), fy = gLegs[id].footY();
        LegAngles a = kin.ik(fx, fy);
        float thighDeg = (float)(a.thigh * 180.0 / PI);
        float kneeDeg  = (float)(a.knee  * 180.0 / PI);
        Serial.printf("  leg%d foot=(%.2f, %.2f) r=%.2fcm | ag2=%.1f° ag1=%.1f° | 舵机 大腿=%.1f° 小腿=%.1f° | %s\n",
                      id, fx, fy, kin.dist(fx, fy), thighDeg, kneeDeg,
                      thighDeg, 180.0f - kneeDeg,
                      gLegs[id].isMoving() ? "[moving]" : "[stop]");
    }

    if (gRaw) Serial.println("  (注: 舵机直控中，以上坐标/关节角不代表实际姿态)");
}

// 某腿某关节"此刻会写入的逻辑角（不含零位偏移）"
static float logicalAngleOf(int leg, int joint, bool& known) {
    const auto* last = ServoManager::lastCommand(robot::SERVO_PINS[leg][joint]);
    known = last != nullptr;
    return last ? last->logical : 0.0f;
}

static void printMap() {
    Serial.println("--- 12 路舵机对照表（写舵机一律经 ServoManager::write，角度=逻辑角）---");
    Serial.println("   ch  leg   关节         当前逻辑角   零位偏移   实际写入角");
    for (int l = 0; l < 6; l++) {
        for (int j = 0; j < 2; j++) {
            bool known = false;
            float lg = logicalAngleOf(l, j, known);
            float off = (float)robot::ANGLE_OFFSET[l][j];
            if (known) {
                Serial.printf("  %3d  leg%d  %-12s  %7.1f°   %+6.1f°   %7.1f°%s\n",
                              robot::SERVO_PINS[l][j], l, jointName(j), lg, off,
                              constrain(lg + off, 0.0f, 180.0f),
                              gSel[l] ? "   ← 选中" : "");
            } else {
                Serial.printf("  %3d  leg%d  %-12s   %7s    %+6.1f°   %7s%s\n",
                              robot::SERVO_PINS[l][j], l, jointName(j), "?", off, "?",
                              gSel[l] ? "   ← 选中" : "");
            }
        }
    }
    Serial.println("  注: \"?\" = 该腿还没上电 / 该关节直控过但没记录；实际写入角 = 逻辑角 + 偏移，再限幅 0~180");
    printState();
}

static void printDirTable() {
    Serial.println("--- 6 个行走方向（三角步态；0° = 机头方向，逆时针为正）---");
    Serial.println("  旧三角步态方向 = 相位表偏移（60° 一档）");
    for (int i = 0; i < 6; i++) {
        int sqo = (gAzSign > 0) ? i : ((6 - i) % 6);
        Serial.printf("  %-2s = %3d°  %-4s  (用 sqo=%d；同向于 leg%d 的方位角)\n",
                      dirKey(i), 60 * i, dirName(i), sqo, i);
    }
    Serial.printf("  当前: 方向 %d° (%s) → sqo=%d | 方位角制式: %s\n",
                  (int)(60.0f * (float)gDir), dirName(gDir), gaitSqo(),
                  (gAzSign > 0) ? "逆时针 leg1=左前" : "顺时针镜像 leg1=右前");
    Serial.println("  兼容档使用 6 个方向；新步态支持连续方向；mirror 镜像腿序");
}

static void printPhase() {
    Serial.println("--- 三角步态状态（引擎 = src/TripodGait.cpp 的 TripodGait）---");
    Serial.printf("  运行: %s | 方向 %d°(%s) | SequentialOffset=%d | 方位角制式: %s\n",
                  gWalk ? "YES" : "no", (int)(60.0f * (float)gDir), dirName(gDir), gaitSqo(),
                  (gAzSign > 0) ? "逆时针 leg1=左前" : "顺时针镜像 leg1=右前");
    Serial.printf("  每拍 %lu ms | 抬腿 %.1f cm | %s\n", (unsigned long)gDur, gLift,
                  gWalk ? ((gWalkCont) ? "连续行走" : "有限拍数") : "未运行");
    Serial.printf("  腿方位角:");
    for (int i = 0; i < 6; i++) Serial.printf(" leg%d=%d°", i, (int)legAzimuthDeg(i));
    Serial.println();
    Serial.printf("  第一拍: %s\n", phaseStr(true).c_str());
    Serial.printf("  第二拍: %s\n", phaseStr(false).c_str());
    Serial.printf("  当前步态 %s，已发 %u 拍，已落足 %u 拍，当前阶段 %s\\n",
                  motion::gaitName(gSettings.gait), gGait.issued(), gGait.completed(),
                  motion::stageName(gGait.stage()));
    printState();
}

static void printHelp() {
    Serial.println("--- 六足主控台 · 方向行走 / 选腿控制 / 单舵机控制 ---");
    Serial.println("【状态】");
    Serial.println("  help | ?                     本帮助");
    Serial.println("  state                        打印全局状态（方向/步态/选腿/直控）");
    Serial.println("  pos                          选中腿足端坐标 + 关节角 + 舵机逻辑角");
    Serial.println("  map                          12 路舵机对照表（通道 / 腿 / 关节 / 当前角 / 偏移）");
    Serial.println("  verify                       方向自检（纯数学，不需要硬件）");
    Serial.println("【方向行走】");
    Serial.println("  gait legacy|five|four|three   旧三角、五足、四足、三足支撑");
    Serial.println("  stride <cm> | height <cm> | duty <0~1>   步幅 / 站高 / 支撑占空比");
    Serial.println("  drive [度] | keepalive | snapshot       带有效期的遥控、续期和命令帧");
    Serial.println("  f  fl  bl  b  br  fr         往 0°/60°/120°/180°/240°/300° 走");
    Serial.println("                               （空闲时直接开跑；跑动时在落足边界应用新方向）");
    Serial.println("  dir <角度>                   设定平移方向（legacy 吸附到 60° 档）；不带角度=打印方向表");
    Serial.println("  dirs                         方向表");
    Serial.println("  walk [n]                     按当前方向连续走；n>0 = 只走 n 拍后回站位");
    Serial.println("  step                         只走一拍（调试相位/接线）");
    Serial.println("  phase                        打印方向 / sqo / 拍数 / 两张相位表");
    Serial.println("  mirror                       腿序镜像（实物走反了 / 腿序与假设相反时用）");
    Serial.println("【选腿控制（下列命令只作用于\"当前选中的腿\"）】");
    Serial.println("  legs                         查看当前选中的腿");
    Serial.println("  legs all | none | <n ...> | <a-b>    选腿: legs 0 / legs 0 1 2 / legs 0-3");
    Serial.println("  cycle                        选中腿同步连续行走循环（摆动抬腿前迈 + 支撑贴地收回）");
    Serial.println("  one                          选中腿只走一个完整周期，然后回站立位");
    Serial.println("  each                         逐腿巡检：选中的腿依次各走一个周期再回位");
    Serial.println("  home                         选中腿平滑回站立位");
    Serial.println("  goto <x> <y>                 选中腿足端移到绝对坐标 (cm，原点=髋关节)");
    Serial.println("  lift <cm>                    抬腿高度（默认 3.0）");
    Serial.println("  dur <ms>                     动作时长 / 每拍时长（默认 600，太小会丢步/堵转）");
    Serial.println("【单舵机控制（直接下发角度，绕过轨迹）】");
    Serial.println("  sv <leg> <joint> <角度>      单条腿的单个舵机（joint: 0/f=大腿, 1/t=小腿）  例: sv 0 0 90");
    Serial.println("  ch <通道> <角度>             按 PCA9685 通道直控（ch0~11，映射见 map）    例: ch 3 90");
    Serial.println("  lsv <leg> <大腿角> <小腿角>  一条腿两个舵机同时下发                      例: lsv 0 67.4 112.6");
    Serial.println("  asv <joint> <角度>           选中腿的同一关节一起下发                    例: asv 0 90");
    Serial.println("  center                       12 路全部回逻辑角 90°（安全位，不是站姿）");
    Serial.println("【停止】");
    Serial.println("  stop                         停一切（步态/循环/直控）并平滑回站立位");
    Serial.println("注: 角度是逻辑角（不含零位偏移）；实际写入角 = 逻辑角 + 偏移，再限幅 0~180；");
    Serial.println("    退出直控后依据最后下发的角度重建足端；");
    Serial.println("    snapshot 使能 @FRAME 遥测，stream off 关闭。");
    printState();
}

// ============================================================
// 10. 命令解析
// ============================================================
static void emitFrame(uint32_t now) {
    motion::Frame frame{};
    frame.sequence = gFrameSequence++;
    frame.ms = now;
    frame.stage = gGait.stage();
    frame.issued = gGait.issued();
    frame.completed = gGait.completed();
    frame.support = gWalk ? gGait.supportMask() : 0;
    frame.mirror = gAzSign;
    for (int ch = 0; ch < robot::CHANNEL_COUNT; ++ch) frame.channels[ch] = ServoManager::lastCommand(ch);
    FootPos feet[6];
    for (int i = 0; i < 6; ++i) {
        if (!gRaw && gInited[i]) {
            feet[i] = {gLegs[i].footX(), gLegs[i].footY()};
            frame.requested[i] = &feet[i];
        }
    }
    char output[motion::frameCapacity];
    motion::formatFrame(output, frame);
    const size_t length = strlen(output);
    if (Serial.availableForWrite() >= int(length + 1)) {
        Serial.write(reinterpret_cast<const uint8_t*>(output), length);
        Serial.write('\n');
    } else {
        ++gFrameDropped;
    }
}

static bool number(const String& input, float& value) {
    char last;
    return sscanf(input.c_str(), "%f %c", &value, &last) == 1 && isfinite(value);
}

static void requestStop() {
    gLeased = false;
    if (gWalk) gGait.stop();
    else { yieldAll(); goHomeAll(); }
    printState();
}

static void handleCommand(const String& line) {
    String cmd = line; cmd.trim();
    if (cmd.length() == 0) return;

    int sp = cmd.indexOf(' ');
    String first = (sp < 0) ? cmd : cmd.substring(0, sp);
    String rest  = (sp < 0) ? String("") : cmd.substring(sp + 1);
    first.toLowerCase();
    rest.trim();

    if (first == "snapshot") {
        gStreaming = true;
        emitState();
        return;
    }
    if (first == "stream" && rest == "off") { gStreaming = false; return; }
    if (first == "keepalive") {
        if (gLeased) gLeaseAt = millis();
        return;
    }
    if (first == "drive") {
        float angle = gSettings.direction;
        if (rest.length() && !number(rest, angle)) { Serial.println("! drive [度]"); return; }
        setDirection(angle, false);
        if (!gWalk || !gLeased) walkStart(true, 0);
        gLeased = true;
        gLeaseAt = millis();
        return;
    }
    if (first == "gait") {
        if (rest.length()) {
            if (rest == "legacy") gSettings.gait = motion::Gait::Legacy;
            else if (rest == "five") gSettings.gait = motion::Gait::Five;
            else if (rest == "four") gSettings.gait = motion::Gait::Four;
            else if (rest == "three") gSettings.gait = motion::Gait::Three;
            else { Serial.println("! gait legacy|five|four|three"); return; }
            gSettings.duty = 0;
            if (gSettings.gait == motion::Gait::Legacy) gSettings.direction = gDir * robot::LEG_SPACING_DEG;
            if (gWalk) gGait.configure(gSettings);
        }
        printState();
        return;
    }
    if (first == "stride" || first == "height" || first == "duty") {
        float v;
        if (number(rest, v)) {
            const int count = motion::groups(gSettings.gait);
            if (first == "stride" && v >= 0.1f && v <= 6.8f) gSettings.stride = v;
            else if (first == "height" && v >= 9.5f && v <= 12.0f) gSettings.height = v;
            else if (first == "duty" && (v == 0 || (v >= 1 - 0.98f / count && v <= 1 - 0.2f / count))) gSettings.duty = v;
            else { Serial.println("! 参数超出范围"); return; }
            if (gWalk) gGait.configure(gSettings);
        } else if (rest.length()) { Serial.println("! 参数必须是有限数"); }
        printState();
        return;
    }

    // ---- 状态类 ----
    if (first == "help" || first == "?") { printHelp(); return; }
    if (first == "state")  { printState(); return; }
    if (first == "pos")    { printPos(); return; }
    if (first == "map")    { printMap(); return; }
    if (first == "verify" || first == "check") { verifyDirections(); return; }
    if (first == "dirs")   { printDirTable(); return; }
    if (first == "phase" || first == "status") { printPhase(); return; }
    if (first == "mirror" || first == "rev") {
        gAzSign = -gAzSign;
        Serial.printf("腿方位角制式 → %s（leg1 = %s）\n",
                      (gAzSign > 0) ? "逆时针" : "顺时针镜像",
                      (gAzSign > 0) ? "左前" : "右前");
        Serial.println("（跑动中在落足边界切换）");
        setDirection(gSettings.direction, false);
        return;
    }

    // ---- 选腿 ----
    if (first == "legs" || first == "sel") {
        if (rest.length() == 0) { printSel(); printState(); }
        else applySel(rest);
        return;
    }

    // ---- 踏板参数 ----
    if (first == "lift") {
        float v = 0;
        if (number(rest, v) && v >= 0.0f && v <= 20.0f) {
            gLift = gSettings.lift = v;
            if (gWalk) gGait.configure(gSettings);
            for (int i = 0; i < 6; i++) gLegs[i].setArcLift(v);
            Serial.printf("抬腿高度 = %.1f cm（下一次原语 / 下一拍生效；三角步态的摆动腿用的是这个值）\n", v);
        } else Serial.println("! 用法: lift <cm>  (0~20)");
        printState();
        return;
    }
    if (first == "dur") {
        float v;
        if (number(rest, v) && v >= 150 && v <= 10000 && v == floorf(v)) {
            gDur = gSettings.beatMs = v;
            if (gWalk) gGait.configure(gSettings);
        } else Serial.println("! 用法: dur <150~10000 ms 的整数>");
        printState();
        return;
    }

    // ---- 方向行走 ----
    if (first == "walk" || first == "go" || first == "tripod" || first == "tri") {
        String arg = rest; arg.toLowerCase(); arg.trim();
        if (arg == "stop" || arg == "off" || arg == "home") { requestStop(); return; }
        if (arg == "one" || arg == "cycle") { walkStart(false, motion::groups(gSettings.gait)); return; }
        if (arg == "step" || arg == "half") { walkStart(false, 1); return; }
        if (arg.length() == 0)              { walkStart(true, 0);  return; }
        if (isAllDigits(arg))               { walkStart(false, arg.toInt()); return; }
        Serial.println("! 用法: walk | walk <拍数> | step | walk stop");
        return;
    }
    if (first == "step") { walkStart(false, 1); return; }
    if (first == "stop" || first == "halt") { requestStop(); return; }
    if (first == "dir") {
        float v;
        if (number(rest, v)) setDirection(v, false);
        else printDirTable();
        return;
    }
    for (int i = 0; i < 6; i++) {
        if (first == dirKey(i)) { setDirIndex(i, true); return; }   // 6 个方向快捷键：设方向 + 开跑
    }

    // ---- 选中腿的动作 ----
    if (first == "cycle") { startCycle(); return; }
    if (first == "one")   { startOne();   return; }
    if (first == "each")  { startEach();  return; }
    if (first == "home")  { yieldAll(); goHomeSel(); return; }
    if (first == "goto") {
        float x = 0, y = 0;
        if (sscanf(rest.c_str(), "%f %f", &x, &y) == 2) doGoto(x, y);
        else Serial.println("! 用法: goto <x> <y>   (cm，各腿自身坐标系，原点=髋关节；站姿 (−6, −10))");
        return;
    }

    // ---- 单舵机直控 ----
    if (first == "sv" || first == "servo1") {
        int leg = -1; char js[16] = { 0 }; float deg = 0;
        if (sscanf(rest.c_str(), "%d %15s %f", &leg, js, &deg) == 3) {
            String jn(js); jn.toLowerCase();
            int joint = -1;
            if (jn == "0" || jn == "f" || jn == "femur" || jn == "thigh" || jn == "大腿") joint = ServoManager::FEMUR;
            else if (jn == "1" || jn == "t" || jn == "tibia" || jn == "knee" || jn == "小腿") joint = ServoManager::TIBIA;
            if (joint < 0) { Serial.println("! joint 只能是 0/f/femur(大腿) 或 1/t/tibia(小腿)"); return; }
            cmdServoSingle(leg, joint, deg);
        } else Serial.println("! 用法: sv <leg 0~5> <joint 0|1> <角度 0~180>   例: sv 0 0 90");
        return;
    }
    if (first == "ch" || first == "channel") {
        int ch = -1; float deg = 0;
        if (sscanf(rest.c_str(), "%d %f", &ch, &deg) == 2) cmdServoChannel(ch, deg);
        else Serial.println("! 用法: ch <通道 0~11> <角度 0~180>   例: ch 3 90（映射见 map）");
        return;
    }
    if (first == "lsv" || first == "legsv") {
        int leg = -1; float a0 = 0, a1 = 0;
        if (sscanf(rest.c_str(), "%d %f %f", &leg, &a0, &a1) == 3) cmdServoLegPair(leg, a0, a1);
        else Serial.println("! 用法: lsv <leg 0~5> <大腿角> <小腿角>   例: lsv 0 67.4 112.6");
        return;
    }
    if (first == "asv") {
        char js[16] = { 0 }; float deg = 0;
        if (sscanf(rest.c_str(), "%15s %f", js, &deg) == 2) {
            String jn(js); jn.toLowerCase();
            int joint = -1;
            if (jn == "0" || jn == "f" || jn == "femur" || jn == "大腿") joint = ServoManager::FEMUR;
            else if (jn == "1" || jn == "t" || jn == "tibia" || jn == "小腿") joint = ServoManager::TIBIA;
            if (joint < 0) { Serial.println("! joint 只能是 0/f(大腿) 或 1/t(小腿)"); return; }
            cmdServoSelectedJoint(joint, deg);
        } else Serial.println("! 用法: asv <joint 0|1> <角度 0~180>   例: asv 0 90（对选中的腿）");
        return;
    }
    if (first == "center" || first == "mid") { cmdCenter(); return; }

    // ---- 兼容老习惯：0 <角度> / 1 <角度>（= asv，选中腿的同一关节）----
    if (first == "0" || first == "1") {
        float ang = 0;
        if (sscanf(rest.c_str(), "%f", &ang) == 1) { cmdServoSelectedJoint(first.toInt(), ang); return; }
        Serial.println("! 用法: 0 <角度>(选中腿大腿) 或 1 <角度>(选中腿小腿)；单条腿请用 sv <leg> <joint> <角度>");
        return;
    }

    Serial.println("! 无法识别的命令（输入 help 查看全部命令）");
}

// ============================================================
// 11. 主程序
// ============================================================
void setup() {
    Serial.setTxBufferSize(1024);
    Serial.begin(115200);
    ServoManager::begin();
    ServoManager::setTelemetry(false);
    Serial.println("=== 六足主控台 (MODE_MASTER_CONTROL) ===");
    applySel(BOOT_SELECTION);
}

void loop() {
    static char line[128];
    static unsigned size = 0;
    static bool overflow = false;
    static uint32_t lastTick = 0, lastFrame = 0;
    static bool dirty = false;
    for (int i = 0; i < 32 && Serial.available(); ++i) {
        const char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (overflow) Serial.println("! 命令过长");
            else if (size) {
                line[size] = 0;
                handleCommand(String(line));
                dirty = true;
            }
            size = 0;
            overflow = false;
        } else if (size < sizeof(line) - 1 && !overflow) line[size++] = c;
        else overflow = true;
    }

    const uint32_t now = millis();
    if (now - lastTick < motion::framePeriodMs) return;
    if (lastTick && now - lastTick - motion::framePeriodMs > gMaxLateMs)
        gMaxLateMs = now - lastTick - motion::framePeriodMs;
    lastTick = now;
    bool moving = gWalk;
    bool boundary = false;
    if (gWalk) {
        boundary = stepWalk(now);
    } else {
        for (int i = 0; i < 6; ++i) {
            if (gInited[i] && gLegs[i].isMoving()) {
                moving = true;
                gLegs[i].update();
            }
        }
        if (gEach) stepEach();
        else stepSelected();
    }
    if (gStreaming && (boundary || ((moving || dirty) && now - lastFrame >= 40))) {
        emitFrame(now);
        lastFrame = now;
        dirty = false;
    }
}
