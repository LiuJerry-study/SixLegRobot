/* 可切换多腿测试（串口交互式）
 *
 * 目的：验证一条或多条腿的完整功能 —— 逆运动学（足端→关节角）、摆动相/支撑相
 *       轨迹、舵机联动。被测腿的集合在【运行时】随时切换，不用改代码重编译：
 *         - 选中 1 条腿：单腿逐步验证（等价于原来的单腿测试）；
 *         - 选中 2~3 条（或更多）腿：这几条腿【严格同步】做同一个动作，
 *           用于验证多腿装配/运动学一致性、同步供电能力等；
 *         - `each`：自动把当前选中的腿【依次】各走一个完整周期再测下一条，
 *           适合装机后逐腿巡检。
 *
 * ── 运行方式（PlatformIO）──────────────────────────────
 *   1. platformio.ini 的 build_flags 设为：-DMODE_SINGLE_LEG_TEST
 *   2. Upload 烧录，打开串口监视器（115200 波特率）
 *   3. 发送命令（每行一条，回车结尾；运动类命令全部作用于当前选中的腿）：
 *
 *        help                → 打印命令帮助
 *        legs                → 打印当前选中的腿
 *        legs all            → 选中全部 6 条腿
 *        legs <n n ...>      → 选中指定腿，如 "legs 0"、"legs 0 1"、"legs 0 1 2"、
 *                              "legs 1 3 5"；支持区间 "legs 0-3"
 *                              （切换后：被移出的腿平滑回位；新选中的腿自动上电回位）
 *        cycle               → 选中腿同步连续行走循环（摆动相抬腿前迈 + 支撑相贴地收回）
 *        one                 → 选中腿只走一个完整周期，然后自动回站立位
 *        each                → 逐腿巡检：选中的每条腿【依次】完整走一个周期再回位
 *        home                → 选中腿平滑回默认站立位
 *        goto <x> <y>        → 选中腿足端同步移动到绝对坐标 (x, y)（cm，各腿自身坐标系，
 *                              原点=髋关节）
 *        lift <cm>           → 设置弧线抬腿高度（默认 3.0）
 *        dur <ms>            → 设置每个动作的时长（默认 600；腿多时建议 400~800）
 *        0 <角度>            → 选中腿的 大腿(FEMUR) 舵机同步转到角度 (0~180)，如 0 90
 *        1 <角度>            → 选中腿的 小腿(TIBIA) 舵机同步转到角度 (0~180)，如 1 45
 *        pos                 → 打印选中腿的足端坐标/关节角与状态
 *        servo               → 打印选中腿舵机的【无偏转角】：大腿 ag2 / 小腿 180−ag1，
 *                              不含 Config 零位偏移、也不限幅（就是交给 ServoManager::write()
 *                              的那一刻的角度）；只打这一个数，不附含偏移角/脉冲
 *        servo watch [ms]    → 连续打印选中腿舵机的【无偏转角】（默认每 200 ms 检查一次，角度有
 *                              变化才打一行）；"servo off" 关闭。格式：F/T 无偏转角
 *        stop                → 停止循环，平滑回站立位
 *
 *   用到的动作原语（定义见 include/SingleLeg.h，实现在 src/SingleLeg.cpp）：
 *     PRIM_ARC_EXTEND   a 弧线张：摆动相，抬腿弧线伸出（脚悬空前迈）
 *     PRIM_ARC_RETRACT  b 弧线收：摆动相，抬腿弧线收回
 *     PRIM_LINE_EXTEND  c 直线张：支撑相，贴地直线伸出（推地）
 *     PRIM_LINE_RETRACT d 直线收：支撑相，贴地直线收回（抓地）
 *   本文件一个行走周期只用到 a → d 两步；每个原语的起点都取"当前足端"（防跳变），
 *   终点：张 → 伸出脚、收 → 站立脚；时间插值用 smoothstep t = t²(3−2t)；
 *   摆动相 Y 额外叠 lift·sin(πt) 抬腿，支撑相 Y 锁死在目标高度（防打滑）。
 *
 *   舵机直控（0 x / 1 x）说明：
 *     - 发送后进入"直控模式"，轨迹运动（cycle/goto 等）会先被冻结，防止打架；
 *     - 直控模式中再发任意运动命令，会自动先瞬间回默认站位并同步内部状态
 *       （此时打印会提示），之后正常运动；
 *     - 注意：直控只发角度、不动内部坐标，故直控期间 pos 打印的坐标不可信。
 *
 *   同步方式：选中腿同时 startPrimitive，每步等它们【全部】停稳才一起进入下一步
 *   （以最慢的一条腿为准），保证严格同步、不互相拉扯。
 *
 *   默认选中的腿改下方 DEFAULT_SELECTION；腿号 0~5 与 PCA9685 通道的映射见 Config.h
 *   （leg0=ch0/1 … leg5=ch10/11）。
 *   每条腿的坐标系都是独立的（原点 = 该腿自己的髋关节，y+ 朝上、x− 朝髋外侧），
 *   本文件不引入任何整机位姿或相位差 —— 所以"选中多条腿"= 这几条腿做完全相同的动作
 *   （同坐标、同帧、同角度）。默认站立足端 = (−6, −10)，伸出足端 ≈ (−12.77, −10)。
 *
 * ── 安全建议 ──────────────────────────────────────────
 *   - 上电时 ServoManager::begin() 不驱动舵机，但被选中的腿会逐条 init() 回默认
 *     站立位（一次只动该腿 2 个舵机，每条腿写完 delay(100) 再处理下一条），
 *     务必先把机器人架起来（被测腿悬空）再发命令；
 *   - 角度口径（"外倾为正"）：ik() 返回【弧度】，默认站姿 ag1 = ag2 = +67.38°；
 *     舵机映射 大腿 = ag2 = 67.38°（2026-09 实物实测：大腿舵机 0° = 竖直向上、180° = 竖直向下）、
 *     小腿 = 180 − ag1 = 112.62°；零位偏移当前整表为 0，叠加后仍在 0~180 内；
 *   - 同时动的腿越多电流越大（2~3 条同时动作峰值可达 2A 以上），舵机电源务必用
 *     独立的大电流 5V（单腿可小电流，多腿建议 5A 级），并与 ESP32 共地；
 *   - 先跑"单舵机/单腿"测试确认每路都正常，再上多腿同步，便于定位问题；
 *   - goto 的坐标要落在可达范围（内圆 |6.5−12.5| = 6cm，外圆 6.5+12.5 = 19cm）内；
 *     默认站立足端 (−6, −10)（d ≈ 11.66cm）与伸出足端 (−12.77, −10)（d ≈ 16.22cm）
 *     都在范围内。
 *
 * ── 这个文件是怎么被编译的 ─────────────────────────────
 *   本文件【不是 PlatformIO 单元测试】：src/main.cpp 里有
 *       #elif defined(MODE_SINGLE_LEG_TEST) … #include "../test/SingleLegTest.cpp"
 *   即它被直接包含进 main.cpp 所在的同一个编译单元（当前 platformio.ini 的
 *   build_flags 生效值就是 -DMODE_SINGLE_LEG_TEST），setup()/loop()/全局变量与
 *   main.cpp 共享同一个 TU —— 因此本文件里的 PI 用的是 Arduino.h 的宏（3.14159…），
 *   而不是 src/LegKinematics.cpp 里那个同名的 const double。
 *
 * ── 数字孪生遥测 ──────────────────────────────────────
 *   本文件不自带遥测：所有写舵机都走 ServoManager::write()，该函数已在固件层内置
 *   节流回传一行机器可读遥测：
 *       @SERVO:<ch>:<最终角>      例： @SERVO:0:67.4
 *   ch = PCA9685 通道号；最终角 = 逻辑角 + 零位偏移，再限幅 0~180；角度变化 ≥0.5°
 *   且每通道 ≥50ms 才回传一条（默认开，可用 ServoManager::setTelemetry(false) 关）。
 *   gait-sim「悬空模式」解析该行即可 1:1 镜像真机 12 路舵机姿态。
 *
 * ── 舵机角度打印（servo 命令）──────────────────────────
 *   `servo` 打印当前选中腿两个舵机的【无偏转角】（每条腿两行）—— 即"计算角"，不含
 *   Config.h::ANGLE_OFFSET、也不限幅，正是 ServoManager::write() 的入参：
 *       无偏转角 = 大腿 ag2 / 小腿 180 − ag1（与 src/SingleLeg.cpp 一致）
 *   只打印这一个角度值：不叠零位偏移、不做 0~180 限幅、不附脉冲计数
 *   （含偏移后的真机实际角看 @SERVO 遥测，或用 test/SingleServoTest.cpp 的 map/写入打印）。
 *   直控模式（0 x / 1 x）下内部足端坐标不更新，故打印的是直控时下发的角度（会标注）。
 *   连续观察用 `servo watch [ms]`（默认每 200ms 检查，角度有变化才打一行，同样是无偏转角），
 *   `servo off` 关闭。
 *   它和 printPos() 的 "servo:" 行口径相同（都是无偏转角 大腿 ag2 / 小腿 180−ag1），区别只在排版
 *   与"单腿完整显示 / 多腿逐条显示"。
 *
 * ⚠ 注意（本轮改动只涉及注释与本文件内与固件同口径的换算）
 *   1) 已解决的旧问题：固件曾写 servoThigh = 180 − ag2（站姿算出 247.38°、被限幅到 180° 端点），
 *      后来改成 大腿 = 90 + ag2；**2026-09 实物实测确认大腿舵机 0° = 竖直向上、180° = 竖直向下，
 *      最终口径是 大腿 = ag2（站姿 67.38°）**，本文件 printPos()/servo 命令的换算已随之改为 ag2。
 *      （ik() 返回【弧度】，本文件与 SingleLeg.cpp 各乘一次 180/π —— 一直是正确的一次换算。）
 *   2) printPos() 的 servo: 行与 `servo` 命令都是"无偏转角"（大腿 ag2、小腿 180 − ag1），
 *      与真机同口径，但没叠 ANGLE_OFFSET（当前整表为 0）、没做 0~180 限幅，所以 ≠ 真正写进舵机的角度；
 *      要看真正写进去的角（叠偏移 + 限幅），用 @SERVO 遥测或 test/SingleServoTest.cpp。
 *   3) src/SingleLeg.cpp 的 init() 里站立高度参数只是被记录（当前实现未参与运算），
 *      默认站姿由几何决定 = 上面的 (−6, −10)；改 STAND_HEIGHT 不会改变站姿。
 *
 * 本文件为自包含版，也可在 Arduino IDE 里与以下源文件一起编译：
 *   include/Config.h、include/LegKinematics.h、include/ServoManager.h、
 *   include/SingleLeg.h、src/LegKinematics.cpp、src/ServoManager.cpp、src/SingleLeg.cpp
 *
 * ✅ 编译状态（2026-09-10 实测）：`pio run` 通过（RAM 6.9% / Flash 25.8%）。
 *   文件头曾记录的编译阻塞 —— src/ServoManager.cpp 里 `const double PI` 与 Arduino.h 的 PI 宏同名相撞
 *   （error: expected unqualified-id before numeric constant）—— 现已不复存在：该文件不再定义 PI
 *   （只用 math.h 的 fabsf），LegKinematics.cpp 里同样的行也已删除。若将来又把 `const double PI`
 *   写到文件作用域，这个错误会重新出现。
 */

#include <Arduino.h>
#include "SingleLeg.h"
#include "ServoManager.h"
#include "LegKinematics.h"
#include "Config.h"

// ================= 硬件上的 6 条腿 =================
// 腿号 0~5 → PCA9685 通道 leg*2 + joint（joint: 0=大腿 FEMUR、1=小腿 TIBIA）
SingleLeg legs[6] = {
    SingleLeg(0), SingleLeg(1), SingleLeg(2),
    SingleLeg(3), SingleLeg(4), SingleLeg(5)
};
bool legInited[6] = { false, false, false, false, false, false };

// ================= 测试参数 =================
// 上电默认选中的腿（运行中用 `legs ...` 命令随时切换，见文件头注释）：
//   "all"      → 全部 6 条
//   "0"        → 只测腿 0（原单腿测试）
//   "0 1 2"    → 同时测 0、1、2（严格同步）
//   "1 3 5" / "0-3" → 任意组合 / 区间
const char DEFAULT_SELECTION[] = "all";        // ← 按需修改
const float STAND_HEIGHT = 10.0f;              // 站立高度（cm）；⚠ 只被 init() 记录，站姿恒为 (−6, −10)
const unsigned long DEFAULT_DURATION_MS = 600; // 单个动作默认时长
const float DEFAULT_ARC_LIFT = 5.0f;           // 弧线默认抬腿高度（cm）

// ================= 选中集合 =================
bool legSelected[6] = { false, false, false, false, false, false };
int  selectedList[6];      // 当前选中的腿号（升序）
int  selectedCount = 0;

// ================= 内部状态 =================
unsigned long primDuration = DEFAULT_DURATION_MS;
float arcLiftCm = DEFAULT_ARC_LIFT;
bool cycling = false;      // 连续循环模式
bool oneShot = false;      // 只跑一个周期模式
bool eachMode = false;     // 逐腿巡检模式
bool eachPending = false;  // each 起始：等全部选中腿回位
int  eachIndex = 0;        // 巡检到第几条
int  eachPhase = 0;        // 当前腿阶段: 0=待发ARC_EXTEND 1=待发LINE_RETRACT 2=待发home 3=完成
int  stepsDone = 0;        // 本周期已完成的步数
int  cycleStep = 0;        // 当前周期步
bool rawDirty = false;     // 舵机直控模式标记（发过 0 x / 1 x 后为 true）

// 直控模式（0 x / 1 x）下最近一次下发的逻辑角：直控不改内部足端坐标，
// 所以只能在这里记录，供 servo 命令如实打印（见「舵机角度打印」一节）
float rawServoDeg[6][2] = { { 0, 0 } };
bool  rawServoValid[6][2] = { { false, false } };

// 一个完整行走周期：摆动相抬腿前迈 → 支撑相贴地收回
const int CYCLE_STEPS = 2;
const LegPrimitive CYCLE[CYCLE_STEPS] = {
    PRIM_ARC_EXTEND,    // 摆动相：抬腿弧线伸出（脚悬空前抬）
    PRIM_LINE_RETRACT   // 支撑相：贴地直线收回（脚掌抓地/推地）
};

const char* primName(LegPrimitive p) {
    switch (p) {
        case PRIM_ARC_EXTEND:   return "ARC_EXTEND(摆动相抬腿前迈)";
        case PRIM_ARC_RETRACT:  return "ARC_RETRACT(摆动相抬腿收回)";
        case PRIM_LINE_EXTEND:  return "LINE_EXTEND(支撑相贴地伸出)";
        case PRIM_LINE_RETRACT: return "LINE_RETRACT(支撑相贴地收回)";
    }
    return "?";
}

// ========== 选中集合：解析 / 应用 ==========

bool isAllDigits(const String& s) {
    if (s.length() == 0) return false;
    for (unsigned int i = 0; i < s.length(); i++) {
        if (s[i] < '0' || s[i] > '9') return false;
    }
    return true;
}

void selAdd(int list[], int& n, int id) {
    if (id < 0 || id > 5) return;
    for (int i = 0; i < n; i++) if (list[i] == id) return;   // 去重
    list[n++] = id;
}

void selAddRange(int list[], int& n, int a, int b) {
    if (a > b) { int t = a; a = b; b = t; }   // 允许 "3-1"
    if (a < 0) a = 0;
    if (b > 5) b = 5;
    for (int i = a; i <= b; i++) selAdd(list, n, i);
}

// 解析 "all" / "none" / "0 1 2" / "0-3" 等；返回选中数量，-1 = 语法错误
int parseSelection(const String& spec, int out[6]) {
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
                if (dash > 0 && isAllDigits(tok.substring(0, dash)) &&
                    isAllDigits(tok.substring(dash + 1))) {
                    selAddRange(out, n, tok.substring(0, dash).toInt(),
                                tok.substring(dash + 1).toInt());
                } else if (isAllDigits(tok)) {
                    int id = tok.toInt();
                    if (id > 5) return -1;   // 腿号只能是 0~5
                    selAdd(out, n, id);
                } else {
                    return -1;   // 非法 token（如 "legs x"）
                }
            }
        }
        if (sp < 0) break;
        pos = sp + 1;
    }
    return n;
}

void sortSelected() {
    for (int i = 0; i < selectedCount - 1; i++)
        for (int j = i + 1; j < selectedCount; j++)
            if (selectedList[j] < selectedList[i]) {
                int t = selectedList[i]; selectedList[i] = selectedList[j]; selectedList[j] = t;
            }
}

void printSelection() {
    Serial.print("当前选中: ");
    if (selectedCount == 0) { Serial.println("(无，请用 legs 选择，如 legs 0 1)"); return; }
    for (int i = 0; i < selectedCount; i++) Serial.printf("leg%d ", selectedList[i]);
    Serial.printf("(共%d条)\n", selectedCount);
}

// 单腿平滑回默认站立位（后台完成，不停顿）
// 走 startGrab(..., lift=0, MODE_SWING)：直线贴地过去、不抬腿，中途被打断也不跳变
void homeLeg(int id) {
    double hx, hy;
    SingleLeg::defaultStandingFoot(hx, hy);
    legs[id].startGrab(legs[id].footX(), legs[id].footY(),
                       (float)hx, (float)hy, 0.0f, MODE_SWING, primDuration);
}

// 让某条腿立刻"接管"回默认站位（直控模式退出用，瞬间到达）
void snapLegHome(int id) {
    double hx, hy;
    SingleLeg::defaultStandingFoot(hx, hy);
    legs[id].setFootPositionImmediate((float)hx, (float)hy);
}

// 应用新的腿选择：取消运行模式 → 被移出的腿平滑回位 → 更新集合 → 新腿首次上电回位
void applySelection(const String& desc) {
    int nl[6];
    int n = parseSelection(desc, nl);
    if (n < 0) {
        Serial.println("! legs 语法: legs all | legs <n n ...> | legs <a-b>  例: legs 0 / legs 0 1 2 / legs 0-3");
        return;
    }

    cycling = false;
    oneShot = false;
    eachMode = false;
    if (rawDirty) {
        // 直控模式的旧选中腿瞬间回默认站位
        for (int i = 0; i < selectedCount; i++) snapLegHome(selectedList[i]);
        rawDirty = false;
        Serial.println("↑ 直控结束（切换腿集合，瞬间回位）");
    }

    // 1) 被移出且正在动的腿：平滑回位（后台完成，不阻塞）
    for (int i = 0; i < 6; i++) {
        bool still = false;
        for (int k = 0; k < n; k++) if (nl[k] == i) { still = true; break; }
        if (legSelected[i] && !still && legInited[i] && legs[i].isMoving()) homeLeg(i);
    }

    // 2) 写入选集（升序）
    for (int i = 0; i < 6; i++) legSelected[i] = false;
    selectedCount = n;
    for (int k = 0; k < n; k++) { selectedList[k] = nl[k]; legSelected[nl[k]] = true; }
    sortSelected();

    // 3) 新入选且从未上电的腿：首次上电回默认站立位
    for (int k = 0; k < selectedCount; k++) {
        int id = selectedList[k];
        if (!legInited[id]) {
            Serial.printf("↑ leg%d 首次上电：回默认站立位\n", id);
            legs[id].init(STAND_HEIGHT);
            legs[id].setArcLift(arcLiftCm);
            legInited[id] = true;
        }
    }

    printSelection();
}

// ========== 选中腿的集合操作 ==========

void startPrimitiveSel(LegPrimitive p, unsigned long dur) {
    for (int i = 0; i < selectedCount; i++) legs[selectedList[i]].startPrimitive(p, dur);
}

// 选中腿是否全部停稳
bool selAllStopped() {
    for (int i = 0; i < selectedCount; i++) {
        if (legs[selectedList[i]].isMoving()) return false;
    }
    return true;
}

bool haveLegs() {
    if (selectedCount > 0) return true;
    Serial.println("! 未选中任何腿，先用 legs 选择，如 legs 0 或 legs 0 1 2");
    return false;
}

void printHelp() {
    Serial.println("--- 可切换多腿测试命令 ---");
    Serial.println("  legs             : 查看当前选中的腿");
    Serial.println("  legs all         : 选中全部 6 条腿");
    Serial.println("  legs <n n ...>   : 选中指定腿 (支持区间)，如 legs 0 / legs 0 1 2 / legs 0-3");
    Serial.println("  cycle            : 选中腿同步连续行走循环（无限）");
    Serial.println("  one              : 选中腿只走一个周期，然后回站立位");
    Serial.println("  each             : 逐腿巡检：选中的每条腿依次走一个完整周期");
    Serial.println("  home             : 选中腿平滑回默认站立位");
    Serial.println("  goto <x> <y>     : 选中腿足端同步移到绝对坐标 (cm)");
    Serial.println("  lift <cm>        : 设置弧线抬腿高度 (默认 3.0)");
    Serial.println("  dur <ms>         : 设置动作时长 (默认 600)");
    Serial.println("  0 <角度>         : 选中腿大腿(FEMUR)舵机同步转到角度 (0~180)");
    Serial.println("  1 <角度>         : 选中腿小腿(TIBIA)舵机同步转到角度 (0~180)");
    Serial.println("  pos              : 打印选中腿足端坐标/状态");
    Serial.println("  servo            : 打印选中腿舵机【无偏转角】(=计算角, 不含零位偏移、不限幅)");
    Serial.println("  servo watch [ms] : 连续打印舵机【无偏转角】（默认200ms，有变化才打一行）；servo off 关闭");
    Serial.println("  stop             : 停止循环，回站立位");
    Serial.println("说明: 选中1条=单腿测试; 选中多条=严格同步一起动; each=逐腿依次测");
}

// ========== 舵机直控（0 x / 1 x） ==========

// 直控命令：选中腿的同一关节舵机同步转到同一角度（0=大腿FEMUR，1=小腿TIBIA）
void servoSelByJoint(int joint, float deg) {
    if (!haveLegs()) return;
    cycling = false;
    oneShot = false;
    eachMode = false;

    // 若腿正在轨迹运动中，先把内部状态冻结在当前位置（取消运动），
    // 否则 update() 会在下一帧用插值角度覆盖刚写入的直控角度
    for (int i = 0; i < selectedCount; i++) {
        int id = selectedList[i];
        if (legs[id].isMoving()) {
            legs[id].setFootPositionImmediate(legs[id].footX(), legs[id].footY());
        }
    }
    for (int i = 0; i < selectedCount; i++) {
        int id = selectedList[i];
        ServoManager::write(id, (ServoManager::Joint)joint, deg);
        rawServoDeg[id][joint]   = deg;      // 记录给 servo 命令打印（直控不动内部坐标）
        rawServoValid[id][joint] = true;
    }
    rawDirty = true;
    Serial.printf("直控: %d条腿的%s舵机 → %.1f° (此后发运动命令会自动先回默认站位)\n",
                  selectedCount,
                  (joint == ServoManager::FEMUR) ? "大腿(FEMUR)" : "小腿(TIBIA)", deg);
}

// 退出直控模式：选中腿瞬间回默认站位并同步内部坐标（供后续轨迹命令使用）
void takeControlBackSel() {
    if (!rawDirty) return;
    rawDirty = false;
    for (int i = 0; i < selectedCount; i++) snapLegHome(selectedList[i]);
    Serial.println("↑ 直控结束：选中腿瞬间回默认站位，内部状态已同步");
}

// ========== 运动命令 ==========

// 选中腿平滑回默认站立位（走直线、不抬腿，避免中途打断时跳变）
void goHomeSel() {
    cycling = false;
    oneShot = false;
    eachMode = false;
    if (!haveLegs()) return;
    if (rawDirty) { takeControlBackSel(); return; }   // 直控中 → 已瞬间回位
    for (int i = 0; i < selectedCount; i++) homeLeg(selectedList[i]);
    Serial.printf("→ %d条腿 home (回站立位)\n", selectedCount);
}

void startCycle() {
    if (!haveLegs()) return;
    oneShot = false;
    eachMode = false;
    cycling = true;
    stepsDone = 0;
    cycleStep = 0;
    takeControlBackSel();           // 直控模式先回站位，从站位开始走
    startPrimitiveSel(CYCLE[0], primDuration);
    Serial.printf("→ %d条腿同步 cycle 开始: %s (%lu ms)\n",
                  selectedCount, primName(CYCLE[0]), (unsigned long)primDuration);
}

void startOneShot() {
    if (!haveLegs()) return;
    cycling = false;
    eachMode = false;
    oneShot = true;
    stepsDone = 0;
    cycleStep = 0;
    takeControlBackSel();           // 直控模式先回站位，从站位开始走
    startPrimitiveSel(CYCLE[0], primDuration);
    Serial.printf("→ %d条腿同步 one 开始: %s (%lu ms)\n",
                  selectedCount, primName(CYCLE[0]), (unsigned long)primDuration);
}

void doGoto(float x, float y) {
    if (!haveLegs()) return;
    cycling = false;
    oneShot = false;
    eachMode = false;
    takeControlBackSel();           // 直控模式先回站位再平滑 goto
    for (int i = 0; i < selectedCount; i++) {
        int id = selectedList[i];
        legs[id].startGrab(legs[id].footX(), legs[id].footY(),
                           x, y, 0.0f, MODE_SWING, primDuration);
    }
    Serial.printf("→ %d条腿同步 goto (%.1f, %.1f)\n", selectedCount, x, y);
}

// ========== 逐腿巡检 each ==========

void startEach() {
    if (!haveLegs()) return;
    if (selectedCount == 1) {
        Serial.println("（当前只选中 1 条腿：each 与 one 等效）");
    }
    cycling = false;
    oneShot = false;
    takeControlBackSel();
    goHomeSel();                     // 所有选中腿先回位（异步），回位后开始逐条测
    eachMode = true;
    eachPending = true;
    eachIndex = 0;
    eachPhase = 0;
    Serial.printf("→ each 逐腿巡检开始: 依次测 %d 条腿（每条完整走一个周期再回位）\n", selectedCount);
}

void stepEach() {
    // 起始阶段：等全部选中腿回位完成
    if (eachPending) {
        if (!selAllStopped()) return;
        eachPending = false;
        if (eachIndex < selectedCount) {
            Serial.printf("-- each: 开始 leg%d --\n", selectedList[eachIndex]);
        }
    }
    if (eachIndex >= selectedCount) {
        eachMode = false;
        Serial.println("each 巡检完成（全部回站立位）");
        return;
    }

    int id = selectedList[eachIndex];
    SingleLeg& L = legs[id];
    if (L.isMoving()) return;       // 等当前腿把本阶段走完

    if (eachPhase == 0) {
        L.startPrimitive(PRIM_ARC_EXTEND, primDuration);
        eachPhase = 1;
        Serial.printf("  leg%d: %s (%lu ms)\n", id, primName(PRIM_ARC_EXTEND),
                      (unsigned long)primDuration);
    } else if (eachPhase == 1) {
        L.startPrimitive(PRIM_LINE_RETRACT, primDuration);
        eachPhase = 2;
        Serial.printf("  leg%d: %s (%lu ms)\n", id, primName(PRIM_LINE_RETRACT),
                      (unsigned long)primDuration);
    } else if (eachPhase == 2) {
        homeLeg(id);
        eachPhase = 3;
        Serial.printf("  leg%d: home\n", id);
    } else {
        eachIndex++;
        eachPhase = 0;
        if (eachIndex < selectedCount) {
            Serial.printf("-- each: 开始 leg%d --\n", selectedList[eachIndex]);
        }
    }
}

// ========== 状态显示 ==========

void printPos() {
    if (selectedCount == 0) {
        Serial.println("当前未选中任何腿（legs ...）");
        return;
    }
    if (selectedCount == 1) {
        // 单腿：完整显示（与旧单腿测试一致）
        int id = selectedList[0];
        LegKinematics kin(robot::LENA, robot::LENB);
        LegAngles ang = kin.ik(legs[id].footX(), legs[id].footY());
        // ik() 返回【弧度】，下面两行乘 180/π 转成度 —— 只换算这一次，是正确的
        // （本编译单元里的 PI 是 Arduino.h 的宏 3.14159…）。
        float thighDeg = ang.thigh * 180.0 / PI;
        float kneeDeg  = ang.knee  * 180.0 / PI;
        // 下面两行是"手算"的舵机角：大腿 = ag2、小腿 = 180 − ag1（与 src/SingleLeg.cpp
        // 同口径，站姿 67.38°/112.62°），但没叠加 Config.h 的 ANGLE_OFFSET（当前整表为 0）、
        // 也没有 0~180 限幅，所以 ≠ ServoManager::write() 真正写下去的角度。
        // 想看含偏移后真正写进舵机的角度：用 @SERVO 遥测，或 test/SingleServoTest.cpp。
        float servoThigh = thighDeg;          // = ag2（实物：0° = 竖直向上、180° = 竖直向下）
        float servoKnee  = 180.0f - kneeDeg;
        Serial.printf("leg%d foot=(%.2f, %.2f) | thigh=%.1f° knee=%.1f° | servo: thigh=%.1f° knee=%.1f° | moving=%s\n",
                      id,
                      legs[id].footX(), legs[id].footY(),
                      thighDeg, kneeDeg,
                      servoThigh, servoKnee,
                      legs[id].isMoving() ? "YES" : "no");
    } else {
        // 多腿：逐条简表
        for (int i = 0; i < selectedCount; i++) {
            int id = selectedList[i];
            Serial.printf("  leg%d foot=(%.2f, %.2f) %s\n",
                          id, legs[id].footX(), legs[id].footY(),
                          legs[id].isMoving() ? "[moving]" : "[stop]");
        }
    }
    if (rawDirty) {
        Serial.println("  (注: 舵机直控模式中，以上坐标不代表实际足端位置)");
    }
}

// ========== 舵机角度打印（servo 命令） ==========
//
// 打印选中腿舵机的【无偏转角】—— 即"计算角"，不含 Config.h::ANGLE_OFFSET、也不限幅，
// 就是交给 ServoManager::write() 的那一刻的角度（口径与 SingleLeg.cpp::writeServosFromRad()
// 严格一致）：
//     ag2 / ag1 = ik() 给出的关节角（弧度）→ ×180/π 转【度】（唯一一次换算）
//     无偏转角 = 大腿: ag2（ag2 为"外倾为正"口径）；小腿: 180 − ag1
// 只打印这一个值：不叠零位偏移、不做 0~180 限幅、不给脉冲计数
// （要看含偏移后真机实际下发的角，用 @SERVO 遥测或 test/SingleServoTest.cpp）。
// ⚠ 大腿口径必须与 src/SingleLeg.cpp::writeServosFromRad() 成对修改：那边是
//   `float servoThigh = thighDeg;`（外倾为正），对应这里 FEMUR_ANGLE_BASE = 0.0f 且仍用加号。
const float FEMUR_ANGLE_BASE = 0.0f;     // 大腿 = ag2 + 0（实物 0° = 竖直向上；站姿 67.38°）
const float TIBIA_ANGLE_BASE = 180.0f;   // 小腿 = 180 − ag1（站姿 112.62°）

const char* jointName(int joint) {
    return (joint == ServoManager::FEMUR) ? "FEMUR(大腿)" : "TIBIA(小腿)";
}

// 某腿某关节"此刻会写入的逻辑角（无偏转角）"（度）：
//   直控模式 → 直控不更新内部足端坐标，用直控时记录的角度（known=false 表示该关节没记录）；
//   其它情况 → 用当前足端坐标反算 IK，与 SingleLeg::update() 每帧写入的完全一致。
float servoLogicalOf(int id, int joint, bool& known) {
    if (rawDirty) {
        known = rawServoValid[id][joint];
        return rawServoDeg[id][joint];
    }
    LegKinematics kin(robot::LENA, robot::LENB);
    LegAngles ang = kin.ik(legs[id].footX(), legs[id].footY());
    known = true;
    if (joint == ServoManager::FEMUR) return FEMUR_ANGLE_BASE + (float)(ang.thigh * 180.0 / PI);
    return TIBIA_ANGLE_BASE - (float)(ang.knee * 180.0 / PI);
}

// 打印一次选中腿的舵机角度明细（每条腿两行：大腿 / 小腿）
void printServoAngles() {
    if (selectedCount == 0) {
        Serial.println("当前未选中任何腿（legs ...）");
        return;
    }
    Serial.println("--- 舵机角度【无偏转角】= 计算角，不含零位偏移（即 ServoManager::write 的入参）---");
    Serial.printf("    口径: 大腿 = %.0f + ag2（= ag2）、小腿 = %.0f − ag1（与 src/SingleLeg.cpp 实际写入一致）\n",
                  FEMUR_ANGLE_BASE, TIBIA_ANGLE_BASE);

    for (int i = 0; i < selectedCount; i++) {
        int id = selectedList[i];
        Serial.printf("  leg%d foot=(%.2f, %.2f) %s\n",
                      id, legs[id].footX(), legs[id].footY(),
                      legs[id].isMoving() ? "[moving]" : "[stop]");
        for (int j = 0; j < 2; j++) {
            bool known = false;
            float logical = servoLogicalOf(id, j, known);
            if (!known) {
                // 直控模式（0 x / 1 x）只动某一关节，另一个关节的直控值没记录过：
                // 此时内部足端坐标也不可信，如实说明比打印一个假数字强
                Serial.printf("    ch%2d %s: 直控模式中该关节未被直控过、角度未记录（home 或发运动命令后恢复）\n",
                              robot::SERVO_PINS[id][j], jointName(j));
                continue;
            }
            Serial.printf("    ch%2d %s: 无偏转 %7.1f°\n",
                          robot::SERVO_PINS[id][j], jointName(j), logical);
        }
    }
    if (rawDirty) {
        Serial.println("  (直控模式中: 以上是直控下发的角度；内部足端坐标未更新，发运动命令或 home 即可退出直控)");
    }
}

// ---- 连续打印（servo watch [ms] / servo off）----
bool servoWatch = false;                   // 是否处于连续打印模式
unsigned long servoWatchInterval = 200;    // 检查间隔（ms），默认 200
unsigned long servoWatchLast = 0;          // 上次检查时刻（0 = 立刻打第一帧）
float servoWatchPrev[6][2] = { { 0, 0 } }; // 上次打印的无偏转角（变化 <0.5° 就不刷屏）
bool  servoWatchHasPrev[6][2] = { { false, false } };
bool  servoWatchNeedHeader = false;        // 是否还需要打印一次口径说明

void servoWatchSet(bool on, unsigned long intervalMs) {
    if (intervalMs >= 20) servoWatchInterval = intervalMs;   // 传 0 = 保持当前间隔
    servoWatch = on;
    if (on) {
        servoWatchLast = 0;                 // 下一次 loop 立刻打一帧
        servoWatchNeedHeader = true;
        for (int l = 0; l < 6; l++)
            for (int j = 0; j < 2; j++) servoWatchHasPrev[l][j] = false;
        Serial.printf("舵机角度连续打印: 开（每 %lu ms 检查，角度有变化才打印；servo off 关闭）\n",
                      servoWatchInterval);
    } else {
        Serial.println("舵机角度连续打印: 关");
    }
}

// loop 每帧调用：到点且角度有变化时，打一行紧凑的实时【无偏转角】
void stepServoWatch() {
    if (!servoWatch || selectedCount == 0) return;
    unsigned long now = millis();
    if (servoWatchLast != 0 && now - servoWatchLast < servoWatchInterval) return;
    servoWatchLast = now;

    // 有一处变化（≥0.5° 或首次）才打，静止时不刷屏
    bool changed = false;
    for (int i = 0; i < selectedCount && !changed; i++) {
        int id = selectedList[i];
        for (int j = 0; j < 2; j++) {
            bool known = false;
            float f = servoLogicalOf(id, j, known);   // 变化判断也用无偏转角
            if (!known) continue;                                  // 直控未记录的关节不参与比较
            if (!servoWatchHasPrev[id][j] || fabsf(f - servoWatchPrev[id][j]) >= 0.5f) {
                changed = true;
                break;
            }
        }
    }
    if (!changed) return;

    if (servoWatchNeedHeader) {
        servoWatchNeedHeader = false;
        Serial.println("--- 舵机角度实时打印【无偏转角】（F=大腿 T=小腿；不含零位偏移，= 写入舵机的入参）---");
    }
    Serial.printf("[watch %lus]", now / 1000);
    for (int i = 0; i < selectedCount; i++) {
        int id = selectedList[i];
        Serial.printf(" | leg%d", id);
        for (int j = 0; j < 2; j++) {
            bool known = false;
            float logical = servoLogicalOf(id, j, known);
            if (!known) {
                Serial.printf(" %s未记录", (j == ServoManager::FEMUR) ? "F" : "T");
                continue;
            }
            // 连续打印只显示无偏转角（含偏移的写入角看 @SERVO 遥测）
            servoWatchPrev[id][j]    = logical;
            servoWatchHasPrev[id][j] = true;
            Serial.printf(" %s%.1f", (j == ServoManager::FEMUR) ? "F" : "T", logical);
        }
    }
    Serial.println();
}

// ========== 串口命令 ==========

void handleCommand(const String& line) {
    String cmd = line; cmd.trim();
    if (cmd.length() == 0) return;

    int sp = cmd.indexOf(' ');
    String first = (sp < 0) ? cmd : cmd.substring(0, sp);
    String rest  = (sp < 0) ? ""  : cmd.substring(sp + 1);
    first.toLowerCase();
    rest.trim();

    if (first == "help")    { printHelp(); return; }
    if (first == "legs" || first == "sel") {
        if (rest.length() == 0) printSelection();
        else applySelection(rest);
        return;
    }
    if (first == "each")    { startEach(); return; }
    if (first == "cycle")   { startCycle(); return; }
    if (first == "one")     { startOneShot(); return; }
    if (first == "home")    { goHomeSel(); return; }
    if (first == "stop")    { goHomeSel(); return; }
    if (first == "pos")     { printPos(); return; }

    // 舵机角度打印：servo（打一次明细） / servo watch [ms]（连续打印） / servo off（关闭）
    if (first == "servo" || first == "ang" || first == "angles") {
        String arg = rest; arg.toLowerCase(); arg.trim();
        if (arg.length() == 0) { printServoAngles(); return; }
        if (arg == "off") { servoWatchSet(false, 0); return; }
        int sp2 = arg.indexOf(' ');
        String key = (sp2 < 0) ? arg : arg.substring(0, sp2);
        String val = (sp2 < 0) ? ""  : arg.substring(sp2 + 1);
        key.trim(); val.trim();
        if (key == "on" || key == "watch") {
            servoWatchSet(true, (val.length() > 0) ? (unsigned long)val.toInt() : 0);   // 0 = 用默认间隔
            return;
        }
        if (isAllDigits(key)) {                 // 简写：servo 300 → 每 300ms 连续打印
            servoWatchSet(true, (unsigned long)key.toInt());
            return;
        }
        Serial.println("! 用法: servo | servo watch [ms] | servo off");
        return;
    }

    if (first == "goto") {
        float x = 0, y = 0;
        if (sscanf(rest.c_str(), "%f %f", &x, &y) == 2) doGoto(x, y);
        else Serial.println("! 用法: goto <x> <y>");
        return;
    }
    if (first == "lift") {
        float v = 0;
        if (sscanf(rest.c_str(), "%f", &v) == 1) {
            arcLiftCm = v;
            for (int i = 0; i < selectedCount; i++) legs[selectedList[i]].setArcLift(v);
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

    // 舵机直控：0 <角度> → 选中腿大腿；1 <角度> → 选中腿小腿
    if (first == "0" || first == "1") {
        int joint = first.toInt();
        float ang = -1;
        if (sscanf(rest.c_str(), "%f", &ang) == 1) {
            servoSelByJoint(joint, constrain(ang, 0.0f, 180.0f));
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

    Serial.printf("=== 可切换多腿测试: 站立高度 %.1fcm ===\n", STAND_HEIGHT);

    // 应用默认腿选择（内部会对选中腿逐条 init 回默认站立位）
    applySelection(DEFAULT_SELECTION);

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

    // 2) 推进所有已上电的腿（选中与否都走完当前动作，避免停在中途）
    for (int i = 0; i < 6; i++) {
        if (legInited[i]) legs[i].update();
    }

    // 2.5) 舵机角度连续打印（servo watch [ms]；未开启时立即返回，不影响时序）
    stepServoWatch();

    // 3) 逐腿巡检状态机
    if (eachMode) { stepEach(); return; }

    // 4) 选中腿全部停稳后，按模式安排下一步（以最慢的腿为准，保证严格同步）
    if (selAllStopped()) {
        if (cycling) {
            cycleStep = (cycleStep + 1) % CYCLE_STEPS;
            startPrimitiveSel(CYCLE[cycleStep], primDuration);
        } else if (oneShot) {
            stepsDone++;
            if (stepsDone >= CYCLE_STEPS) {
                oneShot = false;
                Serial.println("one 周期完成 → home");
                goHomeSel();
            } else {
                cycleStep++;
                startPrimitiveSel(CYCLE[cycleStep], primDuration);
            }
        }
    }

    delay(10);
}
