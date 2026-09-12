/* 六方向步态全功能测试（串口交互式，一个固件全搞定）
 *
 * = 把 test/SingleLegTest.cpp 的【全部功能】+【三角步态】+【6 个行走方向】合到一个文件里 =
 *
 * ── 三块能力 ────────────────────────────────────────────
 *  A) 单腿 / 多腿全部功能（与 SingleLegTest.cpp 完全等价，命令逐条保留）
 *       运行时 `legs` 任意选 1~6 条腿；cycle / one / each / home / goto / lift / dur /
 *       0|1 舵机直控 / pos / servo [watch|off] / stop 全部照旧。
 *  B) 三角步态（引擎 = src/TripodGait.cpp 的 TripodGait，本文件不再自带相位机）
 *       `tripod` 连续跑、`tri one` 跑一个完整周期、`tri step` 只跑一拍。
 *       类用"外部腿"构造函数绑到本文件的 legs[6]，pos / servo / 直控 与步态共用同一批对象。
 *  C) 6 个行走方向 = TripodGait 的 SequentialOffset（0..5 → 0°/60°/120°/180°/240°/300°）
 *       方向快捷键直接开跑；换方向 = 停→回站姿→按新方向重新起步（sqo 构造后固定）。
 *
 * ── 运行方式（PlatformIO）──────────────────────────────
 *   1. platformio.ini 的 build_flags 设为：-DMODE_GAIT_6DIR
 *   2. Upload 烧录，打开串口监视器（115200 波特率）
 *   3. 发命令（每行一条，回车结尾）—— 完整列表见 `help`：
 *
 *      【选腿 / 单腿·多腿测试（= SingleLegTest）】
 *        help                → 打印命令帮助
 *        legs                → 打印当前选中的腿
 *        legs all            → 选中全部 6 条腿
 *        legs <n n ...>      → 选中指定腿，如 "legs 0"、"legs 0 1"、"legs 1 3 5"，支持区间 "legs 0-3"
 *        cycle               → 选中腿同步连续行走循环（摆动相抬腿前迈 + 支撑相贴地收回）
 *        one                 → 选中腿只走一个完整周期，然后自动回站立位
 *        each                → 逐腿巡检：选中的每条腿依次完整走一个周期再回位
 *        home                → 停所有模式，回默认站立位
 *        goto <x> <y>        → 选中腿足端同步移动到绝对坐标 (x, y)（cm，各腿自身坐标系，原点=髋关节）
 *        lift <cm>           → 弧线抬腿高度（三角步态的摆动抬腿也用它）
 *        dur <ms>            → 每个动作 / 三角步态每拍的时长（默认 600）
 *        0 <角度>            → 选中腿 大腿(FEMUR) 舵机同步转到角度 (0~180)
 *        1 <角度>            → 选中腿 小腿(TIBIA) 舵机同步转到角度 (0~180)
 *        pos                 → 打印选中腿足端坐标/关节角与状态（三角步态时附每腿槽位/相位）
 *        servo               → 打印选中腿舵机的【无偏转角】（=计算角，不含零位偏移、不限幅）
 *        servo watch [ms]    → 连续打印【无偏转角】（默认 200ms，有变化才打一行）；servo off 关闭
 *        stop                → 停所有模式并平滑回站立位
 *
 *      【三角步态（引擎：src/TripodGait.cpp 的 TripodGait）】
 *        tripod | tri        → 开始连续三角步态（用当前方向）
 *        tri one             → 只走一个完整三角周期（两拍）后回站立位
 *        tri step            → 只走一拍（调试相位/接线用）
 *        tri stop            → 停止（等价 stop）
 *        phase               → 打印状态：方向/sqo/拍数/两张相位表
 *        verify              → 方向自检（纯数学：sqo=0..5 → 净推力方向，不需要硬件）
 *        stride <cm>         → 仅提示：TripodGait 行程固定（全行程 ≈6.77cm），无此参数
 *
 *      【6 个行走方向（0° = 机头方向；= SequentialOffset 0..5，60° 一档）】
 *        f    =   0°  前      fl =  60°  左前     bl = 120°  左后
 *        b    = 180°  后      br = 240°  右后     fr = 300°  右前
 *        dir <角度>          → 任意角度会吸附到最近的 60° 档（如 dir 90 → 60° 或 120°）
 *        dirs                → 打印方向表（含每个方向用的 sqo）
 *        mirror              → 翻转腿方位角镜像（实测走反了/腿序与假设相反时用）
 *
 * ── 三角步态：引擎就是 src/TripodGait.cpp 的 TripodGait ──────────
 *   本文件不自己写相位机，直接驱动 TripodGait：
 *       TripodGait gait(legs, sqo, dur);   // ← 用"外部腿"构造函数，绑到本文件的 legs[6]
 *       gait.TriGaitInit();                // 6 腿回站姿（会写舵机 + 每条腿 delay(100)）
 *       loop: gait.TriGaitWork();          // 每帧推进；6 腿全部停稳后自动切下一拍
 *   绑外部腿的好处：步态驱动的就是本文件这一批 SingleLeg 对象 ⇒ pos / servo / servo watch /
 *   舵机直控 / goto / each 读到的足端坐标就是步态真正在用的坐标，不会出现"两套账"。
 *   （老构造函数 TripodGait(sqo, dur) 仍在：那种用法下类自己新建 6 条腿、与上层不共享状态。）
 *
 *   ⚠ 起步故意**不调 TriGaitStart()**：它发完第一拍 A 之后，TriGaitWork() 里 mode==0 分支还会
 *     再发一次 A 组 —— 那一拍打在"已经到位的腿"上（腿都停在上一次 A 的终点、目标点相同），
 *     没有水平位移、只有摆动相那三条腿原地抬一下，等于白走半拍。直接从 Init → 反复 Work()，
 *     第一次 Work() 就会发第一拍，之后序列严格 A,B,A,B…（**不用改 TripodGait.cpp**）。
 *
 * ── 6 个行走方向 = TripodGait 的 SequentialOffset（60° 一档）────────
 *   本构型每条腿只有 2 个自由度、没有髋部偏转关节，足端只能在自身径向竖直平面里伸缩 ——
 *   所以"往哪个方向走"是靠**整张相位表绕机体旋转**实现的：
 *
 *       sqo = k  →  相位表按腿序循环挪 k 档：物理腿 = (槽位 + k) mod 6
 *                   净推力方向 = 60°×k（两拍同向，大小 = 2×全行程）
 *
 *       k=0 →   0° 前        k=1 →  60° 左前      k=2 → 120° 左后
 *       k=3 → 180° 后        k=4 → 240° 右后      k=5 → 300° 右前
 *
 *   为什么"旋转相位表"就能转向：三足支撑组里 d（支撑相"收"）把机体推向该腿朝外方向 +ê，
 *   c（支撑相"伸"）推向 −ê ⇒ 净推力 = Σ(槽位 d 的腿)+ê − Σ(槽位 c 的腿)ê；
 *   把整张相位表挪一档 ≡ 把这个矢量和整体转 60°。`verify` 命令会逐档算给你看
 *   （sqo=0..5 → 0°/60°/…/300°，纯数学、不需要硬件）。
 *
 *   sqo=0 时两张相位表就是文档里的经典表（a=弧线张 b=弧线收 c=直线张 d=直线收；数字=腿号）：
 *       第一拍：0d 1a 2c 3b 4c 5a          第二拍：0a 1d 2b 3c 4b 5d
 *
 *   ⚠ sqo 与 dur 都是"构造后固定"：换方向 / 改时长 = 重建 TripodGait 对象，因此本文件换方向
 *     的动作是【停 → 6 腿回站姿 → 按新方向重新起步】，不是边走边转；方向分辨率 60°
 *     （`dir <角度>` 会吸附到最近的 60° 档并提示）。
 *
 * ── 腿方位角与"左/右"的约定（走反了先看这里）────────────────
 *   默认假设装机顺序是【俯视逆时针】：leg0 在正前，leg1 左前、leg2 左后、leg3 正后、
 *   leg4 右后、leg5 右前（方位角 αᵢ = i·60°）。若实物是顺时针装的（序号绕着机体反着走），
 *   同一个 sqo 会走出镜像方向 —— 发一次 `mirror`（≡ 把 sqo 取 (6−k)%6）即可，
 *   或者把文件里的 AZIMUTH_SIGN 初值改成 −1。
 *   净推力恒为 2×全行程、方向恒为 60°×k，这一点只依赖"6 条腿 60° 均布"，与从哪条腿开始数无关。
 *
 * ── 上电行为 / 安全 ────────────────────────────────────
 *   - setup 不自动开跑，只按 DEFAULT_SELECTION 把选中的腿 init() 到默认站姿（会写舵机！
 *     一次只动该腿 2 个舵机、每条腿写完 delay(100)）→ **务必先把机器人架起来再上电/发命令**；
 *   - 三角步态固定使用【全部 6 条腿】，与当前 `legs` 选择无关（两组支撑缺一不可）；
 *     没上电过的腿会被自动 init() 到站姿并打印提示；
 *   - 6 条腿 / 12 路舵机同时动的电流很大（峰值数安培），舵机电源必须独立大电流 5V 并与 ESP32 共地；
 *   - 先跑"单舵机 → 单腿 → 六腿同步 → 三角步态"的顺序排查，定位问题快得多；
 *   - goto 的坐标要落在可达范围（内圆 |6.5−12.5| = 6cm，外圆 6.5+12.5 = 19cm）内；
 *     默认站立足端 (−6, −10)（d ≈ 11.66cm）与伸出足端 (−12.77, −10)（d ≈ 16.22cm）都在范围内；
 *   - TripodGait 的行程固定 = 站立↔伸出（全行程 ≈ 6.77cm），没有 stride 可调；
 *     理想无滑移时每拍机体前进 2×6.77 ≈ 13.5cm（两拍一个周期 ≈27cm）—— 真机会打滑，实测为准；
 *   - 三角步态起步时是先回站姿再走：第一拍里"该支撑的腿"本来就在站位、该摆动的腿抬腿前伸，
 *     所以第一下有半拍是过渡（从第二拍起才是严格稳态）。
 *
 * ── 与 SingleLegTest.cpp 的差异（就这几条）────────────────
 *   1) 新增三角步态 / 方向相关命令（见上），原有命令的名字、参数、语义一字未改；
 *   2) `home` / `stop` / `legs ...` / `goto` / `cycle` / `one` / `each` / `0|1` 这些命令
 *      在三角步态运行中发过来时，会先把三角步态停掉：
 *        · 直控类（0/1）与其它运动类：6 条腿**瞬间**回默认站位再执行（与"直控退出"同一套做法，
 *          保证内部坐标一致、不会从半截步态跳变）；
 *        · home / stop：6 条腿**平滑**回站位；
 *   3) `pos` 在三角步态运行时多打几行每腿槽位/相位（原有输出一行没变）；
 *   4) `stop` / `home` 在三角步态运行中会停下步态、让 6 条腿【平滑】回站立位（走的是本文件的
 *      goHomeAll，不调 TriGaitStop —— 那个是"瞬间写站位"）；被其它运动命令/直控打断时则走
 *      TripodGait::TriGaitStop()（瞬间回站位 + mode 复位）再 snapHomeAll() 兜住它
 *      "不清 isMovingFlag"的坑（不兜的话下一帧 update() 会把腿又拉回上一拍的目标点）；
 *   5) `servo` / `servo watch` / 遥测口径与 SingleLegTest.cpp 完全一致（见下）。
 *
 * ── 这个文件是怎么被编译的 ─────────────────────────────
 *   本文件【不是 PlatformIO 单元测试】：src/main.cpp 里有
 *       #elif defined(MODE_GAIT_6DIR) … #include "../test/Gait6DirTest.cpp"
 *   即它被直接包含进 main.cpp 所在的同一个编译单元（同一个 TU），setup()/loop()/全局变量
 *   与 main.cpp 共享 —— 因此这里用到的 PI 是 Arduino.h 的宏（3.14159…），
 *   千万别在文件作用域写 `const double PI = …;`（宏展开直接编译失败）。
 *
 * ── 数字孪生遥测 / 舵机角度打印（与 SingleLegTest 同口径）──────
 *   本文件不自带遥测：所有写舵机都走 ServoManager::write()，该函数在固件层内置节流回传
 *       @SERVO:<ch>:<最终角>      例： @SERVO:0:67.4
 *   （最终角 = 逻辑角 + 零位偏移，再限幅 0~180；角度变化 ≥0.5° 且每通道 ≥50ms 一条）。
 *   gait-sim「悬空模式」解析该行即可 1:1 镜像真机 12 路舵机姿态。
 *   `servo` / `servo watch` 打印的是【无偏转角】= 大腿 ag2 / 小腿 180−ag1（=计算角，
 *   不含 Config.h::ANGLE_OFFSET、不限幅），即交给 ServoManager::write() 的那一刻的角度。
 *
 *   本文件为自包含版，也可在 Arduino IDE 里与以下源文件一起编译：
 *     include/Config.h、include/LegKinematics.h、include/ServoManager.h、include/SingleLeg.h、
 *     include/TripodGait.h、src/LegKinematics.cpp、src/ServoManager.cpp、src/SingleLeg.cpp、
 *     src/TripodGait.cpp
 *   （三角步态由 src/TripodGait.cpp 提供 —— 本文件用它的"外部腿"构造函数绑到自己的 legs[6]。
 *     TripodGait.h 里那个带 SingleLeg* 的构造函数与 steps() 是 2026-09 加的纯增量接口，
 *     老构造函数 TripodGait(sqo, dur) 的行为一字未变，src/main.cpp 的 -DMODE_TRIPOD 照旧。）
 */

#include <Arduino.h>
#include "SingleLeg.h"
#include "ServoManager.h"
#include "LegKinematics.h"
#include "TripodGait.h"     // ← 三角步态引擎（src/TripodGait.cpp）
#include "Config.h"
#include <math.h>

// ============================================================
// 1. 硬件上的 6 条腿
// ============================================================
// 腿号 0~5 → PCA9685 通道 leg*2 + joint（joint: 0=大腿 FEMUR、1=小腿 TIBIA）
SingleLeg legs[6] = {
    SingleLeg(0), SingleLeg(1), SingleLeg(2),
    SingleLeg(3), SingleLeg(4), SingleLeg(5)
};
bool legInited[6] = { false, false, false, false, false, false };

// ============================================================
// 2. 测试参数
// ============================================================
const char DEFAULT_SELECTION[] = "all";        // 上电默认选中的腿（运行中用 `legs ...` 切换）
const float STAND_HEIGHT = 10.0f;              // 站立高度（cm）；⚠ 只被 init() 记录，站姿恒为 (−6, −10)
const unsigned long DEFAULT_DURATION_MS = 600; // 单个动作 / 三角步态每拍默认时长
const float DEFAULT_ARC_LIFT = 5.0f;           // 弧线/摆动相默认抬腿高度（cm）

// ============================================================
// 3. 腿方位角 / 6 个行走方向表
// ============================================================
// AZIMUTH_SIGN = +1：俯视逆时针，leg0 正前 → 1=左前 2=左后 3=正后 4=右后 5=右前
//              = −1：镜像（腿号顺时针排列）—— 实物走反了用 `mirror` 命令热切换
int AZIMUTH_SIGN = 1;

// 第 id 条腿的方位角（度，0~360，0 = 机头方向）
float legAzimuthDeg(int id) {
    float a = fmodf((float)id * 60.0f * (float)AZIMUTH_SIGN, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}

// 6 个行走方向（φ 与腿方位角同一套坐标：0° = 机头方向 = leg0 的径向）
struct DirEntry {
    const char* key;    // 串口快捷键
    const char* name;   // 显示名
    float phi;          // 方向角（度）
};
const DirEntry DIRS[6] = {
    { "f",  "前",     0.0f },
    { "fl", "左前",  60.0f },
    { "bl", "左后", 120.0f },
    { "b",  "后",   180.0f },
    { "br", "右后", 240.0f },
    { "fr", "右前", 300.0f }
};

// ============================================================
// 4. 选中集合（与 SingleLegTest.cpp 相同）
// ============================================================
bool legSelected[6] = { false, false, false, false, false, false };
int  selectedList[6];      // 当前选中的腿号（升序）
int  selectedCount = 0;

// ============================================================
// 5. 内部状态
// ============================================================
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
// 所以只能在这里记录，供 servo 命令如实打印
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

// ============================================================
// 6. 三角步态状态（引擎：src/TripodGait.cpp 的 TripodGait）
// ============================================================
// 方向 = TripodGait 的 SequentialOffset（构造参数）：sqo = k → 整张相位表按腿序挪 k 档
//（物理腿 = (槽位 + k) mod 6），机体被推往 60°×k —— 6 档就是 6 个行走方向。
// sqo 与 stepDuration 都是"构造后固定"，所以换方向 / 改 dur = 重建对象（先停回站姿再起步）。
// （腿方位角制式 AZIMUTH_SIGN 定义在上面第 3 节，与 legAzimuthDeg() 放在一起。）
int  triDirIndex = 0;       // 当前方向档 0..5 → φ = 60°×index（前/左前/左后/后/右后/右前）

TripodGait* triGait = nullptr;  // 运行中的步态引擎（换方向 / 改时长时重建）
bool triOn = false;             // 三角步态是否在跑
bool triRun = false;            // true = 连续跑；false = 只跑 triPhaseTarget 拍
int  triPhaseTarget = 0;        // 有限模式要跑的拍数（tri one = 2 拍 = 一个完整周期）
int  triLastSteps = 0;          // 已观察到的拍数（triGait->steps() 的快照）

// 几何量（由 SingleLeg 的站立足端/伸出足端推出，setup 里算一次；只用于打印与自检）
float footStandX = -6.0f, footStandY = -10.0f;   // 默认站立足端
float footExtX   = -12.77f, footExtY = -10.0f;   // 伸出足端
float rStand = 6.0f;      // 站立足端"髋外径向距离" = −x
float rExt   = 12.77f;    // 伸出足端"髋外径向距离"
float strokeR = 6.77f;    // TripodGait 的全行程（= rExt − rStand）

// TripodGait 的两张固定相位表（按"槽位"记；物理腿 = (槽位 + sqo) mod 6）—— 与 .cpp 一一对应：
//   第一拍 = TriGaitStart() / TriGaitWork() 的 mode==0 分支（文档 A 组 0d 15a 3b 24c）
//   第二拍 = TriGaitWork() 的 mode==1 分支（文档 B 组 0a 15d 3c 24b）
const char SLOT_A[6] = { 'd', 'a', 'c', 'b', 'c', 'a' };
const char SLOT_B[6] = { 'a', 'd', 'b', 'c', 'b', 'd' };

void triStopGait();   // 前置声明（定义在下面的"三角步态"小节）

bool  triActive() { return triOn; }
int   triSqo() { return (AZIMUTH_SIGN > 0) ? (triDirIndex % 6) : ((6 - triDirIndex) % 6); }
float triPhiDeg() { return 60.0f * (float)triDirIndex; }
const char* dirNameOfIndex(int k) { return DIRS[((k % 6) + 6) % 6].name; }
int   slotOfLeg(int id) { return ((id - triSqo()) % 6 + 6) % 6; }   // 该腿此刻占哪个槽位

// 归一化到 [0, 360)
float normDeg360(float a) {
    a = fmodf(a, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a;
}
// 归一化到 (−180, 180]
float normDeg180(float a) {
    a = fmodf(a + 180.0f, 360.0f);
    if (a < 0.0f) a += 360.0f;
    return a - 180.0f;
}

// ========== 选中集合：解析 / 应用（与 SingleLegTest.cpp 相同）==========

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

// 让某条腿立刻"接管"回默认站位（直控/退出步态用，瞬间到达）
void snapLegHome(int id) {
    double hx, hy;
    SingleLeg::defaultStandingFoot(hx, hy);
    legs[id].setFootPositionImmediate((float)hx, (float)hy);
}

// ---- 6 条腿级别的辅助（三角步态用）----
bool allSixStopped() {
    for (int i = 0; i < 6; i++) if (legs[i].isMoving()) return false;
    return true;
}

void goHomeAll() {
    for (int i = 0; i < 6; i++) homeLeg(i);
}

void snapHomeAll() {
    for (int i = 0; i < 6; i++) snapLegHome(i);
}

// 6 条腿全部确保上电（三角步态需要两组支撑都到位）
void ensureAllLegsInited() {
    for (int i = 0; i < 6; i++) {
        if (!legInited[i]) {
            Serial.printf("↑ leg%d 首次上电：回默认站立位（三角步态需要全部 6 条腿）\n", i);
            legs[i].init(STAND_HEIGHT);
            legs[i].setArcLift(arcLiftCm);
            legInited[i] = true;
        }
    }
}

// ========== 模式互斥：任何一次"开新动作"都先把别的模式停掉 ==========

// 只清模式标志（不动腿）—— 需要"平滑回位"而不是"瞬间回位"的命令自己用它
void clearModes() {
    cycling = false;
    oneShot = false;
    eachMode = false;
    eachPending = false;
    triOn = false;
    triRun = false;
    triPhaseTarget = 0;
}

// 三角步态在跑时被打断：停引擎并让 6 条腿【瞬间】回默认站位（与"直控退出"同一套做法）
// —— 这样后续任何命令都从一个已知的一致状态开始，不会从半截步态跳变。
bool yieldTripod() {
    if (!triActive()) return false;
    clearModes();
    triStopGait();          // TripodGait::TriGaitStop()（回站姿）+ snapHomeAll() 兜底
    Serial.println("↑ 三角步态结束（6 腿回默认站位，内部坐标已同步）");
    return true;
}

// 开新动作前的统一入口：先让三角步态让位（若在跑 → 6 腿瞬间回默认站位），再清模式标志。
// ⚠ 必须先 yield 再清标志：yieldTripod() 靠 triActive() 判断，标志先清就判断不出来了。
void stopAllModes() {
    yieldTripod();
    clearModes();
}

// 应用新的腿选择：取消运行模式 → 被移出的腿平滑回位 → 更新集合 → 新腿首次上电回位
void applySelection(const String& desc) {
    int nl[6];
    int n = parseSelection(desc, nl);
    if (n < 0) {
        Serial.println("! legs 语法: legs all | legs <n n ...> | legs <a-b>  例: legs 0 / legs 0 1 2 / legs 0-3");
        return;
    }

    stopAllModes();                 // 含：三角步态在跑 → 6 腿瞬间回位
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

// ========== 帮助 / 方向表 ==========

void printHelp() {
    Serial.println("--- 六方向步态全功能测试命令 ---");
    Serial.println("【选腿 / 单腿·多腿测试（与 SingleLegTest 相同）】");
    Serial.println("  help             : 本帮助");
    Serial.println("  legs             : 查看当前选中的腿");
    Serial.println("  legs all         : 选中全部 6 条腿");
    Serial.println("  legs <n n ...>   : 选中指定腿 (支持区间)，如 legs 0 / legs 0 1 2 / legs 0-3");
    Serial.println("  cycle            : 选中腿同步连续行走循环（无限）");
    Serial.println("  one              : 选中腿只走一个周期，然后回站立位");
    Serial.println("  each             : 逐腿巡检：选中的每条腿依次走一个完整周期");
    Serial.println("  home             : 停所有模式，平滑回默认站立位");
    Serial.println("  goto <x> <y>     : 选中腿足端同步移到绝对坐标 (cm)");
    Serial.printf("  lift <cm>        : 弧线/摆动抬腿高度 (当前 %.1f)\n", DEFAULT_ARC_LIFT);
    Serial.println("  dur <ms>         : 每个动作 / 三角步态每拍的时长 (默认 600)");
    Serial.println("  0 <角度>         : 选中腿大腿(FEMUR)舵机同步转到角度 (0~180)");
    Serial.println("  1 <角度>         : 选中腿小腿(TIBIA)舵机同步转到角度 (0~180)");
    Serial.println("  pos              : 打印选中腿足端坐标/状态（三角步态时附每腿槽位/相位）");
    Serial.println("  servo            : 打印选中腿舵机【无偏转角】(=计算角, 不含零位偏移、不限幅)");
    Serial.println("  servo watch [ms] : 连续打印【无偏转角】（默认200ms，有变化才打一行）；servo off 关闭");
    Serial.println("  stop             : 停所有模式并回站立位");
    Serial.println("【三角步态（引擎 = src/TripodGait.cpp 的 TripodGait）】");
    Serial.println("  tripod | tri     : 开始连续三角步态（用当前方向）");
    Serial.println("  tri one          : 只走一个完整三角周期（两拍）后回站立位");
    Serial.println("  tri step         : 只走一拍（调试相位/接线用）");
    Serial.println("  tri stop         : 停止（等价 stop）");
    Serial.println("  phase            : 打印状态（方向/sqo/拍数/两张相位表）");
    Serial.println("  stride <cm>      : 无此参数（TripodGait 行程固定 = 站立↔伸出全行程），只作提示");
    Serial.println("  verify           : 方向自检（纯数学：sqo=0..5 → 净推力方向，不需要硬件）");
    Serial.println("【6 个行走方向（0°=机头方向；= SequentialOffset 0..5，60° 一档）】");
    Serial.println("  f=  0°前   fl= 60°左前   bl=120°左后");
    Serial.println("  b=180°后   br=240°右后   fr=300°右前");
    Serial.println("  dir <角度>       : 任意角度会吸附到最近一档（如 dir 90 → 60°/120°）；dirs 看方向表");
    Serial.println("  mirror           : 翻转腿方位角镜像（实测走反了/腿序与假设相反时用）");
    Serial.println("  注: 换方向 = 停→回站姿→按新方向重新起步（sqo 构造后固定），不是边走边转");
    Serial.println("说明: 选中1条=单腿测试; 选中多条=严格同步一起动; each=逐腿依次测; tripod=真行走");
}

void printDirTable() {
    Serial.println("--- 6 个行走方向（三角步态；0° = 机头方向，逆时针为正）---");
    Serial.println("  方向 = TripodGait 的 SequentialOffset（60° 一档，sqo 构造后固定）");
    for (int i = 0; i < 6; i++) {
        int sqo = (AZIMUTH_SIGN > 0) ? i : ((6 - i) % 6);
        Serial.printf("  %-2s = %3.0f°  %s  (用 sqo=%d；同向于 leg%d 的方位角)\n",
                      DIRS[i].key, DIRS[i].phi, DIRS[i].name, sqo, i);
    }
    Serial.printf("  当前: 方向 %.0f° (%s) → sqo=%d | 方位角制式: %s\n",
                  triPhiDeg(), dirNameOfIndex(triDirIndex), triSqo(),
                  (AZIMUTH_SIGN > 0) ? "逆时针 leg1=左前" : "顺时针镜像 leg1=右前");
    Serial.println("  只有 60° 的 6 档；`dir <度>` 会吸附到最近一档；`mirror` 整体镜像；`verify` 自检");
}

// ========== 舵机直控（0 x / 1 x） ==========

// 直控命令：选中腿的同一关节舵机同步转到同一角度（0=大腿FEMUR，1=小腿TIBIA）
void servoSelByJoint(int joint, float deg) {
    if (!haveLegs()) return;
    stopAllModes();                 // 含：三角步态在跑 → 6 腿瞬间回位（不在半截步态上跳变）

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

// 退出直控模式（全部 6 条腿，三角步态用）
void takeControlBackAll() {
    if (!rawDirty) return;
    rawDirty = false;
    snapHomeAll();
    Serial.println("↑ 直控结束：6 腿瞬间回默认站位，内部状态已同步");
}

// ========== 运动命令（选中腿，与 SingleLegTest 相同）==========

// 选中腿平滑回默认站立位（走直线、不抬腿，避免中途打断时跳变）
// 三角步态在跑时：停下并让【6 条腿】一起平滑回位（步态动过全部 6 条腿）
void goHomeSel() {
    bool triWas = triActive();
    clearModes();
    if (triWas) {
        Serial.println("↑ 三角步态结束 → 6 腿平滑回站立位");
        goHomeAll();
        return;
    }
    if (!haveLegs()) return;
    if (rawDirty) { takeControlBackSel(); return; }   // 直控中 → 已瞬间回位
    for (int i = 0; i < selectedCount; i++) homeLeg(selectedList[i]);
    Serial.printf("→ %d条腿 home (回站立位)\n", selectedCount);
}

void startCycle() {
    if (!haveLegs()) return;
    stopAllModes();
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
    stopAllModes();
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
    stopAllModes();
    takeControlBackSel();           // 直控模式先回站位再平滑 goto
    for (int i = 0; i < selectedCount; i++) {
        int id = selectedList[i];
        legs[id].startGrab(legs[id].footX(), legs[id].footY(),
                           x, y, 0.0f, MODE_SWING, primDuration);
    }
    Serial.printf("→ %d条腿同步 goto (%.1f, %.1f)\n", selectedCount, x, y);
}

// ========== 逐腿巡检 each（与 SingleLegTest 相同）==========

void startEach() {
    if (!haveLegs()) return;
    if (selectedCount == 1) {
        Serial.println("（当前只选中 1 条腿：each 与 one 等效）");
    }
    yieldTripod();
    clearModes();
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

// ========== 三角步态（引擎: src/TripodGait.cpp 的 TripodGait）==========

// 重建引擎：方向（sqo）与每拍时长都是构造后固定的 → 换方向 / 改 dur 都要重建对象
void rebuildGait() {
    if (triGait) { delete triGait; triGait = nullptr; }
    triGait = new TripodGait(legs, triSqo(), (int)primDuration);   // ← 用"外部腿"构造：绑本文件的 legs[6]
}

// 停引擎并回站姿：走它自己的 TriGaitStop()（回站位 + mode 复位），再 snapHomeAll() 兜底
void triStopGait() {
    if (!triGait) return;
    triGait->TriGaitStop();   // ⚠ 类自己的已知坑：它不清 SingleLeg 的 isMovingFlag
    snapHomeAll();            // 同步内部坐标 + 清 isMovingFlag，
                              // 否则下一帧 update() 会按上一拍的旧目标把腿又拉走
}

// 相位表 → 字符串（按腿号 0..5 排，方便与文档 / 项目认知.md 的经典表对照）
String phasePatternByLeg(const char* slotTable) {
    int sqo = triSqo();
    String s;
    for (int leg = 0; leg < 6; leg++) {
        int slot = ((leg - sqo) % 6 + 6) % 6;
        s += String(leg) + slotTable[slot];
        if (leg < 5) s += ' ';
    }
    return s;
}

void triFinish() {
    triOn = false;
    triRun = false;
    triPhaseTarget = 0;
    Serial.println("三角步态结束 → 6 腿平滑回站立位");
    goHomeAll();
}

// 启动三角步态：continuous = 无限跑；否则只走 phases 拍（tri one = 2 拍 = 一个完整周期，tri step = 1 拍）
void triStart(bool continuous, int phases) {
    // 三角步态若已经在跑：直接重建引擎就行（旧对象 delete 掉，不必先 TriGaitStop 再 Init，
    // 省一次 6×delay(100)）；triOn 先清掉，下面 stopAllModes() 就不会再重复停一遍。
    triOn = false;
    triRun = false;
    triPhaseTarget = 0;
    stopAllModes();                 // 停 cycle / one / each
    if (rawDirty) takeControlBackAll();
    ensureAllLegsInited();          // 本文件的 6 条腿都上电（init 会写站姿）
    // 先 snapHomeAll()：把 6 条腿钉在站位并**清掉可能残留的 isMovingFlag**
    // （例如刚从 cycle/each 打断过来时，标志还是 true、目标点还是旧的）——
    // 不这么做的话，引擎的第一次 TriGaitWork() 会按旧目标把腿拉走。
    snapHomeAll();
    rebuildGait();                  // 按当前方向档 sqo + 当前 dur 建引擎
    triGait->TriGaitInit();         // 引擎自带的初始化：6 腿回站姿（写舵机 + 每条腿 delay(100)）
    for (int i = 0; i < 6; i++) legs[i].setArcLift(arcLiftCm);   // 覆盖它内部写死的 3.0

    triOn = true;
    triRun = continuous;
    triPhaseTarget = continuous ? 0 : phases;
    triLastSteps = triGait->steps();     // 拍数基线（刚建好 = 0）

    // ⚠ 故意**不调 TriGaitStart()**：它发完第一拍（A 组）之后，TriGaitWork() 里 mode==0 分支
    //   还会再发一次 A 组 —— 那一拍打在"已经到位的腿"上（没有水平位移，只有摆动腿原地抬一下），
    //   等于白走半拍。直接从 Init 起步、让第一次 Work() 发第一拍，序列就是严格 A,B,A,B…
    //   （TripodGait.cpp 的相位逻辑一行都没动。）
    Serial.printf("→ 三角步态%s开始: φ=%.0f°(%s) | SequentialOffset=%d | 每拍 %lu ms | 抬腿 %.1fcm\n",
                  continuous ? "" : (phases >= 2 ? "（走一个完整周期）" : "（走一拍）"),
                  triPhiDeg(), dirNameOfIndex(triDirIndex), triSqo(),
                  (unsigned long)primDuration, arcLiftCm);
    Serial.printf("  腿方位角:");
    for (int i = 0; i < 6; i++) Serial.printf(" leg%d=%.0f°", i, legAzimuthDeg(i));
    Serial.println();
    Serial.printf("  第一拍: %s\n", phasePatternByLeg(SLOT_A).c_str());
    Serial.printf("  第二拍: %s   (a/b=摆动抬腿伸/收  c/d=支撑贴地伸/收，数字=腿号)\n",
                  phasePatternByLeg(SLOT_B).c_str());
}

// 停三角步态（`tri stop`）：让引擎自己回站位，再兜住它"不清 isMovingFlag"的坑
void stopTripod() {
    bool was = triOn;
    triOn = false;
    triRun = false;
    triPhaseTarget = 0;
    if (!was) { Serial.println("(三角步态本来就没在跑)"); return; }
    triStopGait();
    Serial.println("↑ 三角步态停止（6 腿回站立位）");
}

// loop 每帧调用：推进引擎 —— 它内部每帧 update 全部 6 条腿，等全部停稳才切下一拍
//（严格同步、不互相拉扯）；本函数只负责"有限模式数够拍数就收工"和打拍号。
void stepTripod() {
    if (!triOn || !triGait) return;

    // 有限模式：拍数够了且腿已停稳 → 收工。必须放在 TriGaitWork() 之前判断，
    // 否则引擎会在同一帧里又发出下一拍。
    if (!triRun && triLastSteps >= triPhaseTarget && allSixStopped()) { triFinish(); return; }

    triGait->TriGaitWork();

    int st = triGait->steps();       // 引擎每发出一拍就 +1
    if (st != triLastSteps) {
        triLastSteps = st;
        Serial.printf("  [第%d拍] %s: %s\n", st, (st % 2 == 1) ? "第一拍" : "第二拍",
                      ((st % 2 == 1) ? phasePatternByLeg(SLOT_A)
                                     : phasePatternByLeg(SLOT_B)).c_str());
    }
}

// 方向自检（纯数学，不需要硬件）：
//   把 TripodGait 的两张相位表按 sqo 旋转，算每一拍的净推力方向 —— 规则：
//     支撑相 d（直线收）→ 把机体推向该腿朝外方向 +ê
//     支撑相 c（直线伸）→ 推向 −ê
//     摆动相 a/b        → 不产生推力
//   期望：sqo = k → 两拍都恰好推向 60°×k（这就是"SequentialOffset 就是方向选择器"）。
//   若真机走出来的方向与这里不符，那不是算法问题，先查腿序/方位角（试 `mirror`）与舵机装反。
void verifyDirections() {
    Serial.println("--- 方向自检：SequentialOffset → 净推力方向（纯数学，无需硬件）---");
    Serial.println("  规则: d(支撑相收)→推向该腿朝外方向 +ê  c(支撑相伸)→推向 −ê  a/b(摆动相)→不产生推力");
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
                int leg = (slot + sqo) % 6;                  // 该槽位由哪条物理腿来跑
                float a = legAzimuthDeg(leg) * DEG_TO_RAD;
                sx += sgn * cosf(a);
                sy += sgn * sinf(a);
            }
            mag[ph] = sqrtf(sx * sx + sy * sy);
            dirDeg[ph] = (mag[ph] < 1e-6f) ? 0.0f : atan2f(sy, sx) * RAD_TO_DEG;
            float expect = 60.0f * (float)sqo * (float)AZIMUTH_SIGN;
            float err = normDeg180(dirDeg[ph] - expect);
            if (fabsf(err) > fabsf(worst)) { worst = err; worstSqo = sqo; worstPh = ph; }
        }
        Serial.printf("  sqo=%d → 期望 %3d°(%-4s) | 第一拍 %6.1f°(净%.2f×行程)  第二拍 %6.1f°(净%.2f×行程)\n",
                      sqo, 60 * sqo, dirNameOfIndex(sqo),
                      normDeg360(dirDeg[0]), mag[0], normDeg360(dirDeg[1]), mag[1]);
    }
    Serial.printf("  最大方向偏差: %+.2f°（sqo=%d 的第%d拍）\n", worst, worstSqo, worstPh + 1);
    Serial.printf("  当前制式(%s)下，6 个方向用的 sqo:\n",
                  (AZIMUTH_SIGN > 0) ? "逆时针 leg1=左前" : "顺时针镜像 leg1=右前");
    for (int k = 0; k < 6; k++) {
        int sqo = (AZIMUTH_SIGN > 0) ? k : ((6 - k) % 6);
        Serial.printf("    %-2s %3d° %s → sqo=%d\n", DIRS[k].key, 60 * k, DIRS[k].name, sqo);
    }
    Serial.println("  ⚠ 真机方向不对时先查: ①腿序/方位角是否镜像（发 mirror）②该腿舵机是否装反");
}

// 三角步态状态打印（`phase` 命令）
void printTripodStatus() {
    Serial.println("--- 三角步态状态（引擎: src/TripodGait.cpp 的 TripodGait）---");
    Serial.printf("  运行: %s | 方向 φ = %.0f° (%s) | SequentialOffset = %d | 方位角制式: %s\n",
                  triActive() ? "YES" : "no", triPhiDeg(), dirNameOfIndex(triDirIndex), triSqo(),
                  (AZIMUTH_SIGN > 0) ? "逆时针 leg1=左前" : "顺时针镜像 leg1=右前");
    Serial.printf("  每拍 %lu ms | 抬腿 %.1f cm | 全行程 %.2f cm（TripodGait 固定：站立↔伸出）\n",
                  (unsigned long)primDuration, arcLiftCm, strokeR);
    Serial.printf("  腿方位角:");
    for (int i = 0; i < 6; i++) Serial.printf(" leg%d=%.0f°", i, legAzimuthDeg(i));
    Serial.println();
    Serial.printf("  第一拍: %s\n", phasePatternByLeg(SLOT_A).c_str());
    Serial.printf("  第二拍: %s\n", phasePatternByLeg(SLOT_B).c_str());
    if (triActive() && triGait) {
        int st = triGait->steps();
        Serial.printf("  已发出 %d 拍（奇数=第一拍，偶数=第二拍）→ 下一步: %s\n",
                      st, (st % 2 == 0) ? "第一拍" : "第二拍");
    }
    Serial.printf("  理想无滑移: 每拍前进 %.2f cm（= 2×全行程），一个周期(两拍) %.1f cm\n",
                  2.0f * strokeR, 4.0f * strokeR);
    Serial.println("  ⚠ 换方向 / 改 dur = 停→回站姿→重新起步（sqo 与时长都是构造后固定）");
}

// 设置方向档 k（0..5 → 0°/60°/…/300°）：
//   跑动中 = 先停回站姿、再按新方向重新起步；空闲且 startIfIdle = 直接开跑
void setGaitDirIndex(int k, bool startIfIdle) {
    k = ((k % 6) + 6) % 6;
    bool wasRunning = triOn;
    bool wasRun     = triRun;
    int  wasTarget  = triPhaseTarget;

    triDirIndex = k;
    Serial.printf("行走方向 = %.0f° (%s) | SequentialOffset = %d\n",
                  triPhiDeg(), dirNameOfIndex(k), triSqo());

    if (wasRunning) {
        Serial.println("（换方向：先停回站姿，再按新方向重新起步）");
        triStart(wasRun, wasRun ? 0 : wasTarget);
    } else if (startIfIdle) {
        triStart(true, 0);
    } else {
        Serial.println("（方向已设定：发 tripod / f / fl / bl / b / br / fr 开始走）");
    }
}

// ========== 状态显示 ==========

void printPos() {
    if (selectedCount == 0) {
        Serial.println("当前未选中任何腿（legs ...）");
    } else if (selectedCount == 1) {
        // 单腿：完整显示（与 SingleLegTest 一致）
        int id = selectedList[0];
        LegKinematics kin(robot::LENA, robot::LENB);
        LegAngles ang = kin.ik(legs[id].footX(), legs[id].footY());
        // ik() 返回【弧度】，下面两行乘 180/π 转成度 —— 只换算这一次
        float thighDeg = ang.thigh * 180.0 / PI;
        float kneeDeg  = ang.knee  * 180.0 / PI;
        // 无偏转角（与 src/SingleLeg.cpp 同口径）：大腿 = ag2、小腿 = 180 − ag1
        float servoThigh = thighDeg;
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
    // 三角步态运行时补一段每腿槽位/相位（原有输出不变，只在后面追加）
    if (triActive()) {
        Serial.printf("  [三角步态] 方向%.0f°(%s) sqo=%d 每拍%lums 已发%d拍（本文件 legs[] 就是引擎在用的对象）\n",
                      triPhiDeg(), dirNameOfIndex(triDirIndex), triSqo(), (unsigned long)primDuration,
                      triGait ? triGait->steps() : 0);
        for (int i = 0; i < 6; i++) {
            int slot = slotOfLeg(i);
            Serial.printf("    leg%d 槽位%d 第一拍'%c' 第二拍'%c' foot=(%.2f, %.2f) %s\n",
                          i, slot, SLOT_A[slot], SLOT_B[slot],
                          legs[i].footX(), legs[i].footY(),
                          legs[i].isMoving() ? "[moving]" : "[stop]");
        }
    }
}

// ========== 舵机角度打印（servo 命令，与 SingleLegTest 同口径）==========
//
// 打印选中腿舵机的【无偏转角】—— 即"计算角"，不含 Config.h::ANGLE_OFFSET、也不限幅，
// 就是交给 ServoManager::write() 的那一刻的角度：
//     ag2 / ag1 = ik() 给出的关节角（弧度）→ ×180/π 转【度】（唯一一次换算）
//     无偏转角 = 大腿: ag2、小腿: 180 − ag1
// ⚠ 大腿口径必须与 src/SingleLeg.cpp::writeServosFromRad() 成对修改。
const float FEMUR_ANGLE_BASE = 0.0f;     // 大腿 = ag2 + 0（实物 0° = 竖直向上；站姿 67.38°）
const float TIBIA_ANGLE_BASE = 180.0f;   // 小腿 = 180 − ag1（站姿 112.62°）

const char* jointName(int joint) {
    return (joint == ServoManager::FEMUR) ? "FEMUR(大腿)" : "TIBIA(小腿)";
}

// 某腿某关节"此刻会写入的逻辑角（无偏转角）"（度）：
//   直控模式 → 用直控时记录的角度（known=false 表示该关节没记录）；
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
            float f = servoLogicalOf(id, j, known);
            if (!known) continue;
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

    // ---- 通用 ----
    if (first == "help" || first == "?") { printHelp(); return; }
    if (first == "pos")    { printPos(); return; }
    if (first == "dirs")   { printDirTable(); return; }
    if (first == "verify" || first == "check") { verifyDirections(); return; }
    if (first == "mirror" || first == "rev") {
        AZIMUTH_SIGN = -AZIMUTH_SIGN;
        Serial.printf("腿方位角制式 → %s（leg1 = %s）\n",
                      (AZIMUTH_SIGN > 0) ? "逆时针" : "顺时针镜像",
                      (AZIMUTH_SIGN > 0) ? "左前" : "右前");
        Serial.println("（同一个 sqo 现在对应镜像的物理方向；跑动中会自动按新方向重新起步）");
        setGaitDirIndex(triDirIndex, false);   // 跑动中 → 停回站姿后按新制式重新起步
        return;
    }

    // ---- 选腿 / 单腿·多腿（与 SingleLegTest 相同）----
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
            servoWatchSet(true, (val.length() > 0) ? (unsigned long)val.toInt() : 0);
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
            for (int i = 0; i < 6; i++) legs[i].setArcLift(v);
            Serial.printf("抬腿高度 = %.1f cm（选中腿与三角步态共用；下一次原语/下一拍生效）\n", v);
        } else Serial.println("! 用法: lift <cm>");
        return;
    }
    if (first == "dur") {
        unsigned long v = 0;
        if (sscanf(rest.c_str(), "%lu", &v) == 1 && v >= 50) {
            primDuration = v;
            Serial.printf("动作时长 / 三角步态每拍 = %lu ms%s\n", (unsigned long)primDuration,
                          triActive() ? "（三角步态：下次 tripod 起步时套用 —— 每拍时长是构造后固定的）" : "");
        } else Serial.println("! 用法: dur <ms> (≥50)");
        return;
    }

    // ---- 三角步态 ----
    if (first == "tripod" || first == "tri" || first == "walk") {
        String arg = rest; arg.toLowerCase(); arg.trim();
        if (arg == "stop" || arg == "off" || arg == "home") { stopTripod(); return; }
        if (arg == "one" || arg == "cycle")                { triStart(false, 2); return; }
        if (arg == "step" || arg == "half")                { triStart(false, 1); return; }
        if (arg.length() > 0) {
            Serial.println("! 用法: tripod | tri one | tri step | tri stop");
            return;
        }
        triStart(true, 0);
        return;
    }
    if (first == "phase" || first == "tripos") { printTripodStatus(); return; }
    if (first == "stride") {
        Serial.printf("! TripodGait 没有 stride 参数：行程固定 = 站立↔伸出（全行程 %.2f cm）。\n"
                      "  想要更小的步幅只能改 tripod 的方向档 / 用 legs+cycle 手动调（见 SingleLegTest 的 goto）。\n",
                      strokeR);
        return;
    }
    if (first == "dir") {
        float v = 0;
        if (sscanf(rest.c_str(), "%f", &v) == 1) {
            // TripodGait 的方向分辨率是 60°（SequentialOffset）→ 吸附到最近一档
            int k = ((int)lroundf(v / 60.0f) % 6 + 6) % 6;
            if (fabsf(v - 60.0f * k) > 1.0f) {
                Serial.printf("! 方向只有 60° 一档：%.0f° → 吸附到 %.0f°\n", v, 60.0f * k);
            }
            setGaitDirIndex(k, false);       // 只设方向，不开跑
        } else printDirTable();
        return;
    }

    // ---- 6 个方向快捷键：设方向；空闲时直接开跑，跑动中 = 停止后按新方向重新起步 ----
    for (int i = 0; i < 6; i++) {
        if (first == DIRS[i].key) { setGaitDirIndex(i, true); return; }
    }

    // ---- 舵机直控：0 <角度> → 选中腿大腿；1 <角度> → 选中腿小腿 ----
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

    // 站立足端/伸出足端 → 三角步态的全行程（站姿几何改了这里自动跟着变；只用于打印/自检）
    double sx, sy, ex, ey;
    SingleLeg::defaultStandingFoot(sx, sy);
    SingleLeg::extendedFoot(ex, ey);
    footStandX = (float)sx; footStandY = (float)sy;
    footExtX   = (float)ex; footExtY   = (float)ey;
    rStand = -footStandX;
    rExt   = -footExtX;
    strokeR = rExt - rStand;

    Serial.printf("=== 六方向步态全功能测试: 站立高度 %.1fcm | 站立足 (%.2f, %.2f) 径向 %.2fcm | "
                  "伸出足 (%.2f, %.2f) 径向 %.2fcm\n"
                  "    三角步态引擎 = src/TripodGait.cpp 的 TripodGait（全行程 %.2fcm，"
                  "理想每拍前进 %.2fcm）===\n",
                  STAND_HEIGHT, footStandX, footStandY, rStand, footExtX, footExtY, rExt,
                  strokeR, 2.0f * strokeR);

    // 应用默认腿选择（内部会对选中腿逐条 init 回默认站立位 —— 会写舵机，先架起机器人）
    applySelection(DEFAULT_SELECTION);

    printHelp();
    printDirTable();
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

    // 3) 舵机角度连续打印（servo watch [ms]；未开启时立即返回，不影响时序）
    stepServoWatch();

    // 4) 三角步态：推进 TripodGait（引擎内部每帧 update 6 条腿、全部停稳才切下一拍）
    if (triActive()) { stepTripod(); delay(10); return; }

    // 5) 逐腿巡检状态机
    if (eachMode) { stepEach(); delay(5); return; }

    // 6) 选中腿全部停稳后，按模式安排下一步（以最慢的腿为准，保证严格同步）
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
