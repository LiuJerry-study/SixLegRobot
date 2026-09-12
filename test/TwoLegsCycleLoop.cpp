/* 两腿同步周期循环测试（Arduino sketch）
 *
 * 目的：让两条腿「同时」各自周期运动，并且在 loop 里持续循环运行。
 *       与 TwoLegsTest（顺序串联、只跑一次）不同，这里是——
 *         1) 双腿同步推进：每一帧都同时 update legA 和 legB；
 *         2) 循环不停：任一腿走完一个周期后立即开始下一个周期，永不停止。
 *
 * 一个完整周期（对每条腿）：
 *   1) 摆动相抬腿前迈  = 弧线张  PRIM_ARC_EXTEND （抬腿伸出）
 *   2) 支撑相贴地收回  = 直线收  PRIM_LINE_RETRACT （贴地后推）
 *
 * 两种模式：
 *   - PHASE_SYNC  ：两腿完全同步（同时张、同时收）。适合检验左右腿舵机对称性。
 *   - PHASE_OPPOSITE：两腿反相 180°（一条张时另一条收），呈「踏步 / 原地走路」效果。
 *   修改下方 PHASE_MODE 宏即可切换。
 *
 * 用的两条腿是 legA = leg0（PCA9685 ch0/ch1）、legB = leg1（ch2/ch3）；两腿各自独立
 * 计时、互不等待，所以 PHASE_SYNC 下几乎完全同步，PHASE_OPPOSITE 下始终保持
 * "一伸一收"的 180° 相位差（踏步效果）。
 *
 * 本文件是保留的独立实现（自含 setup/loop），不在 src/ 下，`pio run` 不编译它，
 * 也不是 PlatformIO 单元测试；等价的平台模式见 src/main.cpp 的 -DMODE_TWO_LEGS_CYCLE。
 * 要在 Arduino IDE 里单独编译，需与以下源文件一起编译：
 *   include/Config.h、include/LegKinematics.h、include/ServoManager.h、
 *   include/SingleLeg.h、src/LegKinematics.cpp、src/ServoManager.cpp、src/SingleLeg.cpp
 *
 * 角度口径（2026-09-10 定为"外倾为正"，与 src/SingleLeg.cpp 配套；映射按 2026-09 实物实测）：
 *   ik() 返回【弧度】，默认站姿 ag1 = ag2 = +67.38°；舵机映射 大腿 = ag2（舵机 0° = 大腿竖直向上、
 *   180° = 竖直向下）、小腿 = 180 − ag1（只换算一次 180/π）；Config.h::ANGLE_OFFSET 整表已清零
 *   ⇒ 站姿写入就是 67.38° / 112.62°（仍在 0~180° 内）。
 */

#include <Arduino.h>
#include "SingleLeg.h"

// 使用的两条腿
SingleLeg legA(0);   // 腿 0
SingleLeg legB(1);   // 腿 1

// 每条腿单个原语（张/收）的时长（毫秒）
const unsigned long PRIM_DURATION = 500;

// 每条腿上依次执行的原语序列（构成一个完整周期：抬腿前迈 → 贴地收回）
const int CYCLE_STEPS = 2;                              // 每一步对应一个原语
const LegPrimitive CYCLE[CYCLE_STEPS] = {
    PRIM_ARC_EXTEND,    // 摆动相：抬腿弧线伸出
    PRIM_LINE_RETRACT   // 支撑相：贴地直线收回
};

// 相位模式：双腿同步 / 双腿反相（踏步）
#define PHASE_SYNC       0
#define PHASE_OPPOSITE   1
const int PHASE_MODE = PHASE_OPPOSITE;   // 改成 PHASE_SYNC 即双腿同步

// 反相时 legB 从周期的第 1 步（收）开始，使两腿相位相差 180°
const int legA_seed = 0;                                  // legA 依次跑 CYCLE[0..CYCLE_STEPS-1]
const int legB_seed = (PHASE_MODE == PHASE_OPPOSITE)      // legB 从反相步开始
                    ? (CYCLE_STEPS / 2)                   // 2 步周期 → 从第 1 步即"收"开始
                    : 0;

// 当前腿正处于周期中的第几步
int  legA_step;
int  legB_step;

void setup() {
    Serial.begin(115200);
    delay(200);

    // 初始化两条腿：传 10.0 只是把站立高度记进成员变量（当前实现未参与运算），
    // 真正的默认站姿由几何决定 = (−6, −10)（小腿竖直、与内圆相切）；
    // init() 内部会幂等调 ServoManager::begin()，并立刻把该腿两路舵机写到默认站姿
    legA.init(10.0);
    legB.init(10.0);

    // 抬腿高度（弧线原语的拱高，cm），可根据实际调整；直线原语不用这个值
    legA.setArcLift(3.0);
    legB.setArcLift(3.0);

    // 同时启动两条腿（legA 从第 0 步；legB 按相位模式取起始步）
    legA_step = legA_seed;
    legB_step = legB_seed;
    legA.startPrimitive(CYCLE[legA_step], PRIM_DURATION);
    legB.startPrimitive(CYCLE[legB_step], PRIM_DURATION);

    Serial.print("TwoLegsCycleLoop started, phase mode = ");
    Serial.println(PHASE_MODE == PHASE_SYNC ? "SYNC" : "OPPOSITE");
}

void loop() {
    // 1) 同时推进两条腿（每帧都 update，保证双腿一起动）
    legA.update();
    legB.update();

    // 2) legA：当前原语完成 → 进入周期下一步；越界即回到第 0 步 → 持续循环
    if (!legA.isMoving()) {
        legA_step++;
        if (legA_step >= CYCLE_STEPS) legA_step = 0;      // 循环回起点
        legA.startPrimitive(CYCLE[legA_step], PRIM_DURATION);
    }

    // 3) legB：同上，独立循环
    if (!legB.isMoving()) {
        legB_step++;
        if (legB_step >= CYCLE_STEPS) legB_step = 0;      // 循环回起点
        legB.startPrimitive(CYCLE[legB_step], PRIM_DURATION);
    }
}
