/* 两腿顺序运动测试（Arduino sketch）
 *
 * 目的：验证两条腿的「周期」串联 —— 一条腿完成一个完整步态周期后，
 *       第二条腿才开始自己的周期。用于检查每条腿的摆动相/支撑相运动学
 *       是否正常，以及同一时刻只有一条腿在运动。
 *
 * 一个完整周期（对每条腿）：
 *   1) 摆动相抬腿前迈  = 弧线张  PRIM_ARC_EXTEND （抬腿伸出）
 *   2) 支撑相贴地收回  = 直线收  PRIM_LINE_RETRACT （贴地后推）
 *
 * 两条腿是 legA = leg0（PCA9685 ch0/ch1）、legB = leg1（ch2/ch3）；本文件是"串行"演示：
 * legA 跑完整个周期（a→d）之后才启动 legB，全程同一时刻只有一条腿在动。
 *
 * 本文件是保留的独立实现（自含 setup/loop），不在 src/ 下，`pio run` 不编译它，
 * 也不是 PlatformIO 单元测试；等价的平台模式见 src/main.cpp 的 -DMODE_TWO_LEGS。
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

// legA 是否已经完成一整周期（完成后才启动 legB）
bool legA_done     = false;
bool legB_running  = false;

// 当前腿正处于周期中的第几步
int  legA_step = 0;
int  legB_step = 0;

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

    // 启动第一条腿的第一个动作，开始第一步
    legA.startPrimitive(CYCLE[0], PRIM_DURATION);
    legA_step = 0;

    Serial.println("TwoLegsTest started");
}

void loop() {
    // 1) 推进正在运动的那条腿（每帧调用 update 完成插值并写舵机）
    if (!legA_done) {
        legA.update();

        // 当前原语完成 → 进入下一个原语，直到整周期结束
        if (!legA.isMoving()) {
            legA_step++;
            if (legA_step < CYCLE_STEPS) {
                // 继续跑下一个原语
                legA.startPrimitive(CYCLE[legA_step], PRIM_DURATION);
            } else {
                // 第一周期全部完成
                legA_done = true;
                Serial.println("legA finished one cycle");
            }
        }
    }

    // 2) legA 完成整周期后，才开始 legB 的周期
    if (legA_done && !legB_running) {
        legB.startPrimitive(CYCLE[0], PRIM_DURATION);
        legB_step = 0;
        legB_running = true;
        Serial.println("legB started");
    }

    // 3) 推进 legB 的周期
    if (legB_running) {
        legB.update();
        if (!legB.isMoving()) {
            legB_step++;
            if (legB_step < CYCLE_STEPS) {
                legB.startPrimitive(CYCLE[legB_step], PRIM_DURATION);
            } else {
                Serial.println("legB finished one cycle");
                // 两条腿都完成：停止，或重置再来一轮
                // legA_done = false; legB_running = false;  // 如需循环则取消注释
                while (1) { delay(100); }   // 演示完成后保持
            }
        }
    }
}
