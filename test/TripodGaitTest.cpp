/* 三角步态 test（六足）
 * ------------------------------------------------------------
 * 目的：用 TripodGait 类跑完整的三角步态（六足分两组、交替支撑）。
 *
 *   TripodGait tripodGait(0, 500);   // (顺序偏移, 每段原语时长 ms)
 *   TriGaitInit()  → 6 条腿各 init(10.0) 并设抬腿高度 3.0cm
 *                    （会立刻写舵机，把每条腿摆到默认站姿 (−6, −10)，务必先架空机器人）
 *   TriGaitStart() → 按三角步态相位发第一组原语：
 *                    leg0 = d 直线收、leg1/leg5 = a 弧线张、
 *                    leg3 = b 弧线收、leg2/leg4 = c 直线张
 *   TriGaitWork()  → 每帧推进 6 条腿（插值 + IK + 写舵机）；6 条腿全部停稳后按 mode 发下一拍，
 *                    loop 里反复调用它 → 无限循环的三角步态。
 *                    ⚠ 当前代码里 mode==0 分支发的仍是 A 组那 6 条原语，所以真实时序是
 *                      A(TriGaitStart 第一拍)、A、B、A、B…（A 组被连跑两拍，疑似分支写反；
 *                      本轮只改注释、未改代码）
 *
 * ⚠ 注意：文件头注释之后有一行裸文本 TripodGaitTest.cpp（既不是注释、也不是合法代码），
 *   真拿去编译会直接语法错误。它一直没暴露，是因为：
 *     1) `pio run` 只编译 src/，test/ 目录不在 PlatformIO 的编译范围内；
 *     2) 三角步态已并入 src/main.cpp 的 -DMODE_TRIPOD（那段是等价实现：
 *        同样 new TripodGait(0, 500) 并依次调 TriGaitInit/TriGaitStart/TriGaitWork）。
 *   本轮同样只改注释，不动那行文本。
 *
 * 角度口径（2026-09-10 定为"外倾为正"，与 src/SingleLeg.cpp 配套；映射按 2026-09 实物实测）：
 *   ik() 返回【弧度】，默认站姿 ag1 = ag2 = +67.38°；舵机映射 大腿 = ag2（舵机 0° = 大腿竖直向上、
 *   180° = 竖直向下）、小腿 = 180 − ag1（只换算一次 180/π）；Config.h::ANGLE_OFFSET 整表已清零
 *   ⇒ 站姿写入就是 67.38° / 112.62°（仍在 0~180° 内）。
 */
TripodGaitTest.cpp
//三角步态test（完整说明见上方文件头）

#include <Arduino.h>
#include "TripodGait.h"

void setup() {
    Serial.begin(115200);
    Serial.println("Tripod Gait Test");
    TripodGait tripodGait(0, 500); // 顺序偏移 0（腿号按原序 0~5），每段原语时长 500ms
    tripodGait.TriGaitInit(); // 初始化步态：6 条腿 init(10.0) + setArcLift(3.0)，会立刻写舵机到默认站姿
    tripodGait.TriGaitStart(); // 启动步态：按相位表给 6 条腿各发一个原语（0d 1a 5a 3b 2c 4c）
}

void loop() {
    // 1) 推进步态：每帧推进 6 条腿（smoothstep 插值 + IK + 写舵机），全部停稳后自动切相位
    tripodGait.TriGaitWork();
}

