/* 单腿控制类：足端位置管理 + 轨迹插值（摆动相 / 支撑相）。
 *
 * 方案 A：不直接操作舵机——引脚、零位偏移、限幅全部由 ServoManager 负责，
 * 本类只做「足端坐标 → 关节角 → 写入」的运动学逻辑。
 */

#ifndef SINGLE_LEG_H
#define SINGLE_LEG_H

#include <Arduino.h>
#include "LegKinematics.h"

// 轨迹模式
enum TrajectoryMode {
    MODE_SWING,    // 摆动相：走弧线（抬腿）
    MODE_STANCE    // 支撑相：贴地锁高（防打滑）
};

class SingleLeg {
public:
    // 只需腿 ID（0~5）；3 个舵机引脚由 Config.h 的 SERVO_PINS[legId] 决定
    SingleLeg(int id);

    // setup 中调用；内部自动调 ServoManager::begin()（幂等）
    void init(float standHeight);

    // 立即移动到指定足端坐标（瞬间到达，用于初始复位 / 调试）
    void setFootPositionImmediate(float x, float y);

    // 默认站立足端：小腿竖直并与内圆（半径 |LENA-LENB|）相切，取负侧切线
    static void defaultStandingFoot(double& x, double& y);

    // 伸出足端：默认站立高度上，站立脚与达外圆同侧交点所成线段的 2/3 分点
    static void extendedFoot(double& x, double& y);

    // 启动一段轨迹：起点 → 终点，按模式走弧线或贴地，持续 durationMs
    void startGrab(float startX, float startY,
                   float targetX, float targetY,
                   float liftHeight,
                   TrajectoryMode mode,
                   unsigned long durationMs);

    void update();   // 每帧调用（放 loop），推进轨迹并写舵机
    bool isMoving() const { return isMovingFlag; }   // 是否仍在运动中

private:
    int legId;
    LegKinematics kin;   // 腿长取自 Config.h（LENA 大腿 / LENB 小腿）

    float currentX, currentY;   // 当前足端坐标
    float standHeight;          // 站立高度

    // 插值运动状态
    float startX, startY;
    float targetX, targetY;
    float liftHeight;
    TrajectoryMode currentMode;
    unsigned long startTime;
    unsigned long duration;
    bool isMovingFlag;

    // 弧度 → 舵机坐标系 → ServoManager 写入（偏移与限幅在其内部）
    void writeServosFromRad(float baseAngleDeg, float thighRad, float kneeRad);
};

#endif
