/* 单腿控制类：足端位置管理 + 轨迹插值（摆动相 / 支撑相）。
 *
 * 硬件：每条腿 2 个舵机（大腿 + 小腿），两自由度在同一竖直平面内，无髋节旋转。
 * 方案 A：不直接操作舵机——通道、零位偏移、限幅全部由 ServoManager 负责，
 * 本类只做「足端坐标 → 关节角 → 写入」的运动学逻辑。
 *
 * 坐标系：原点 = 髋关节（大腿根部舵机），x+ 指向机体中心（内侧）、x− 为外侧，
 *         y+ 竖直向上 → 着地方向是 y 为负；默认站立足端 (−6, −10)。
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

// 单腿原语：形状(弧线/直线) × 方向(张/收)
// 张 = 向 extendedFoot() 伸出，收 = 回到 defaultStandingFoot()
enum LegPrimitive {
    PRIM_ARC_EXTEND,   // a弧线张：带抬腿弧的伸出（摆动相）   
    PRIM_ARC_RETRACT,  // b弧线收：带抬腿弧的收回（摆动相）   
    PRIM_LINE_EXTEND,  // c直线张：贴地直线伸出（支撑相推地） 
    PRIM_LINE_RETRACT  // d直线收：贴地直线收回（支撑相推地） 
};

class SingleLeg {
public:
    // 只需腿 ID（0~5）；2 个舵机引脚由 Config.h 的 SERVO_PINS[legId] 决定
    SingleLeg(int id);

    // setup 中调用；内部自动调 ServoManager::begin()（幂等）
    void init(float standHeight);

    // 立即移动到指定足端坐标（瞬间到达，用于初始复位 / 调试）
    void setFootPositionImmediate(float x, float y);

    // 默认站立足端：小腿竖直并与内圆（半径 |LENA-LENB|）相切，取负侧切线
    static void defaultStandingFoot(double& x, double& y);

    // 伸出足端：默认站立高度上，站立脚与外圆（半径 LENA+LENB）同侧交点所成线段的 2/3 分点
    static void extendedFoot(double& x, double& y);

    // 启动一段轨迹：起点 → 终点，按模式走弧线或贴地，持续 durationMs
    void startGrab(float startX, float startY,
                   float targetX, float targetY,
                   float liftHeight,
                   TrajectoryMode mode,
                   unsigned long durationMs);

    // 执行一条单腿原语：自动填起终点与抬腿高度。
    // 起点 = 当前足端（避免跳变），终点 = 张→伸出脚 / 收→站立脚；
    // 弧线 → 抬腿拱弧（抬腿高度 = arcLift），直线 → 贴地直线。
    void startPrimitive(LegPrimitive prim, unsigned long durationMs);

    // 设置弧线原语的默认抬腿高度（cm），默认 3.0
    void setArcLift(float cm) { arcLift = cm; }

    void update();   // 每帧调用（放 loop），推进轨迹并写舵机
    bool isMoving() const { return isMovingFlag; }   // 是否仍在运动中

    // 当前足端坐标（cm，原点=髋关节），调试/外部读取用
    float footX() const { return currentX; }
    float footY() const { return currentY; }

private:
    int legId;
    LegKinematics kin;   // 腿长取自 Config.h（LENA 大腿 / LENB 小腿）

    float currentX, currentY;   // 当前足端坐标
    float standHeight;          // 站立高度（cm，由 init() 记录；当前实现里只存不用，
                                // 初始姿势实际由 defaultStandingFoot() 的恒定足端 (−6,−10) 决定）

    // 插值运动状态
    float startX, startY;
    float targetX, targetY;
    float liftHeight;
    float arcLift;          // 弧线原语默认抬腿高度（cm）
    TrajectoryMode currentMode;
    unsigned long startTime;
    unsigned long duration;
    bool isMovingFlag;

    // 足端坐标 → 关节角(弧度) → 舵机角(度) → ServoManager 写入（偏移与限幅在其内部）。
    // 入参单位是弧度（LegKinematics::ik() 的返回值），本函数乘 180/π 转度（只换算这一次）。
    // 映射：大腿 = ag2（ag2 为"外倾为正"口径；实物实测 0°=竖直向上、180°=竖直向下）、小腿 = 180 − ag1
    //   → 默认站姿 67.38° / 112.62°，详见 src/SingleLeg.cpp 实现处的口径说明。
    void writeServosFromRad(float thighRad, float kneeRad);
};

#endif
