/*# 单腿控制类（管理当前足端位置、目标位置、抓取程度）*/

#ifndef SINGLE_LEG_H
#define SINGLE_LEG_H

#include <Arduino.h>
#include <ESP32Servo.h>   // 如果你用 Arduino Uno，改成 #include <Servo.h>
#include "LegKinematics.h"

// 轨迹模式枚举
enum TrajectoryMode {
    MODE_SWING,    // 摆动相：走弧线
    MODE_STANCE    // 支撑相：走平地（贴地）
};

class SingleLeg {
public:
    // 构造函数：传入腿的ID和3个舵机引脚
    SingleLeg(int id, int pinBase, int pinFemur, int pinTibia);

    // 初始化（在 setup 中调用）
    void init(float standHeight);

    // 立即移动到指定足端坐标（瞬间到达，用于初始复位）
    void setFootPositionImmediate(float x, float y);

    // 根据几何约束计算【默认站立】足端位置：
    //   小腿竖直，且与最小圆（内圆，半径 |LENA-LENB|）相切。
    //   为使 IK 算出的腿真正竖直，取负侧切线 kx = -(LENA-LENB)。
    static void defaultStandingFoot(double& x, double& y);

    // 根据几何约束计算【伸出】足端位置：
    //   在默认站立位置的水平线上，取它与大圆（半径 LENA+LENB）同侧交点
    //   所成线段的 2/3 处。
    static void extendedFoot(double& x, double& y);

    // 启动抓取动作（核心函数）
    void startGrab(float startX, float startY, 
                   float targetX, float targetY, 
                   float liftHeight, 
                   TrajectoryMode mode, 
                   unsigned long durationMs);

    // 必须放在主循环 loop() 中循环调用
    void update();

    // 检查是否运动完成
    bool isMoving() const { return isMovingFlag; }

private:
    int legId;
    int pinBase, pinFemur, pinTibia;
    
    Servo servoBase;
    Servo servoFemur;
    Servo servoTibia;

    LegKinematics kin;  // 腿长取自 Config.h（LENA 大腿 / LENB 小腿）

    // 状态变量
    float currentX, currentY;
    float standHeight;

    // 插值运动变量
    float startX, startY;
    float targetX, targetY;
    float liftHeight;
    TrajectoryMode currentMode;
    unsigned long startTime;
    unsigned long duration;
    bool isMovingFlag;

    // 私有方法：将 IK 计算的角度写入舵机（处理弧度→度→舵机脉宽）
    void writeServosFromRad(float baseAngleDeg, float thighRad, float kneeRad);
};

#endif