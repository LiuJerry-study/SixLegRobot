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

    LegKinematics kin;  // 你的 IK 类（默认构造用 4.0, 3.0）

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