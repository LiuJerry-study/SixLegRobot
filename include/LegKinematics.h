
/* 单腿逆运动学（足端坐标 → 舵机角度）
 * 适配自 角度计算/try.cpp 的 2 自由度腿部解算
 *
 * 坐标原点 = 大腿根部舵机（髋关节）
 * 角度制：返回值均为弧度
 *  - knee  (ag1 / 两腿角) ：膝盖角
 *  - thigh (ag2 / 机腿角) ：大腿角
 */
#ifndef LEGKINEMATICS_H
#define LEGKINEMATICS_H

#include <cmath>

struct FootPos {
    double x;
    double y;
};

struct LegAngles {
    double knee;  // ag1 两腿角（膝盖角）
    double thigh; // ag2 机腿角（大腿角）
};

class LegKinematics {
public:
    // thigh_len：大腿长，calf_len：小腿长
    LegKinematics(double thigh_len, double calf_len);

    // 由足端坐标计算两关节角（弧度制）
    // 原点附近 (dist<1e-9) 无法构成角度，返回 0
    LegAngles ik(double fx, double fy) const;

    // 到达给定点的距离
    double dist(double fx, double fy) const;

    // 可达性检查（按距离 / 按坐标）
    bool reachable(double d) const;
    bool reachable(double fx, double fy) const;

private:
    double lena_;           // 大腿长
    double lenb_;           // 小腿长
    double outer_range_;    // lena + lenb（外圆半径）
    double inner_range_;    // |lena - lenb|（内圆半径）
};

#endif