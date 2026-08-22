/* 单腿逆运动学：足端坐标 → 关节角（2 自由度，公式适配自 角度计算/try.cpp）
 * 坐标原点 = 大腿根部舵机（髋关节）；返回角度均为弧度。
 */
#ifndef LEGKINEMATICS_H
#define LEGKINEMATICS_H

#include <cmath>

struct FootPos {
    double x;
    double y;
};

struct LegAngles {
    double knee;  // 膝盖角（两腿角 ag1）
    double thigh; // 大腿角（机腿角 ag2）
};

class LegKinematics {
public:
    // thigh_len：大腿长，calf_len：小腿长
    LegKinematics(double thigh_len, double calf_len);

    // 足端坐标 → 两关节角（弧度）；原点附近（dist < 1e-9）无法构成角度，返回 0
    LegAngles ik(double fx, double fy) const;

    // 足端到原点的距离
    double dist(double fx, double fy) const;

    // 可达性检查（按距离 / 按坐标）：内圆 |lena-lenb| ≤ d ≤ 外圆 lena+lenb
    bool reachable(double d) const;
    bool reachable(double fx, double fy) const;

private:
    double lena_;        // 大腿长
    double lenb_;        // 小腿长
    double outer_range_; // 外圆半径 lena + lenb
    double inner_range_; // 内圆半径 |lena - lenb|
};

#endif
