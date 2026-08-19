/* 逆运动学实现
 * 核心公式与角度计算/try.cpp 中的 ccl() 一致：
 *   knee  = acos((lena^2 + lenb^2 - dist^2) / (2*lena*lenb))
 *   thigh = acos((lena^2 + dist^2 - lenb^2) / (2*lena*dist)) + atan2(x, y)
 */
#include "LegKinematics.h"
#include <algorithm>
#include <cmath>

LegKinematics::LegKinematics(double thigh_len, double calf_len)
    : lena_(thigh_len)
    , lenb_(calf_len)
    , outer_range_(thigh_len + calf_len)
    , inner_range_(std::fabs(thigh_len - calf_len)) {}

double LegKinematics::dist(double fx, double fy) const {
    return std::sqrt(fx * fx + fy * fy);
}

bool LegKinematics::reachable(double d) const {
    return d >= inner_range_ && d <= outer_range_;
}

bool LegKinematics::reachable(double fx, double fy) const {
    return reachable(dist(fx, fy));
}

LegAngles LegKinematics::ik(double fx, double fy) const {
    const double d = dist(fx, fy);

    LegAngles a;
    a.knee = 0.0;
    a.thigh = 0.0;

    // 原点附近无法构成角度
    if (d < 1e-9) {
        return a;
    }

    // 将余弦值夹逼到 [-1, 1]，避免浮点越界导致 acos 返回 NaN
    auto clamp = [](double v) { return std::max(-1.0, std::min(1.0, v)); };

    // ag1 两腿角（膝盖角）
    const double cos1 = (lena_ * lena_ + lenb_ * lenb_ - d * d) / (2 * lena_ * lenb_);
    a.knee = std::acos(clamp(cos1));

    // ag2 机腿角（大腿角）
    const double cos2 = (lena_ * lena_ + d * d - lenb_ * lenb_) / (2 * lena_ * d);
    a.thigh = std::acos(clamp(cos2)) + std::atan2(fx, fy);

    return a;
}
