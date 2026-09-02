/* 逆运动学实现
 * 核心公式（与 角度计算/try.cpp 的 ccl() 一致）：
 *   knee  = acos((lena² + lenb² - dist²) / (2·lena·lenb))
 *   thigh = acos((lena² + dist² - lenb²) / (2·lena·dist)) + atan2(x, y)
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

    if (d < 1e-9)   // 原点附近无法构成角度
        return a;

    // 余弦夹逼到 [-1, 1]，避免浮点误差使 acos 返回 NaN
    auto clamp = [](double v) { return std::max(-1.0, std::min(1.0, v)); };

    // 膝盖角（两腿角 ag1）
    const double cos1 = (lena_ * lena_ + lenb_ * lenb_ - d * d) / (2 * lena_ * lenb_);
    a.knee = std::acos(clamp(cos1));

    // 大腿角（机腿角 ag2）
    const double cos2 = (lena_ * lena_ + d * d - lenb_ * lenb_) / (2 * lena_ * d);
    a.thigh = std::acos(clamp(cos2)) + std::atan2(fx, fy);

    return a;
}
