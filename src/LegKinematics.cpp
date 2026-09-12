/* 逆运动学实现：足端坐标 → 关节角，2 自由度（与 角度计算/try.cpp 的 ccl() 同式，整式取了反号）
 * 本文件实际在算的公式（d = √(fx² + fy²)，结果均为【弧度】，这里不做角度转换）：
 *   knee  = acos((lena² + lenb² − d²) / (2·lena·lenb))                     // 两腿角 ag1，恒为正
 *   thigh = −( acos((lena² + d² − lenb²) / (2·lena·d)) + atan2(fx, fy) )   // 机腿角 ag2，外倾为正
 * 默认站姿脚 (−6, −10)：d ≈ 11.66 → knee ≈ +1.1760 rad (+67.38°)、thigh ≈ +1.1760 rad (+67.38°)。
 * 取反只是换符号口径、姿态不变：站姿的 Λ 拱既可以写 ag2 = +67.38°（外倾为正，当前口径），
 * 也可以写 −67.38°（体内侧为正，旧口径）—— 但舵机映射必须配套（现在 = ag2，见 SingleLeg.cpp）。
 * ⚠ 只翻 atan2 的符号（`acos + |atan2|`、`acos − atan2`）是错的：acos 项只依赖距离 d、镜像下不变，
 *   翻 atan2 等于把解换成镜像脚位 (−fx, fy)（站姿会解到 (+6, −10)、膝盖甩到髋下方）。
 * 转成「度」是调用方的事（src/SingleLeg.cpp 与 test/SingleLegTest.cpp 各乘一次 180/π）。
 *
 * 编译注记：本文件与 src/ServoManager.cpp 曾有文件作用域的 `const double PI`：后者因为 include 了
 *   <Arduino.h>（其中 `#define PI 3.14159…`）而被宏展开成非法声明，pio run 报
 *   "expected unqualified-id before numeric constant"。**两处该行均已删除，2026-09-10 实测
 *   pio run 全量编译通过**；今后别再往任何 include 了 <Arduino.h> 的文件作用域写 `const double PI`。
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

    // 膝盖角（两腿角 ag1）：两连杆夹角，180° = 完全伸直、0° = 完全折叠（单位：弧度）
    // 注意：几何弯曲角 θ = 180° − ag1（默认站姿 ag1 = 67.38° → θ = 112.62°）
    const double cos1 = (lena_ * lena_ + lenb_ * lenb_ - d * d) / (2 * lena_ * lenb_);
    a.knee = std::acos(clamp(cos1));  // 弧度，不在此处转度

    // 大腿角（机腿角 ag2）：两项之和再取反 → 外倾为正（弧度，见文件头说明）
    const double cos2 = (lena_ * lena_ + d * d - lenb_ * lenb_) / (2 * lena_ * d);
    a.thigh = -(std::acos(clamp(cos2)) + std::atan2(fx, fy));  // 弧度；外倾为正 → 站姿 +67.38°

    return a;
}
