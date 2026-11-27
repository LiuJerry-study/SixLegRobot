/* 单腿逆运动学：足端坐标 → 关节角（2 自由度；与 角度计算/try.cpp 的 ccl() 同式，只是整式取了反号）
 * 坐标原点 = 大腿根部舵机（髋关节）；y+ 竖直向上，x+ 指向机体中心（内侧），x− 为外侧（髋外侧）。
 *
 * 单位以本文件代码为准：ik() 返回的是【弧度】，不是度（转度由调用方自己乘 180/π）：
 *     d     = √(fx² + fy²)
 *     knee （膝内角 ag1）= acos((LENA² + LENB² − d²) / (2·LENA·LENB))                     // 恒为正
 *     thigh（大腿角 ag2）= −( acos((LENA² + d² − LENB²) / (2·LENA·d)) + atan2(fx, fy) )
 *   d < 1e-9 时两个角都返回 0；两个余弦值都先 clamp 到 [−1, 1]（防浮点误差让 acos 出 NaN）。
 *
 * 【ag2 的符号口径：外倾为正】（与舵机映射 `大腿 = ag2` 配套，2026-09-10 定口径、09 月实物实测确认映射）
 *   ag2 = 大腿方向相对 +y（竖直向上）的角度，向 −x（髋外侧/外拱）偏为正、向 +x（体内侧）偏为负。
 *   默认站姿脚 (−6, −10)：d ≈ 11.66 → knee ≈ +1.1760 rad (67.38°)、thigh ≈ +1.1760 rad (+67.38°)
 *   —— 上拱外倾故为正（"Λ" 拱）。同一姿态若按"体内侧为正"记则是 −67.38°（旧口径）。
 *   推导：acos((LENA²+d²−LENB²)/(2·LENA·d)) ≈ +81.66°（只含大小），atan2(fx, fy) ≈ −149.04°
 *   （脚在髋下方 fy<0，方向几乎指向 −y），两项之和 ≈ −67.38°，整式取反即 +67.38°。
 *   ⚠ 不要只改 atan2 的符号（`acos + |atan2|` 或 `acos − atan2`）：acos 项只依赖距离 d，镜像下不变，
 *     翻 atan2 的符号等于把解换成【镜像脚位】(−fx, fy) —— 站姿会解到 (+6, −10)、膝盖甩到髋下方。
 *     符号就是"脚在髋外侧还是内侧"这一个信息位，只能整式取反。
 *
 * 配套映射见 src/SingleLeg.cpp::writeServosFromRad()：大腿 = ag2（度）、小腿 = 180 − ag1
 *   → 站姿 67.38° / 112.62°；RobotModel.h 保存逐路零位偏移，ServoManager 负责叠加和限幅。
 * 有效行程：大腿舵机 0~180° ↔ ag2 ∈ [0°, 180°]（外拱可用区 ag2 ∈ [0°, 90°]），
 *   ag2 < 0 或 > 180° 会被 ServoManager 限幅到端点。
 */
#ifndef LEGKINEMATICS_H
#define LEGKINEMATICS_H

#include <cmath>

struct FootPos {
    double x;
    double y;
};

struct LegAngles {
    double knee;  // 膝盖角（两腿角 ag1），单位：弧度，180° = 完全伸直、0° = 完全折叠（弯曲角 θ = 180° − ag1）
    double thigh; // 大腿角（机腿角 ag2），单位：弧度，相对 +y（竖直向上），偏向 −x（髋外侧）为正
};

class LegKinematics {
public:
    // thigh_len：大腿长，calf_len：小腿长
    LegKinematics(double thigh_len, double calf_len);

    // 足端坐标(cm) → 两关节角【弧度】（ag1 膝内角 / ag2 大腿角）；原点附近（dist < 1e-9）无法构角，返回 0
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
