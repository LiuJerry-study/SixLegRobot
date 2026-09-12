/* 单腿状态更新与足端轨迹规划（方案 A：写入统一走 ServoManager）
 *
 * 坐标系：原点 = 髋关节（大腿根部舵机），x+ 指向机体中心（内侧）、x− 为外侧，
 *         y+ 竖直向上（着地即 y 为负）；默认站立足端 (−6, −10)，
 *         伸出足端 ≈ (−12.77, −10)。
 * 两类轨迹：MODE_SWING（摆动相，抬腿走弧线，Y 轴叠 lift·sin(πt)）
 *          MODE_STANCE（支撑相，Y 锁死在目标高度，贴地防打滑）。
 * 角度口径：LegKinematics::ik() 返回【弧度】，writeServosFromRad() 里乘 180/π 转成【度】后
 *          交给 ServoManager（只换算这一次）；ag2 采用"外倾为正"口径，大腿舵机角 = ag2
 *          （2026-09 实物实测：大腿舵机 0° = 竖直向上、180° = 竖直向下，旧的 90 + ag2 已作废），
 *          详见下方 writeServosFromRad() 上方的注释。
 */

#include "SingleLeg.h"
#include "ServoManager.h"
#include "Config.h"
#include <math.h>

SingleLeg::SingleLeg(int id)
    : legId(id)
    , kin(robot::LENA, robot::LENB)   // 腿长取自 Config.h
    , arcLift(5.0f)                   // 弧线原语默认抬腿 5cm，可用 setArcLift 改
    , currentMode(MODE_STANCE)
    , isMovingFlag(false) {}

// 默认站立足端：小腿竖直并与内圆相切。
// 内圆半径 r = |LENA-LENB| = 6；取负侧切线 kx = -r，使 IK 解出的小腿真正竖直：
// 膝 = (kx, √(LENA²−r²)) = (−6, 2.5)，脚 = 膝正下方一个小腿长 → (−6, −10)。
void SingleLeg::defaultStandingFoot(double& x, double& y) {
    const double r  = fabs(robot::LENA - robot::LENB);
    const double kx = -r;
    const double ky = sqrt(robot::LENA * robot::LENA - r * r);
    x = kx;
    y = ky - robot::LENB;
}

// 伸出足端：默认站立高度上，站立脚与达外圆同侧交点所成线段的 2/3 分点
// 外圆 R = LENA+LENB = 19 → 交点 x = −√(19²−10²) ≈ −16.16，
// 2/3 分点 ≈ (−12.77, −10)（比 1/2 分点更靠外，用于给出更大的迈步行程）。
void SingleLeg::extendedFoot(double& x, double& y) {
    double sx, sy;
    defaultStandingFoot(sx, sy);
    const double R  = robot::LENA + robot::LENB;   // 大圆半径
    const double xi = -sqrt(R * R - sy * sy);      // 同高度大圆交点（负侧）
    x = sx + (xi - sx) * (2.0 / 3.0);              // 2/3 分点
    y = sy;
}

// 初始化：ServoManager::begin()（幂等，只初始化 PCA9685）→ 记录站立高度
//        → 用默认站立足端做一次 IK 并写舵机，把腿摆到站位
void SingleLeg::init(float standH) {
    ServoManager::begin();                          // 幂等

    standHeight = standH;                           // 仅记录，当前实现未参与运算

    double sx, sy;
    defaultStandingFoot(sx, sy);
    currentX = (float)sx;
    currentY = (float)sy;

    LegAngles ang = kin.ik(currentX, currentY);
    writeServosFromRad(ang.thigh, ang.knee);

    delay(100);   // 等舵机从无力状态走到站位（此处是上电后的第一次写舵机）
}

// 立即跳转（调试用）
void SingleLeg::setFootPositionImmediate(float x, float y) {
    currentX = x;
    currentY = y;
    LegAngles ang = kin.ik(x, y);
    writeServosFromRad(ang.thigh, ang.knee);
    isMovingFlag = false;
}

// 启动抓取动作（核心入口）：记录轨迹参数并开始计时。
// 若传入起点与当前坐标不同，会强制从传入起点开始，避免瞬间跳变。
void SingleLeg::startGrab(float sX, float sY,
                          float tX, float tY,
                          float lift,
                          TrajectoryMode mode,
                          unsigned long dur) {
    startX = sX;
    startY = sY;
    targetX = tX;
    targetY = tY;
    liftHeight = lift;
    currentMode = mode;

    startTime = millis();
    duration = dur;
    isMovingFlag = true;

    currentX = sX;
    currentY = sY;
}

// 执行单腿原语：方向(张/收)决定终点，形状(弧线/直线)决定轨迹模式与抬腿高度。
// 起点取当前足端，保证连续调用不跳变（如上一原语被打断）。
void SingleLeg::startPrimitive(LegPrimitive prim, unsigned long dur) {
    bool extend = (prim == PRIM_ARC_EXTEND || prim == PRIM_LINE_EXTEND);
    bool arc    = (prim == PRIM_ARC_EXTEND || prim == PRIM_ARC_RETRACT);

    double tx, ty;
    if (extend) extendedFoot(tx, ty);        // 张 → 伸出脚
    else        defaultStandingFoot(tx, ty); // 收 → 站立脚

    float lift = arc ? arcLift : 0.0f;
    TrajectoryMode mode = arc ? MODE_SWING : MODE_STANCE;

    startGrab(currentX, currentY, (float)tx, (float)ty, lift, mode, dur);
}

// 每帧推进轨迹：时间进度 → 平滑插值 → IK → 写舵机
void SingleLeg::update() {
    if (!isMovingFlag) return;

    // 1. 时间进度 (0~1)
    unsigned long now = millis();
    float progress = (now - startTime) / (float)duration;
    if (progress > 1.0) progress = 1.0;

    // 2. ease-in-out 平滑，起步 / 停止更柔和
    float t = progress * progress * (3 - 2 * progress);

    // 3. X 轴直线插值
    float x = startX + (targetX - startX) * t;

    // 4. Y 轴按模式区分
    float y;
    if (currentMode == MODE_SWING) {
        // 摆动相：直线高度上叠加正弦抬腿（两端低、中间高）
        float baseY = startY + (targetY - startY) * t;
        float arc = liftHeight * sin(PI * t);
        y = baseY + arc;
    } else {
        // 支撑相：锁死在目标高度，脚掌贴地防打滑
        y = targetY;
    }

    // 5~6. IK 求角并写入两个舵机
    LegAngles ang = kin.ik(x, y);
    writeServosFromRad(ang.thigh, ang.knee);

    // 7. 更新当前坐标
    currentX = x;
    currentY = y;

    // 8. 完成判定
    if (progress >= 1.0) {
        isMovingFlag = false;
        currentX = targetX;
        currentY = targetY;   // 保证最终落点精确
    }
}

void SingleLeg::writeServosFromRad(float thighRad, float kneeRad) {
    float thighDeg = thighRad * 180.0 / PI;   // 弧度 → 度（本函数唯一一次换算）
    float kneeDeg  = kneeRad  * 180.0 / PI;   // 弧度 → 度（同上）

    float servoThigh = thighDeg;              // 实物实测 0°=竖直向上、180°=竖直向下 → 站姿 = 67.38

    float servoknee = 180.0 - kneeDeg;

    // 偏移补偿（Config.h::ANGLE_OFFSET）与限幅 0~180 由 ServoManager 完成
    ServoManager::write(legId, ServoManager::FEMUR, servoThigh);
    ServoManager::write(legId, ServoManager::TIBIA, servoknee);
}
