/* 单腿状态更新与足端轨迹规划（方案 A：写入统一走 ServoManager） */

#include "SingleLeg.h"
#include "ServoManager.h"
#include "Config.h"
#include <math.h>

SingleLeg::SingleLeg(int id)
    : legId(id)
    , kin(robot::LENA, robot::LENB)   // 腿长取自 Config.h
    , currentMode(MODE_STANCE)
    , isMovingFlag(false) {}

// 默认站立足端：小腿竖直并与内圆相切。
// 内圆半径 r = |LENA-LENB|；取负侧切线 kx = -r，使 IK 解出的小腿真正竖直：
// 膝 = (kx, sqrt(LENA²-r²))，脚 = 膝正下方一个小腿长。
void SingleLeg::defaultStandingFoot(double& x, double& y) {
    const double r  = fabs(robot::LENA - robot::LENB);
    const double kx = -r;
    const double ky = sqrt(robot::LENA * robot::LENA - r * r);
    x = kx;
    y = ky - robot::LENB;
}

// 伸出足端：默认站立高度上，站立脚与达外圆同侧交点所成线段的 2/3 分点
void SingleLeg::extendedFoot(double& x, double& y) {
    double sx, sy;
    defaultStandingFoot(sx, sy);
    const double R  = robot::LENA + robot::LENB;   // 大圆半径
    const double xi = -sqrt(R * R - sy * sy);      // 同高度大圆交点（负侧）
    x = sx + (xi - sx) * (2.0 / 3.0);              // 2/3 分点
    y = sy;
}

// 初始化：attach 舵机 → 记录站立高度 → 复位到默认站立足端
void SingleLeg::init(float standH) {
    ServoManager::begin();                          // 幂等

    standHeight = standH;

    double sx, sy;
    defaultStandingFoot(sx, sy);
    currentX = (float)sx;
    currentY = (float)sy;

    LegAngles ang = kin.ik(currentX, currentY);
    writeServosFromRad(90.0, ang.thigh, ang.knee);  // 基节舵机居中 90°

    delay(100);   // 舵机上电稳定时间
}

// 立即跳转（调试用）
void SingleLeg::setFootPositionImmediate(float x, float y) {
    currentX = x;
    currentY = y;
    LegAngles ang = kin.ik(x, y);
    writeServosFromRad(90.0, ang.thigh, ang.knee);
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

    // 5~6. IK 求角并写入（基节保持 90° 居中；将来转向时替换该值）
    LegAngles ang = kin.ik(x, y);
    writeServosFromRad(90.0, ang.thigh, ang.knee);

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

// 弧度 → 舵机坐标系 → ServoManager 写入
void SingleLeg::writeServosFromRad(float baseAngleDeg, float thighRad, float kneeRad) {
    float thighDeg = thighRad * 180.0 / PI;
    float kneeDeg  = kneeRad  * 180.0 / PI;

    // 大腿：数学水平向前为 0°，向上为正；舵机安装水平时为 90°，
    // 故 舵机角 = 90° - 数学角（装反则改成 90° + thighDeg）。
    float servoThigh = 90.0 - thighDeg;

    // 小腿：数学 0° = 伸直、180° = 折叠；映射为 舵机角 = 180° - 数学角
    // （装反则改成 kneeDeg）。
    float servoknee = 180.0 - kneeDeg;

    // 偏移补偿与限幅由 ServoManager 完成
    ServoManager::write(legId, ServoManager::BASE,  baseAngleDeg);
    ServoManager::write(legId, ServoManager::FEMUR, servoThigh);
    ServoManager::write(legId, ServoManager::TIBIA, servoknee);
}
