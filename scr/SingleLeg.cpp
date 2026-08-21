/*# 单腿状态更新、足端轨迹规划（向前/向后抓）*/

#include "SingleLeg.h"
#include "Config.h"
#include <math.h>

// 构造函数：记住引脚，但先不 attach（延迟到 init 执行）
// 初始化列表的作用：
//   1) kin(robot::LENA, robot::LENB)：LegKinematics 只有带参构造函数，没有默认构造，
//      所以只能在初始化列表里“构造”它（函数体内只能给已构造的对象赋值）。
//   2) 让成员在进入函数体之前就准备好，避免“先默认构造再赋值”的多余开销；
//      对 const 成员、引用成员、无默认构造的类成员，初始化列表是唯一写法。
//   3) 成员的初始化顺序 = 它们在类里声明的顺序，与初始化列表书写顺序无关：
//      legId -> pinBase -> pinFemur -> pinTibia -> kin -> isMovingFlag -> ...
SingleLeg::SingleLeg(int id, int pinB, int pinF, int pinT)
    : legId(id)
    , pinBase(pinB)
    , pinFemur(pinF)
    , pinTibia(pinT)
    , kin(robot::LENA, robot::LENB)   // 腿长取自 Config.h，不再写死 4/3
    , isMovingFlag(false)
    , currentMode(MODE_STANCE) {
    // 构造函数体内啥也不干，等 init 再 attach 舵机
}

// ---------- 默认站立足端（小腿竖直，且与内圆相切） ----------
// 内圆半径 r = |LENA - LENB|（最小圆）。
// 想要 IK 算出来时小腿真正“竖直”（小腿线 = 膝到脚，dx=0），
// 取负侧切线 kx = -r：膝 = (kx, sqrt(LENA^2-r^2))，脚 = 膝正下方一个小腿长。
void SingleLeg::defaultStandingFoot(double& x, double& y) {
    const double r  = fabs(robot::LENA - robot::LENB);              // 内圆半径
    const double kx = -r;                                           // 竖直小腿的 x（与内圆相切，负侧）
    const double ky = sqrt(robot::LENA * robot::LENA - r * r);      // 膝的 y
    x = kx;
    y = ky - robot::LENB; // 脚 = 膝正下方一个小腿长
}

// ---------- 伸出足端（默认站立水平线 × 大圆的 2/3 分点） ----------
// 大圆半径 R = LENA + LENB。
// 过默认站立脚 (sx, sy) 画水平线 y = sy，与大圆同侧交点
//   xi = -sqrt(R^2 - sy^2)（因站立脚在负侧）。
// 在线段 [sx, xi] 上取 2/3 处：x = sx + (xi - sx) * 2/3。
void SingleLeg::extendedFoot(double& x, double& y) {
    double sx, sy;
    defaultStandingFoot(sx, sy);
    const double R  = robot::LENA + robot::LENB;   // 大圆半径
    const double xi = -sqrt(R * R - sy * sy);      // 同高度大圆交点（负侧）
    x = sx + (xi - sx) * (2.0 / 3.0);              // 2/3 分点
    y = sy;                                        // 保持同一高度
}

// ---------- 初始化 ----------
void SingleLeg::init(float standH) {
    // 1. 挂载舵机
    servoBase.attach(pinBase);
    servoFemur.attach(pinFemur);
    servoTibia.attach(pinTibia);

    // 2. 记录站立高度（比如 -3.0 表示脚在髋关节下方 3cm）
    standHeight = standH;

    // 3. 初始位置：按几何约束取默认站立足端
    //    （小腿竖直且与内圆相切，脚在髋节下方）
    double sx, sy;
    defaultStandingFoot(sx, sy);
    currentX = (float)sx;
    currentY = (float)sy;

    // 4. 计算 IK 并立即转到该位置
    LegAngles ang = kin.ik(currentX, currentY);
    writeServosFromRad(90.0, ang.thigh, ang.knee); // 基节舵机放中间（90°）

    delay(100); // 给舵机上电稳定时间
}

// ---------- 立即跳转（调试用） ----------
void SingleLeg::setFootPositionImmediate(float x, float y) {
    currentX = x;
    currentY = y;
    LegAngles ang = kin.ik(x, y);
    writeServosFromRad(90.0, ang.thigh, ang.knee);
    isMovingFlag = false;
}

// ---------- 启动抓取动作（最重要的函数） ----------
void SingleLeg::startGrab(float sX, float sY,
                          float tX, float tY,
                          float lift,
                          TrajectoryMode mode,
                          unsigned long dur) {
    // 记录起点和终点
    startX = sX;
    startY = sY;
    targetX = tX;
    targetY = tY;
    liftHeight = lift;
    currentMode = mode;

    // 重置计时器
    startTime = millis();
    duration = dur;
    isMovingFlag = true;

    // 注意：起点不一定等于 currentX/currentY。如果外部传入了不同的起点，
    // 我们可以在这里把 currentX/Y 强行设为 start，防止瞬间跳变。
    // 但对于连续步态，外部传入的起点通常就是 currentX/Y。
    currentX = sX;
    currentY = sY;
}

// ---------- 循环更新（每帧调用） ----------
void SingleLeg::update() {
    if (!isMovingFlag) return;

    // 1. 计算时间进度 (0.0 ~ 1.0)
    unsigned long now = millis();
    float progress = (now - startTime) / (float)duration;
    if (progress > 1.0) progress = 1.0;

    // 2. 平滑插值（ease-in-out），让起步和停止更柔和
    float t = progress * progress * (3 - 2 * progress);

    // 3. 计算 X 轴坐标（始终直线插值）
    float x = startX + (targetX - startX) * t;

    // 4. 计算 Y 轴坐标（根据模式决定）
    float y;
    if (currentMode == MODE_SWING) {
        // ===== 弧线模式（摆动相） =====
        // 先算基础直线高度
        float baseY = startY + (targetY - startY) * t;
        // 叠加正弦波抬腿（两端低，中间高）
        float arc = liftHeight * sin(PI * t);
        y = baseY + arc;
    } else {
        // ===== 平地模式（支撑相） =====
        // 直接锁死在目标高度（即站立高度），保证脚掌稳抓地面
        // 注意：为了更平滑，也可以写成 startY + (targetY - startY)*t，但会轻微起伏。
        // 舵机六足推荐直接锁死，防止打滑。
        y = targetY;
        // 如果你想看平滑过渡效果，可以把下面这行注释掉，用上面那行：
        // y = startY + (targetY - startY) * t;
    }

    // 5. 调用你的逆运动学（传入 x, y）
    LegAngles ang = kin.ik(x, y);

    // 6. 写入舵机（基节舵机保持 90° 中间位置，因为你这一步是在单一平面内移动）
    //    如果你以后要加转向，把 90.0 换成对应的水平旋转角度即可。
    writeServosFromRad(90.0, ang.thigh, ang.knee);

    // 7. 更新当前坐标（给下次循环作为起点参考）
    currentX = x;
    currentY = y;

    // 8. 判断是否结束
    if (progress >= 1.0) {
        isMovingFlag = false;
        currentX = targetX;
        currentY = targetY; // 确保最终落点精确
    }
}

// ---------- 私有方法：弧度转舵机角度并写入 ----------
void SingleLeg::writeServosFromRad(float baseAngleDeg, float thighRad, float kneeRad) {
    // 1. 弧度 转 度
    float thighDeg = thighRad * 180.0 / PI;
    float kneeDeg  = kneeRad  * 180.0 / PI;

    // 2. 映射到舵机坐标系 (0~180°)
    // 大腿：数学上水平向前为 0°，向上为正。舵机安装通常水平时为 90°，向下摆角度增大。
    // 所以舵机角度 = 90° - 数学角度。如果装反了，改成 90° + thighDeg。
    float servoThigh = 90.0 - thighDeg;

    // 小腿（膝盖）：数学上 0° 表示完全伸直，180° 表示完全折叠。
    // 舵机通常 0° 和 180° 对应极限位置。这里映射为 180° - 数学角度（让伸直时舵机在 180° 或 0°）。
    // 大部分舵机腿直立时膝盖舵机在 180° 附近，所以我用 180° - kneeDeg。
    // 如果装反了，改成 kneeDeg。
    float servoknee = 180.0 - kneeDeg;

    // 3. 基节舵机：你传入的 baseAngleDeg 直接输出（默认 90° 居中）
    float servoBase = baseAngleDeg;

    // 4. 限幅保护（防止舵机堵转）
    servoBase  = constrain(servoBase, 0, 180);
    servoThigh = constrain(servoThigh, 0, 180);
    servoknee  = constrain(servoknee, 0, 180);

    // 5. 写入舵机
    servoBase.write(servoBase);
    servoFemur.write(servoThigh);
    servoTibia.write(servoknee);
}
