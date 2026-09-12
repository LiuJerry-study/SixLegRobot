/* 三角步态（六足）：把 6 条 SingleLeg 分成两组交替迈步。
 *
 * 相位记法（数字 = 腿号 0~5，字母 = 原语 a/b/c/d）：
 *   A 组：0d 15a 3b 24c      B 组：0a 15d 3c 24b
 *   （a=PRIM_ARC_EXTEND 摆动抬腿前伸，b=PRIM_ARC_RETRACT 摆动抬腿收回，
 *     c=PRIM_LINE_EXTEND 支撑贴地伸出，d=PRIM_LINE_RETRACT 支撑贴地收回）
 *
 * ★ SequentialOffset 就是"行走方向"：sqo = k → 整张相位表按腿序循环挪 k 档
 *   （物理腿 = (槽位 + sqo) mod 6），机体被推往 60°×k 方向、两拍同向（净推力 2×行程）。
 *   推导：支撑相里 d（收）把机体推向该腿朝外方向 +ê、c（伸）推向 −ê，
 *   所以净推力 = Σ(槽位 d 的腿) +ê − Σ(槽位 c 的腿) ê；sqo 旋转整张表 = 旋转该矢量和。
 *   纯数学自检见 test/Gait6DirTest.cpp 的 `verify` 命令（sqo=0..5 → 0°/60°/…/300°）。
 *   前提：腿号按方位角每 60° 递增安装（leg0 正前、leg1 左前…）；若实物是反的，
 *   把 sqo 取 (6−k)%6 即得同一物理方向。sqo 与 StepDuration 都是构造后固定。
 */
#ifndef TRIPODGAIT_H
#define TRIPODGAIT_H

#include "SingleLeg.h"

class TripodGait {
public:
    // SequentialOffset：腿序整体循环偏移（mod 6，可为负），0 = 腿号即顺序；它同时是方向档
    // StepDuration：每条原语的动作时长(ms)，≤0 时按 100ms 处理
    TripodGait(int SequentialOffset, int StepDuration);

    // ★ 新增（2026-09，纯增量、老行为完全不变）：借用**外部**的 6 条 SingleLeg 来驱动，
    //   自己不新建腿。上层（test/Gait6DirTest.cpp）借此与 pos / servo / 舵机直控 共用同一批
    //   SingleLeg 对象 —— 步态跑起来时读到的足端坐标才与舵机实际姿态一致；传 nullptr 则退回自建腿。
    TripodGait(SingleLeg* externalLegs, int SequentialOffset, int StepDuration);

    void TriGaitWork();   // 每帧调用（放 loop）：推进全部 6 条腿，全部停稳后切下一组

    void TriGaitStart();  // 启动 A 组（0d 15a 3b 24c）
                          // ⚠ 它发完这一拍后，TriGaitWork() 的 mode==0 分支还会再发一次 A 组；
                          //   那第二次 A 打在"已经到位的腿"上（无水平位移，只有摆动腿原地抬一下），
                          //   等于白走半拍。想避免就**别调 TriGaitStart()**：直接
                          //   TriGaitInit() → 反复 TriGaitWork()，第一次 Work 就会发第一拍，
                          //   之后序列严格 A,B,A,B…（test/Gait6DirTest.cpp 就是这么用的）。

    void TriGaitInit();   // 6 条腿全部 init(站立高度 10cm) 并 setArcLift(3cm) → 摆到站位
                          // （会写舵机、每条腿 delay(100)；用外部腿时同样作用于外部那 6 条）

    void TriGaitStop();   // 回站位（内部就是再调一次 TriGaitInit）并把 mode 复位为 0
                          // ⚠ 不清 SingleLeg 的 isMovingFlag：若调用时正有原语在跑，
                          //   下一次 TriGaitWork()/update() 会继续朝旧目标走完。上层停完步态后
                          //   建议再 setFootPositionImmediate() 一次兜底（见 Gait6DirTest.cpp）。

    int  steps() const { return stepCount; }   // ★ 新增：构造以来已发出的拍数（Start / 每次切相位 +1）
    int  stepDurationMs() const { return stepDuration; }
    int  sequentialOffset() const { return sqo; }
private:
    int sqo;              // 腿序循环偏移 = 方向档（构造后固定）
    int stepDuration;     // 单段原语时长(ms)

    // 自建腿（用"外部腿"构造函数时闲置不用，仅被构造出来占内存）
    SingleLeg ownLegs[6] = { SingleLeg(0), SingleLeg(1), SingleLeg(2), SingleLeg(3), SingleLeg(4), SingleLeg(5) };
    SingleLeg* legs = ownLegs;   // 真正被驱动的那 6 条腿（默认 = 自建腿；可指向外部数组）

    int legIndices[6] = { 0, 1, 2, 3, 4, 5 };   // 施加 sqo 后的实际腿序（TriGaitWork 全部按它取腿）
    bool mode=0; // 当前相位标记：0 = A 组阶段(15a 3b 0d 24c)，1 = B 组阶段(15d 3c 0a 24b)
                 // ⚠ 注意：TriGaitWork 里 mode==0 的分支启动的仍是 A 组，
                 //   而 TriGaitStart() 已经先跑过一次 A 组 → 若调了 Start，实际时序是 A,A,B,A,B…；
                 //   只用 Init + Work 起步则严格 A,B,A,B…（见上面 TriGaitStart 的说明）
    int stepCount = 0;   // 已发出的拍数（TriGaitStart / TriGaitWork 每发一拍 +1）

    void initCommon(int SequentialOffset, int StepDuration);   // 两个构造函数共用的初始化
};
#endif
