/* 三角步态时序与相位分配（六足两组交替：A = 0d 15a 3b 24c，B = 0a 15d 3c 24b）
 *
 * 时序约定：一"拍"= 6 条腿同时各跑一条原语；TriGaitWork() 每帧推进全部腿，
 * 等 6 条腿全部 isMoving()==false 才启动下一拍（严格同步，不插帧）。
 * 记法：数字 = 腿号 0~5；a=弧线张(摆动前伸) b=弧线收(摆动收回)
 *       c=直线张(支撑伸出) d=直线收(支撑收回)。
 *
 * ★ SequentialOffset = 行走方向档：sqo = k 把整张相位表按腿序挪 k 档（物理腿 = (槽位+k) mod 6），
 *   机体被推往 60°×k、两拍同向，净推力 2×全行程（纯数学自检见 test/Gait6DirTest.cpp 的 verify）：
 *     sqo=0 → 0°（前） sqo=1 → 60° sqo=2 → 120° sqo=3 → 180° sqo=4 → 240° sqo=5 → 300°
 *   依据：支撑相 d（收）把机体推向该腿朝外 +ê、c（伸）推向 −ê，故净推力
 *   = Σ(槽位 d 的腿)+ê − Σ(槽位 c 的腿)ê；旋转整张表 = 旋转这个矢量和。
 *   前提：腿号按方位角每 60° 递增安装（leg0 正前、leg1 左前…）；实物反向时用 (6−k)%6。
 *
 * 2026-09 增量（本轮只加接口，**相位逻辑一行未改**）：
 *   1) TripodGait(SingleLeg* externalLegs, sqo, dur)：借用外部 6 条腿，便于上层共用同一批对象；
 *   2) steps()：构造以来已发出的拍数（Start / 每次切相位 +1），供上层观察相位/实现"只走 N 拍"。
 */

#include "TripodGait.h"
#include "SingleLeg.h"
#include "Arduino.h"


// 两个构造函数共用的初始化：方向档 sqo（= SequentialOffset）、单拍时长、施加 sqo 后的腿序表
void TripodGait::initCommon(int SequentialOffset, int StepDuration){
    sqo = SequentialOffset;
    stepDuration = StepDuration;
    if (StepDuration <= 0) stepDuration = 100;   // 非法时长兜底 100ms

    for(int i = 0; i < 6; i++){
        legIndices[i] = ((i + sqo)%6+6)%6;       // 确保顺序偏移为正数（i = 原始腿序）
    }
}

TripodGait::TripodGait(int SequentialOffset, int StepDuration){
    initCommon(SequentialOffset, StepDuration);  // legs 保持默认 = 自建的 ownLegs
}

// ★ 新增：用外部传进来的 6 条腿（不新建）。上层就能与 pos/servo/直控 共用同一批 SingleLeg 对象。
//   传 nullptr → 退回自建腿（等价于老构造函数）。
TripodGait::TripodGait(SingleLeg* externalLegs, int SequentialOffset, int StepDuration){
    if(externalLegs != nullptr) legs = externalLegs;
    initCommon(SequentialOffset, StepDuration);
}

void TripodGait::TriGaitInit(){//0a
    // 6 条腿统一站位：站立高度 10cm（写舵机 → 腿会实际动到站位）、抬腿弧高 3cm
    for(int i = 0; i < 6; i++){
        legs[i].init(10.0); // 初始化每条腿，站立高度按实际安装填，默认 10.0
        legs[i].setArcLift(3.0); // 设置抬腿高度，可根据实际调整
    }
}

void TripodGait::TriGaitStart(){//15a 3b 0d 24c
    // 第 1 拍 = A 组：leg0 贴地收(d)；leg1/leg5 抬腿前伸(a)；leg3 抬腿收(b)；leg2/leg4 贴地伸(c)
    legs[legIndices[0]].startPrimitive(PRIM_LINE_RETRACT, stepDuration); // leg0 d

    legs[legIndices[5]].startPrimitive(PRIM_ARC_EXTEND, stepDuration); // leg5 a
    legs[legIndices[1]].startPrimitive(PRIM_ARC_EXTEND, stepDuration); // leg1 a

    legs[legIndices[3]].startPrimitive(PRIM_ARC_RETRACT, stepDuration); // leg3 b

    legs[legIndices[2]].startPrimitive(PRIM_LINE_EXTEND, stepDuration); // leg2 c
    legs[legIndices[4]].startPrimitive(PRIM_LINE_EXTEND, stepDuration); // leg4 c

    stepCount++;   // ★ 记一拍（供上层用 steps() 观察相位推进 / 实现"只走 N 拍"）
}

void TripodGait::TriGaitWork(){
    // 每帧按 legIndices 顺序推进 6 条腿（顺序只影响写入次序，不影响相位）
    legs[legIndices[0]].update();
    legs[legIndices[2]].update();
    legs[legIndices[4]].update();
    legs[legIndices[1]].update();
    legs[legIndices[3]].update();
    legs[legIndices[5]].update();

    // 6 条腿全部停稳（严格同步）才切换到下一拍
    if(!legs[legIndices[0]].isMoving() && !legs[legIndices[2]].isMoving() && !legs[legIndices[4]].isMoving() && !legs[legIndices[1]].isMoving() && !legs[legIndices[3]].isMoving() && !legs[legIndices[5]].isMoving()){
        if(mode == 0){// 15a 3b 0d 24c
            // ⚠ 这里启动的仍是 A 组（与 TriGaitStart 的第一拍完全相同），
            //    因此上电后的实际时序是 A、A、B、A、B…（A 组多跑一拍），疑似分支写反。
            mode = 1;
            legs[legIndices[0]].startPrimitive(PRIM_LINE_RETRACT, stepDuration); // leg0 d
            legs[legIndices[5]].startPrimitive(PRIM_ARC_EXTEND, stepDuration); // leg5 a
            legs[legIndices[1]].startPrimitive(PRIM_ARC_EXTEND, stepDuration); // leg1 a
            legs[legIndices[3]].startPrimitive(PRIM_ARC_RETRACT, stepDuration); // leg3 b
            legs[legIndices[2]].startPrimitive(PRIM_LINE_EXTEND, stepDuration); // leg2 c
            legs[legIndices[4]].startPrimitive(PRIM_LINE_EXTEND, stepDuration); // leg4 c
            stepCount++;   // ★ 记一拍
        } else {//15d 3c 0a 24b
            // B 组：leg0 抬腿前伸(a)；leg1/leg5 贴地收(d)；leg3 贴地伸(c)；leg2/leg4 抬腿收(b)
            mode = 0;
            legs[legIndices[0]].startPrimitive(PRIM_ARC_EXTEND, stepDuration); // leg0 a
            legs[legIndices[5]].startPrimitive(PRIM_LINE_RETRACT, stepDuration); // leg5 d
            legs[legIndices[1]].startPrimitive(PRIM_LINE_RETRACT, stepDuration); // leg1 d
            legs[legIndices[3]].startPrimitive(PRIM_LINE_EXTEND, stepDuration); // leg3 c
            legs[legIndices[2]].startPrimitive(PRIM_ARC_RETRACT, stepDuration); // leg2 b
            legs[legIndices[4]].startPrimitive(PRIM_ARC_RETRACT, stepDuration); // leg4 b
            stepCount++;   // ★ 记一拍
        }
    }
}

void TripodGait::TriGaitStop(){
    // "停止"= 重新回站位（TriGaitInit 会写舵机，把 6 条腿摆回站姿）并把相位标记复位为 A。
    // ⚠ 注意：它不会清掉 SingleLeg 的 isMovingFlag —— 若调用时正有原语在跑，
    //    下一次 TriGaitWork() 会继续朝旧目标点走完，并不是立刻冻结。
    TriGaitInit();
    mode = 0;
}


/*
时序（数字=腿号，字母=原语；一拍 = 6 条腿同时各跑一条原语）

初始化：0a            ← TriGaitInit()：6 条腿回站姿（并非真的执行 a 原语）

15a 3b                ← A 组（TriGaitStart() 第一拍）
0d 24c

15d 3c                ← B 组
0a 24b

⚠ 实际运行次序：A（Start）→ A（Work 里 mode==0 分支）→ B → A → B → …，
   第二次 A 打在"已经到位的腿"上（腿都在上一次 A 的终点、目标点相同）→ 无水平位移，
   只有摆动相那三条腿原地抬一下，等于白走半拍；若确认是笔误，把 Work 里两个分支的启停内容对调即可。
   ★ 不改代码也想拿到严格 A,B,A,B…：**别调 TriGaitStart()**，直接 TriGaitInit() → 反复
     TriGaitWork()，第一次 Work() 就会发第一拍（mode==0 分支发的内容与 TriGaitStart 完全相同），
     之后交替正确。test/Gait6DirTest.cpp 用的就是这条路子。

*/
