/* # 三角步态时序、相位分配*/

#include "TripodGait.h"
#include "SingleLeg.h"
#include "Arduino.h"


TripodGait::TripodGait(int SequentialOffset, int StepDuration){
    sqo = SequentialOffset; 
    stepDuration = StepDuration;
    if (StepDuration <= 0) stepDuration = 100;

    for(int i = 0; i < 6; i++){
        legIndices[i] = ((legIndices[i]+sqo)%6+6)%6; // 确保顺序偏移为正数
    }

}

void TripodGait::TriGaitInit(){//0a
    for(int i = 0; i < 6; i++){
        legs[i].init(10.0); // 初始化每条腿，站立高度按实际安装填，默认 10.0
        legs[i].setArcLift(3.0); // 设置抬腿高度，可根据实际调整
    }
}

void TripodGait::TriGaitStart(){//15a 3b 0d 24c
    legs[legIndices[0]].startPrimitive(PRIM_LINE_RETRACT, stepDuration); // leg0 d

    legs[legIndices[5]].startPrimitive(PRIM_ARC_EXTEND, stepDuration); // leg5 a
    legs[legIndices[1]].startPrimitive(PRIM_ARC_EXTEND, stepDuration); // leg1 a

    legs[legIndices[3]].startPrimitive(PRIM_ARC_RETRACT, stepDuration); // leg3 b

    legs[legIndices[2]].startPrimitive(PRIM_LINE_EXTEND, stepDuration); // leg2 c
    legs[legIndices[4]].startPrimitive(PRIM_LINE_EXTEND, stepDuration); // leg4 c
}

void TripodGait::TriGaitWork(){
    legs[legIndices[0]].update();
    legs[legIndices[2]].update();
    legs[legIndices[4]].update();
    legs[legIndices[1]].update();
    legs[legIndices[3]].update();
    legs[legIndices[5]].update();

    if(!legs[legIndices[0]].isMoving() && !legs[legIndices[2]].isMoving() && !legs[legIndices[4]].isMoving() && !legs[legIndices[1]].isMoving() && !legs[legIndices[3]].isMoving() && !legs[legIndices[5]].isMoving()){
        if(mode == 0){// 15a 3b 0d 24c
            mode = 1;
            legs[legIndices[0]].startPrimitive(PRIM_LINE_RETRACT, stepDuration); // leg0 d
            legs[legIndices[5]].startPrimitive(PRIM_ARC_EXTEND, stepDuration); // leg5 a
            legs[legIndices[1]].startPrimitive(PRIM_ARC_EXTEND, stepDuration); // leg1 a
            legs[legIndices[3]].startPrimitive(PRIM_ARC_RETRACT, stepDuration); // leg3 b
            legs[legIndices[2]].startPrimitive(PRIM_LINE_EXTEND, stepDuration); // leg2 c
            legs[legIndices[4]].startPrimitive(PRIM_LINE_EXTEND, stepDuration); // leg4 c
        } else {//15d 3c 0a 24b
            mode = 0;
            legs[legIndices[0]].startPrimitive(PRIM_ARC_EXTEND, stepDuration); // leg0 a
            legs[legIndices[5]].startPrimitive(PRIM_LINE_RETRACT, stepDuration); // leg5 d
            legs[legIndices[1]].startPrimitive(PRIM_LINE_RETRACT, stepDuration); // leg1 d
            legs[legIndices[3]].startPrimitive(PRIM_LINE_EXTEND, stepDuration); // leg3 c
            legs[legIndices[2]].startPrimitive(PRIM_ARC_RETRACT, stepDuration); // leg2 b
            legs[legIndices[4]].startPrimitive(PRIM_ARC_RETRACT, stepDuration); // leg4 b
        }
    }
}

void TripodGait::TriGaitStop(){
    TriGaitInit();
    mode = 0;
}


/*
初始化：0a

15a 3b
0d 24c

15d 3c
0a 24b

*/
