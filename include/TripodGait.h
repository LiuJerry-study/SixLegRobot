/*# 三角步态（六足）*/

#ifndef TRIPODGAIT_H
#define TRIPODGAIT_H

#include "SingleLeg.h"

class TripodGait {
public:
    TripodGait(int SequentialOffset, int StepDuration);

    void TriGaitWork();

    void TriGaitStart();

    void TriGaitInit();

    void TriGaitStop();
private:
    int sqo;
    int stepDuration;

    SingleLeg legs[6]= { SingleLeg(0), SingleLeg(1), SingleLeg(2), SingleLeg(3), SingleLeg(4), SingleLeg(5) };

    int legIndices[6] = { 0, 1, 2, 3, 4, 5 };
    bool mode=0; // 0: 15a 3b 0d 24c, 1: 15d 3c 0a 24b
};
#endif