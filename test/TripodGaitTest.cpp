TripodGaitTest.cpp
//三角步态test

#include <Arduino.h>
#include "TripodGait.h"

void setup() {
    Serial.begin(115200);
    Serial.println("Tripod Gait Test");
    TripodGait tripodGait(0, 500); // 顺序偏移为0，步长为500ms
    tripodGait.TriGaitInit(); // 初始化步态
    tripodGait.TriGaitStart(); // 启动步态
}

void loop() {
    // 1) 推进步态（每帧调用 update 完成插值并写舵机）
    tripodGait.TriGaitWork();
}

