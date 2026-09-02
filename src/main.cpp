/*
 * 六足机器人 统一主入口（PlatformIO）
 * ------------------------------------------------------------
 * 一个固件，多个测试模式。通过编译宏切换，改完宏保存后自动重新编译。
 *
 * 已集成模式（每个都是原 test/ 里对应文件移植而来）：
 *   -DMODE_SERVO_SWEEP   单舵机摆动测试          （原 ServoSweepTest.cpp）
 *   -DMODE_LEG_SWEEP     单腿两关节摆动测试
 *   -DMODE_SINGLE_LEG_GRAB 单腿不断抓地（行走/刨地循环）★新增
 *   -DMODE_TWO_LEGS      两腿顺序串联，跑一个周期  （原 TwoLegsTest.cpp）
 *   -DMODE_TWO_LEGS_CYCLE 两腿同步/反相无限循环   （原 TwoLegsCycleLoop.cpp）
 *   -DMODE_TRIPOD        完整三角步态（六足）     （原 TripodGaitTest.cpp）★默认
 *
 * 切换方法：改 platformio.ini 里 build_flags 的 "-DMODE_XXX"，
 *           然后重新 Upload 即可。
 *
 * 连接烧录：
 *   1. USB 插上 ESP32，确认驱动/COM 口
 *   2. VS Code 左下角选 env:esp32 → 点 "Upload"
 *   3. 打开 "Serial Monitor"（右下角，115200）可看日志
 */

#include <Arduino.h>

// ================= 运行时模式选择 =================
#if defined(MODE_SERVO_SWEEP)
    // ---- 模式1：单舵机扫描（原 ServoSweepTest）----
    #include "ServoManager.h"
    const int TARGET_LEG   = 0;
    const int TARGET_JOINT = ServoManager::FEMUR;
    const float SWEEP_MIN = 30.0f, SWEEP_MAX = 150.0f;
    const unsigned long SWEEP_MS = 500;
    bool heading_to_max = true;
    unsigned long last_sweep_time = 0;
    float current_angle = SWEEP_MIN;

    void setup() {
        Serial.begin(115200); delay(200);
        ServoManager::begin();
        Serial.printf("Mode=SERVO_SWEEP leg=%d joint=%s\n", TARGET_LEG,
                      (TARGET_JOINT == ServoManager::FEMUR) ? "FEMUR" : "TIBIA");
        ServoManager::write(TARGET_LEG, (ServoManager::Joint)TARGET_JOINT, SWEEP_MIN);
        current_angle = SWEEP_MIN;
        last_sweep_time = millis();
    }
    void loop() {
        unsigned long now = millis();
        if (now - last_sweep_time >= SWEEP_MS) {
            heading_to_max = !heading_to_max;
            last_sweep_time = now;
            current_angle = heading_to_max ? SWEEP_MIN : SWEEP_MAX;
        }
        float t = constrain((float)(now - last_sweep_time) / SWEEP_MS, 0.0f, 1.0f);
        current_angle = heading_to_max
                       ? SWEEP_MIN + (SWEEP_MAX - SWEEP_MIN) * t
                       : SWEEP_MAX - (SWEEP_MAX - SWEEP_MIN) * t;
        ServoManager::write(TARGET_LEG, (ServoManager::Joint)TARGET_JOINT, current_angle);
        static unsigned long last_print = 0;
        if (now - last_print > 200) { last_print = now; Serial.printf("angle=%.1f\n", current_angle); }
        delay(10);
    }

#elif defined(MODE_LEG_SWEEP)
    // ---- 模式2：单腿两关节摆动测试 ----
    // 不依赖 SingleLeg，直接驱动腿0的两个舵机在 30°~150° 对摆，
    // 用于快速确认某条腿的舵机与电源是否到位。
    #include "ServoManager.h"
    const unsigned long SWEEP_MS = 600;
    bool up = true;
    unsigned long prev = 0;
    float ang = 30.0f;
    void setup() {
        Serial.begin(115200); delay(200);
        ServoManager::begin();
        Serial.println("Mode=LEG_SWEEP leg0");
    }
    void loop() {
        unsigned long now = millis();
        if (now - prev >= SWEEP_MS) { up = !up; prev = now; }
        float t = constrain((float)(now - prev) / SWEEP_MS, 0.0f, 1.0f);
        ang = up ? 30.0f + 120.0f * t : 150.0f - 120.0f * t;
        ServoManager::write(0, ServoManager::FEMUR, ang);
        delay(10);
    }

#elif defined(MODE_SINGLE_LEG_GRAB)
    // ---- 模式3：单腿不断抓地（行走/刨地无限循环）----
    // 用 SingleLeg 驱动一条腿的两个舵机（腿0），反复执行：
    //   PRIM_ARC_EXTEND   摆动相：抬腿弧线向前伸出，脚悬空前抬
    //   PRIM_LINE_RETRACT 支撑相：贴地直线收回，脚掌抓地/推地
    // 循环往复即"不断抓地"的行进动作（类似原地踏步/刨地）。
    // 要换腿改 SingleLeg leg(编号)。
    #include "SingleLeg.h"
    SingleLeg leg(0);
    const unsigned long GRAB_DURATION = 600;      // 每段时长(ms)，调快慢
    int leg_step = 0;
    const int GRAB_STEPS = 2;
    const LegPrimitive GRAB_CYCLE[GRAB_STEPS] = { PRIM_ARC_EXTEND, PRIM_LINE_RETRACT };

    void setup() {
        Serial.begin(115200); delay(200);
        leg.init(10.0);               // 站立高度 10cm，可调
        leg.setArcLift(3.0);          // 抬腿高度 3cm
        leg.startPrimitive(GRAB_CYCLE[0], GRAB_DURATION);
        leg_step = 0;
        Serial.println("Mode=SINGLE_LEG_GRAB");
    }
    void loop() {
        leg.update();
        if (!leg.isMoving()) {
            leg_step = (leg_step + 1) % GRAB_STEPS;
            leg.startPrimitive(GRAB_CYCLE[leg_step], GRAB_DURATION);
        }
    }

#elif defined(MODE_TWO_LEGS)
    // ---- 模式4：两腿顺序串联，跑一次（原 TwoLegsTest）----
    #include "SingleLeg.h"
    SingleLeg legA(0), legB(1);
    const unsigned long PRIM_DURATION = 500;
    const int CYCLE_STEPS = 2;
    const LegPrimitive CYCLE[CYCLE_STEPS] = { PRIM_ARC_EXTEND, PRIM_LINE_RETRACT };
    bool legA_done = false, legB_running = false;
    int legA_step = 0, legB_step = 0;

    void setup() {
        Serial.begin(115200); delay(200);
        legA.init(10.0); legB.init(10.0);
        legA.setArcLift(3.0); legB.setArcLift(3.0);
        legA.startPrimitive(CYCLE[0], PRIM_DURATION);
        legA_step = 0;
        Serial.println("Mode=TWO_LEGS");
    }
    void loop() {
        if (!legA_done) {
            legA.update();
            if (!legA.isMoving()) {
                legA_step++;
                if (legA_step < CYCLE_STEPS) legA.startPrimitive(CYCLE[legA_step], PRIM_DURATION);
                else { legA_done = true; Serial.println("legA done"); }
            }
        }
        if (legA_done && !legB_running) {
            legB.startPrimitive(CYCLE[0], PRIM_DURATION);
            legB_step = 0; legB_running = true; Serial.println("legB started");
        }
        if (legB_running) {
            legB.update();
            if (!legB.isMoving()) {
                legB_step++;
                if (legB_step < CYCLE_STEPS) legB.startPrimitive(CYCLE[legB_step], PRIM_DURATION);
                else { Serial.println("legB done"); while (1) delay(100); }
            }
        }
    }

#elif defined(MODE_TWO_LEGS_CYCLE)
    // ---- 模式5：两腿同步/反相无限循环（原 TwoLegsCycleLoop）----
    #include "SingleLeg.h"
    SingleLeg legA(0), legB(1);
    const unsigned long PRIM_DURATION = 500;
    const int CYCLE_STEPS = 2;
    const LegPrimitive CYCLE[CYCLE_STEPS] = { PRIM_ARC_EXTEND, PRIM_LINE_RETRACT };
    #define PHASE_SYNC 0
    #define PHASE_OPPOSITE 1
    const int PHASE_MODE = PHASE_OPPOSITE;
    const int legB_seed = (PHASE_MODE == PHASE_OPPOSITE) ? (CYCLE_STEPS / 2) : 0;
    int legA_step = 0, legB_step = 0;

    void setup() {
        Serial.begin(115200); delay(200);
        legA.init(10.0); legB.init(10.0);
        legA.setArcLift(3.0); legB.setArcLift(3.0);
        legA_step = 0; legB_step = legB_seed;
        legA.startPrimitive(CYCLE[legA_step], PRIM_DURATION);
        legB.startPrimitive(CYCLE[legB_step], PRIM_DURATION);
        Serial.println(PHASE_MODE == PHASE_SYNC ? "Mode=TWO_LEGS_CYCLE SYNC" : "Mode=TWO_LEGS_CYCLE OPPOSITE");
    }
    void loop() {
        legA.update(); legB.update();
        if (!legA.isMoving()) { legA_step = (legA_step + 1) % CYCLE_STEPS; legA.startPrimitive(CYCLE[legA_step], PRIM_DURATION); }
        if (!legB.isMoving()) { legB_step = (legB_step + 1) % CYCLE_STEPS; legB.startPrimitive(CYCLE[legB_step], PRIM_DURATION); }
    }

#elif defined(MODE_TRIPOD)
    // ---- 模式6：完整三角步态（原 TripodGaitTest）----
    #include "TripodGait.h"
    TripodGait tripodGait(0, 500);
    void setup() {
        Serial.begin(115200);
        Serial.println("Mode=TRIPOD Tripod Gait Test");
        tripodGait.TriGaitInit();
        tripodGait.TriGaitStart();
    }
    void loop() {
        tripodGait.TriGaitWork();
    }

#else
    #error "未定义运行模式！请在 platformio.ini 的 build_flags 中定义 -DMODE_XXX"
#endif
