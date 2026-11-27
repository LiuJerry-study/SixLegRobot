/*
 * 六足机器人 统一主入口（PlatformIO）
 * ------------------------------------------------------------
 * 一个固件，多个测试模式。通过编译宏切换，改完宏保存后自动重新编译。
 *
 * 已集成模式（每个都是原 test/ 里对应文件移植而来）：
 *   -DMODE_SERVO_SWEEP   单舵机摆动测试          （原 ServoSweepTest.cpp）
 *   -DMODE_LEG_SWEEP     单腿摆动测试（只往复动 leg0 的大腿舵机 30°~150°，小腿不动）
 *   -DMODE_SINGLE_LEG_GRAB 单腿不断抓地（行走/刨地循环）★新增
 *   -DMODE_TWO_LEGS      两腿顺序串联，跑一个周期  （原 TwoLegsTest.cpp）
 *   -DMODE_TWO_LEGS_CYCLE 两腿同步/反相无限循环   （原 TwoLegsCycleLoop.cpp）
 *   -DMODE_TRIPOD        完整三角步态（六足）     （原 TripodGaitTest.cpp）
 *   -DMODE_SERVO_TEST    通用舵机测试（扫描/同步/串口控制）★新增
 *   -DMODE_SINGLE_SERVO_TEST 平台单舵机测试（串口交互式，测任意 1 路）★新增
 *   -DMODE_SINGLE_LEG_TEST   可切换多腿测试（legs 运行时选 1~6 条腿）★新增
 *   -DMODE_SIX_LEGS_SYNC     六腿同步测试（单腿测试动作扩展为 6 腿同步）★新增
 *   -DMODE_GAIT_6DIR         六方向步态测试（SingleLegTest 全部功能 + 三角步态 + 6 个行走方向）★新增
 *   -DMODE_MASTER_CONTROL    六足主控台（方向行走 + 选腿控制 + 单舵机控制，专门配合 gait-sim 网页）★新增
 *
 *  ★ PlatformIO 用法：见 platformio.ini，改 build_flags 里的 -DMODE_XXX 即可。
 *    舵机测试的串口命令：
 *       sweep      → 通道逐个扫描     all → 全部同步扫描     mode 3 → 串口精确控制
 *       <通道> <角> → 串口控制模式下精确转动（如 "0 90"）
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
    // 经 ServoManager 驱动 leg0 大腿（ch0）在 30°~150° 之间匀速往复，
    // 每 500ms 换向；因此会叠加 Config.h 的零位偏移，并回传 @SERVO 遥测。
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
    // ---- 模式2：单腿摆动测试 ----
    // 不依赖 SingleLeg，直接经 ServoManager 让 leg0 的**大腿舵机**在 30°~150° 之间
    // 往复摆动（小腿舵机全程不动），用于快速确认该路舵机与电源是否到位。
    // 想连小腿一起动请用 MODE_SINGLE_SERVO_TEST / MODE_SINGLE_LEG_TEST。
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
    // legA(0) 先跑完 ARC_EXTEND → LINE_RETRACT 两段，再轮到 legB(1) 跑同样两段；
    // 两条腿全部跑完后进入 while(1) 停住（不循环）。
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
    // legA(0)、legB(1) 各自无限循环 ARC_EXTEND → LINE_RETRACT 两段；
    // PHASE_MODE 决定相位：PHASE_SYNC 两腿同步，PHASE_OPPOSITE（当前生效）
    // 让 legB 从半步后的原语起步，即两腿反相（交替迈步）。
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

#elif defined(MODE_SERVO_TEST)
    // ---- 模式7：通用舵机测试（PCA9685 驱动，多通道）----
    // 自带一套最小的 PCA9685 驱动逻辑（自实现 svAngleToPulse/svWrite），
    // **刻意不经过 ServoManager**：因此不吃 Config.h 的零位偏移、也不会回传 @SERVO 遥测，
    // 适合裸测通道/接线。串口命令：
    //   sweep → 通道逐个扫描；all → 全部同步扫描；mode 3 → 串口精确控制
    //   串口控制模式下输入 "<通道> <角度>"（如 "0 90"）精确转到某角度
    // 上电时 setup 里会先把 12 路全部写到 90°（中位）。
    #include <Wire.h>
    #include <Adafruit_PWMServoDriver.h>
    #include "Config.h"

    Adafruit_PWMServoDriver servoPwm = Adafruit_PWMServoDriver(PCA9685_I2C_ADDR);
    const int SERVO_TEST_COUNT = 12;   // 测试的通道数（按接线改，最多16）
    int servoTestMode = 1;             // 1=逐个扫描 2=同步扫描 3=串口控制
    int8_t sv_goingUp[SERVO_TEST_COUNT];
    int sv_angle[SERVO_TEST_COUNT];
    bool sv_inited = false;

    int svAngleToPulse(float deg) {
        deg = constrain(deg, 0.0f, 180.0f);
        return (int)(SERVO_MIN_PULSE_CNT + (SERVO_MAX_PULSE_CNT - SERVO_MIN_PULSE_CNT) * (deg / 180.0f));
    }

    void svWrite(int ch, float deg) {
        if (ch < 0 || ch > 15) return;
        servoPwm.setPWM(ch, 0, svAngleToPulse(deg));
    }

    void svSweepStep(int channel) {
        if (!sv_inited) {
            for (int c = 0; c < SERVO_TEST_COUNT; c++) { sv_angle[c] = 30; sv_goingUp[c] = 1; }
            sv_inited = true;
        }
        sv_angle[channel] += sv_goingUp[channel];
        if (sv_angle[channel] >= 150) { sv_angle[channel] = 150; sv_goingUp[channel] = -1; }
        if (sv_angle[channel] <= 30)  { sv_angle[channel] = 30;  sv_goingUp[channel] = 1; }
        svWrite(channel, sv_angle[channel]);
    }

    void svCenterAll() {
        for (int c = 0; c < SERVO_TEST_COUNT; c++) svWrite(c, 90.0f);
    }

    bool svIsNumeric(const String& s) {
        if (s.length() == 0) return false;
        for (unsigned int i = 0; i < s.length(); i++) {
            char ch = s[i];
            if (!(ch >= '0' && ch <= '9') && ch != '-' && ch != '.') return false;
        }
        return true;
    }

    void svHandle(const String& line) {
        if (line.equalsIgnoreCase("sweep")) { servoTestMode = 1; svCenterAll(); Serial.println("Mode: AUTO SWEEP"); return; }
        if (line.equalsIgnoreCase("all"))   { servoTestMode = 2; svCenterAll(); Serial.println("Mode: ALL SWEEP"); return; }
        if (line.equalsIgnoreCase("mode 3")){ servoTestMode = 3; Serial.println("Mode: SERIAL (<ch> <angle>)"); return; }
        int sp = line.indexOf(' ');
        if (sp <= 0) { Serial.println("! 格式 <通道> <角度> 或 sweep/all/mode 3"); return; }
        String a = line.substring(0, sp); a.trim();
        String b = line.substring(sp + 1);  b.trim();
        if (!svIsNumeric(a) || !svIsNumeric(b)) { Serial.println("! 请输入数字"); return; }
        int ch = a.toInt(); float ang = b.toFloat();
        if (ch < 0 || ch >= SERVO_TEST_COUNT) { Serial.println("! 通道越界"); return; }
        ang = constrain(ang, 0.0f, 180.0f);
        servoTestMode = 3;
        svWrite(ch, ang);
        Serial.printf("CH%d -> %.0f\n", ch, ang);
    }

    void setup() {
        Serial.begin(115200); delay(200);
        Wire.begin(PCA9685_SDA_PIN, PCA9685_SCL_PIN);
        servoPwm.begin();
        servoPwm.setOscillatorFrequency(PCA9685_OSC_FREQ_HZ);
        servoPwm.setPWMFreq(PCA9685_PWM_FREQ_HZ);
        svCenterAll();
        Serial.println("Mode=SERVO_TEST");
        Serial.println("sweep=逐个扫描  all=同步扫描  mode 3=串口控制(<ch> <angle>)");
        Serial.println("初始=逐个扫描");
    }

    void loop() {
        if (Serial.available() > 0) {
            String line = Serial.readStringUntil('\n'); line.trim();
            if (line.length() > 0) svHandle(line);
            delay(5);
        }
        static unsigned long prev = 0;
        unsigned long now = millis();
        if (now - prev < 15) return;
        prev = now;
        if (servoTestMode == 1) { static int ch = 0; svSweepStep(ch); ch = (ch + 1) % SERVO_TEST_COUNT; }
        else if (servoTestMode == 2) { for (int c = 0; c < SERVO_TEST_COUNT; c++) svSweepStep(c); }
    }

#elif defined(MODE_SINGLE_SERVO_TEST)
    // ---- 模式8：平台单舵机测试（串口交互式）----
    // 串口命令：help / map / center / sweep <ch> / sweep leg <l> <j> /
    //           stop / <ch> <angle> / leg <l> <j> <angle>
    // 完整说明见 test/SingleServoTest.cpp（自包含文件，此处直接引用）
    #include "../test/SingleServoTest.cpp"

#elif defined(MODE_SINGLE_LEG_TEST)
    // ---- 模式9：可切换多腿测试（串口交互式）----
    // 串口命令：legs(运行时选择被测腿: all / 0 / 0 1 2 / 0-3) / each(逐腿巡检) /
    //           cycle / one / home / goto <x> <y> / lift <cm> / dur <ms> /
    //           0|1 <角度>(选中腿舵机直控) / pos / stop
    // 选中 1 条腿 = 单腿测试；选中 2~6 条 = 严格同步一起动。
    // 完整说明见 test/SingleLegTest.cpp（自包含文件，此处直接引用）
    #include "../test/SingleLegTest.cpp"

#elif defined(MODE_GAIT_6DIR)
    // ---- 模式11：六方向步态测试（串口交互式）----
    // = SingleLegTest 的全部功能（legs 选腿 / cycle / one / each / home / goto / lift / dur /
    //   0|1 舵机直控 / pos / servo [watch|off] / stop）
    //   + 三角步态（6 腿自动分两组交替支撑，时序严格 A,B,A,B…，修掉了 TripodGait 的 A,A,B 问题）
    //   + 6 个行走方向（行程系数投影法 cᵢ = cos(φ−αᵢ)，任意 φ 走直线）
    // 三角步态命令：tripod|tri [one|step|stop] / phase / stride <cm> / verify
    // 方向命令：f fl bl b br fr（0/60/120/180/240/300°）/ dir <角度> / dirs / mirror
    // 完整说明见 test/Gait6DirTest.cpp（自包含文件，此处直接引用）
    #include "../test/Gait6DirTest.cpp"

#elif defined(MODE_MASTER_CONTROL)
    // ---- 模式12：六足主控台（方向行走 + 选腿控制 + 单舵机控制）----
    // 三块能力都在一个串口命令集里，专门给 gait-sim 网页（悬空孪生 → 固件模式选「主控台」）驱动：
    //   A) 方向行走: f fl bl b br fr / dir <角度> / walk [n] / step / phase / mirror
    //   B) 选腿控制: legs [all|none|<n...>|<a-b>] / cycle / one / each / home / goto <x> <y>
    //   C) 单舵机  : sv <leg> <joint> <角度> / ch <通道> <角度> / lsv <leg> <a0> <a1> /
    //                asv <joint> <角度> / center
    //   另有 pos / map / state / verify / lift / dur / stop。
    // 舵机指令通过 ServoManager::write() 下发。snapshot 开启版本化 @FRAME 遥测，
    // @STATE 报告拍数、步态参数与遥控有效期。
    // 完整说明见 test/MasterControl.cpp 文件头（自包含文件，此处直接引用）
    #include "../test/MasterControl.cpp"

#elif defined(MODE_SIX_LEGS_SYNC)
    // ---- 模式10：六腿同步测试（串口交互式）----
    // 单腿测试的动作扩展为 6 腿同步：所有腿同一时刻做同一动作，
    // 每步等 6 腿全部完成后一起进入下一步（严格同步）。
    // 串口命令：help / cycle / one / home / goto <x> <y> / lift <cm> /
    //           dur <ms> / 0 <角度>(6条大腿直控) / 1 <角度>(6条小腿直控) /
    //           pos / stop
    // 完整说明见 test/SixLegsSyncTest.cpp（自包含文件，此处直接引用）
    #include "../test/SixLegsSyncTest.cpp"

#else
    #error "未定义运行模式！请在 platformio.ini 的 build_flags 中定义 -DMODE_XXX"
#endif
