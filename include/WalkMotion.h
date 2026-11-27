#pragma once

#include <stdint.h>
#include "LegKinematics.h"
#include "RobotModel.h"

namespace motion {
struct JointCommand {
    float logical;
    float applied;
    bool limited;
};

JointCommand command(int leg, int joint, float logical);
bool solve(FootPos foot, float (&logical)[2]);
FootPos footFromCommands(int leg, float femur, float tibia);

enum class Gait { Legacy, Five, Four, Three };
enum class Stage { Idle, Align, Walk, Recover, Hold };

struct Settings {
    Gait gait = Gait::Three;
    float direction = 0;
    int mirror = 1;
    float stride = 3;
    float lift = 3;
    float height = -robot::STAND_Y;
    float duty = 0; // 0 selects the preset default.
    uint32_t beatMs = 600;
};

struct Path {
    FootPos from;
    FootPos to;
    double lift;
    double window;
    bool swing;
    bool legacy;
    FootPos sample(double progress) const;
    bool isSwinging(double progress) const;
};

bool feasible(int leg, const Path& path, double margin, double& peakDegrees);
unsigned long durationForPeak(double peakDegrees);
const char* gaitName(Gait gait);
const char* stageName(Stage stage);
int groups(Gait gait);
char legacyPrimitive(int leg, int offset, unsigned beat);
constexpr uint32_t framePeriodMs = 20;

class Walk {
public:
    void start(const FootPos (&feet)[robot::LEG_COUNT], const Settings& settings, int beats);
    void configure(const Settings& settings);
    void stop();
    void cancel() { stage_ = Stage::Idle; }
    void hold(unsigned completed);
    bool tick(uint32_t now);
    bool active() const { return stage_ == Stage::Align || stage_ == Stage::Walk || stage_ == Stage::Recover; }
    Stage stage() const { return stage_; }
    unsigned completed() const { return completed_; }
    unsigned issued() const { return issued_; }
    unsigned supportMask() const { return support_; }
    uint32_t duration() const { return duration_; }
    float stride() const { return stride_; }
    float lift() const { return lift_; }
    FootPos foot(int i) const { return feet_[i]; }

private:
    bool prepare(uint32_t now);
    void makePaths(float stride, float lift);

    Settings settings_, pending_;
    FootPos feet_[robot::LEG_COUNT];
    Path paths_[robot::LEG_COUNT];
    Stage stage_ = Stage::Idle;
    unsigned issued_ = 0, completed_ = 0, phase_ = 0, leg_ = 0, support_ = 0;
    int target_ = 0;
    bool prepared_ = false, change_ = false, stop_ = false;
    uint32_t start_ = 0, duration_ = 0;
    float stride_ = 0, lift_ = 0;
};
}
