#include "WalkMotion.h"
#include <algorithm>
#include <cmath>

namespace motion {
namespace {
const LegKinematics kinematics(robot::LENA, robot::LENB);
constexpr double pi = 3.14159265358979323846;
constexpr double degrees = 180.0 / pi;
constexpr double angleMargin = 0.25;
constexpr double maxDegreesPerSecond = 150.0;
constexpr int pathSamples = 48;

double smooth(double t) { return t * t * (3 - 2 * t); }
double unit(double t) { return std::max(0.0, std::min(1.0, t)); }

int groupOf(Gait gait, int leg) {
    if (gait == Gait::Five) return leg;
    if (gait == Gait::Four) return leg % 3;
    return (leg + 1) % 2; // 1/3/5 swing first, as in the legacy default.
}

double delta(const Settings& settings, int leg, float stride) {
    return stride * std::cos((settings.direction - settings.mirror * leg *
                             robot::LEG_SPACING_DEG) / degrees);
}

FootPos phaseFoot(const Settings& settings, int leg, int phase, float stride) {
    const int count = groups(settings.gait);
    const int sinceSwing = (phase - groupOf(settings.gait, leg) + count - 1) % count;
    return {robot::STAND_X - stride / 2.0 +
                delta(settings, leg, stride) * (double(sinceSwing) / (count - 1) - 0.5),
            -settings.height};
}

double swingWindow(const Settings& settings) {
    if (settings.duty > 0) return groups(settings.gait) * (1 - settings.duty);
    return 0.85;
}
}

JointCommand command(int leg, int joint, float logical) {
    const float requested = logical + robot::ANGLE_OFFSET[leg][joint];
    const float applied = std::max(robot::SERVO_MIN_DEG, std::min(robot::SERVO_MAX_DEG, requested));
    return {logical, applied, requested != applied};
}

bool solve(FootPos foot, float (&logical)[2]) {
    if (!std::isfinite(foot.x) || !std::isfinite(foot.y) || !kinematics.reachable(foot.x, foot.y)) return false;
    const LegAngles angles = kinematics.ik(foot.x, foot.y);
    logical[0] = angles.thigh * degrees;
    logical[1] = robot::SERVO_MAX_DEG - angles.knee * degrees;
    return logical[0] >= 0 && logical[0] <= robot::SERVO_MAX_DEG;
}

FootPos footFromCommands(int leg, float femur, float tibia) {
    const double thigh = (robot::THIGH_ZERO_DEG - femur + robot::ANGLE_OFFSET[leg][0]) / degrees;
    const double shin = thigh - (tibia - robot::ANGLE_OFFSET[leg][1]) / degrees;
    return {-robot::LENA * std::cos(thigh) - robot::LENB * std::cos(shin),
             robot::LENA * std::sin(thigh) + robot::LENB * std::sin(shin)};
}

bool Path::isSwinging(double progress) const {
    const double start = (1 - window) / 2;
    return swing && progress > start && progress < 1 - start;
}

FootPos Path::sample(double progress) const {
    const double t = swing ? unit((progress - (1 - window) / 2) / window) : unit(progress);
    const double u = smooth(t);
    const double height = legacy ? std::sin(pi * u) : 16.0 * t * t * (1 - t) * (1 - t);
    return {from.x + (to.x - from.x) * u,
            from.y + (to.y - from.y) * u + lift * height};
}

bool feasible(int leg, const Path& path, double margin, double& peakDegrees) {
    float last[2] = {};
    for (int i = 0; i <= pathSamples; ++i) {
        float angles[2];
        if (!solve(path.sample(double(i) / pathSamples), angles)) return false;
        for (int j = 0; j < 2; ++j) {
            const float requested = angles[j] + robot::ANGLE_OFFSET[leg][j];
            if (requested < robot::SERVO_MIN_DEG + margin || requested > robot::SERVO_MAX_DEG - margin) return false;
            if (i > 0) peakDegrees = std::max(peakDegrees, double(std::abs(angles[j] - last[j])) * pathSamples);
            last[j] = angles[j];
        }
    }
    return true;
}

unsigned long durationForPeak(double peakDegrees) {
    return static_cast<unsigned long>(std::ceil(1000 * peakDegrees / maxDegreesPerSecond));
}

const char* gaitName(Gait gait) {
    switch (gait) {
        case Gait::Legacy: return "legacy";
        case Gait::Five: return "five";
        case Gait::Four: return "four";
        case Gait::Three: return "three";
    }
    return "three";
}

const char* stageName(Stage stage) {
    switch (stage) {
        case Stage::Idle: return "idle";
        case Stage::Align: return "align";
        case Stage::Walk: return "walk";
        case Stage::Recover: return "recover";
        case Stage::Hold: return "hold";
    }
    return "hold";
}

int groups(Gait gait) {
    return gait == Gait::Five ? 6 : gait == Gait::Four ? 3 : 2;
}

char legacyPrimitive(int leg, int offset, unsigned beat) {
    return (beat % 2 == 0 ? "dacbca" : "adbcbd")[(leg - offset + 6) % 6];
}

void Walk::start(const FootPos (&feet)[robot::LEG_COUNT], const Settings& settings, int beats) {
    std::copy(feet, feet + robot::LEG_COUNT, feet_);
    settings_ = pending_ = settings;
    issued_ = completed_ = phase_ = leg_ = 0;
    target_ = beats;
    stage_ = Stage::Align;
    prepared_ = change_ = stop_ = false;
    support_ = 63;
}

void Walk::configure(const Settings& settings) {
    pending_ = settings;
    change_ = true;
}

void Walk::stop() {
    stop_ = true;
}

void Walk::hold(unsigned completed) {
    completed_ = completed;
    stage_ = Stage::Hold;
    support_ = 0;
}

void Walk::makePaths(float stride, float lift) {
    for (int i = 0; i < robot::LEG_COUNT; ++i) {
        auto& path = paths_[i];
        path = {feet_[i], feet_[i], 0, 1, false, false};
        if (stage_ == Stage::Walk && settings_.gait == Gait::Legacy) {
            const int heading = int(std::lround(settings_.direction / robot::LEG_SPACING_DEG));
            const int offset = (settings_.mirror * heading + 6) % 6;
            const char primitive = legacyPrimitive(i, offset, phase_);
            const double range = robot::LENA + robot::LENB;
            const double extended = robot::STAND_X + 2.0 / 3 *
                (-std::sqrt(range * range - settings_.height * settings_.height) - robot::STAND_X);
            path.to = {primitive == 'a' || primitive == 'c' ? extended : robot::STAND_X, -settings_.height};
            path.swing = primitive == 'a' || primitive == 'b';
            path.legacy = true;
        } else if (stage_ == Stage::Walk) {
            path.to = phaseFoot(settings_, i, phase_ + 1, stride);
            path.swing = groupOf(settings_.gait, i) == int(phase_);
            path.window = swingWindow(settings_);
        } else if (i == int(leg_)) {
            path.to = stage_ == Stage::Recover || settings_.gait == Gait::Legacy
                ? FootPos{robot::STAND_X, -settings_.height} : phaseFoot(settings_, i, 0, stride);
            path.swing = std::abs(path.to.x - path.from.x) > 0.001 || std::abs(path.to.y - path.from.y) > 0.001;
            path.window = 0.85;
        }
        if (path.swing) path.lift = lift;
    }
}

bool Walk::prepare(uint32_t now) {
    if (stage_ == Stage::Align && leg_ == 0) {
        stride_ = settings_.stride;
        lift_ = settings_.lift;
        if (settings_.gait != Gait::Legacy) {
            // Fit every nominal phase before prepositioning. Keep all radial projections in proportion.
            bool valid = false;
            for (int attempt = 0; attempt < 24 && !valid; ++attempt) {
                valid = true;
                for (int i = 0; i < robot::LEG_COUNT && valid; ++i) {
                    for (int p = 0; p < groups(settings_.gait) && valid; ++p) {
                        const bool swing = groupOf(settings_.gait, i) == p;
                        const Path path{phaseFoot(settings_, i, p, stride_), phaseFoot(settings_, i, p + 1, stride_),
                                        swing ? lift_ : 0.0, swingWindow(settings_), swing, false};
                        double peak = 0;
                        valid = feasible(i, path, angleMargin, peak);
                    }
                }
                if (!valid) {
                    if (attempt % 2 == 0) lift_ *= 0.75f;
                    else stride_ *= 0.75f;
                }
            }
            if (!valid) return false;
        }
    }

    duration_ = std::max(settings_.beatMs, framePeriodMs);
    const bool legacyBeat = stage_ == Stage::Walk && settings_.gait == Gait::Legacy;
    double lift = stage_ == Stage::Walk ? lift_ : std::min(1.0f, lift_);
    bool valid = false;
    double peak = 0;
    for (int attempt = 0; attempt < 7 && !valid; ++attempt) {
        makePaths(stride_, lift);
        peak = 0;
        valid = true;
        for (int i = 0; i < robot::LEG_COUNT && valid && !legacyBeat; ++i) {
            valid = feasible(i, paths_[i], stage_ == Stage::Walk ? angleMargin : 0, peak);
        }
        if (stage_ == Stage::Walk) break;
        lift = attempt < 5 ? lift * 0.5 : 0;
    }
    if (!valid) return false;
    duration_ = std::max(duration_, uint32_t(durationForPeak(peak)));
    // A leg already at its transfer target needs no timed movement.
    if (stage_ != Stage::Walk && !paths_[leg_].swing) duration_ = 0;
    start_ = now;
    prepared_ = true;
    if (stage_ == Stage::Walk) ++issued_;
    return true;
}

bool Walk::tick(uint32_t now) {
    if (!active()) return false;
    if (!prepared_) {
        if (stop_ && stage_ != Stage::Recover) { stage_ = Stage::Recover; leg_ = 0; }
        if (change_ && !stop_ && stage_ == Stage::Walk) {
            settings_ = pending_;
            stage_ = Stage::Align;
            leg_ = phase_ = 0;
            change_ = false;
        }
        if (!prepare(now)) {
            hold(completed_);
            return false;
        }
    }

    const double progress = duration_ ? unit(double(now - start_) / duration_) : 1;
    support_ = 0;
    for (int i = 0; i < robot::LEG_COUNT; ++i) {
        feet_[i] = paths_[i].sample(progress);
        if (!paths_[i].isSwinging(progress)) support_ |= 1u << i;
    }
    if (progress < 1) return true;

    prepared_ = false;
    if (stage_ == Stage::Walk) {
        ++completed_;
        phase_ = (phase_ + 1) % groups(settings_.gait);
        if (target_ > 0 && completed_ >= unsigned(target_)) stop_ = true;
    } else if (++leg_ == robot::LEG_COUNT) {
        stage_ = stage_ == Stage::Align ? Stage::Walk : Stage::Idle;
        leg_ = 0;
    }
    return true;
}
}
