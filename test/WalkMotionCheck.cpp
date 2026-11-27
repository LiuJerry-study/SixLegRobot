#include "MotionFrame.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

using namespace motion;

static const FootPos stand[6] = {{-6, -10}, {-6, -10}, {-6, -10}, {-6, -10}, {-6, -10}, {-6, -10}};

static double supportMargin(const Walk& walk, int mirror) {
    FootPos points[6];
    int count = 0;
    for (int leg = 0; leg < 6; ++leg) {
        if (!(walk.supportMask() & (1u << leg))) continue;
        const double azimuth = mirror * leg * robot::LEG_SPACING_DEG * std::acos(-1.0) / 180;
        const double radius = 7.4 - walk.foot(leg).x; // The browser model's assumed hip radius.
        points[count++] = {radius * std::cos(azimuth), -radius * std::sin(azimuth)};
    }
    double margin = 100;
    for (int i = 0; i < count; ++i) {
        for (int j = 0; j < count; ++j) {
            if (i == j) continue;
            const auto a = points[i], b = points[j];
            bool edge = true;
            for (int k = 0; k < count; ++k) {
                const auto p = points[k];
                if ((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x) < -1e-8) edge = false;
            }
            if (edge) {
                const double distance = (a.x * b.y - a.y * b.x) / std::hypot(b.x - a.x, b.y - a.y);
                margin = std::min(margin, distance);
            }
        }
    }
    assert(count >= 3 && margin < 100);
    return margin;
}

static void printFrame(const Walk& walk, unsigned sequence, unsigned now, int mirror) {
    JointCommand commands[16];
    FootPos feet[6];
    Frame frame{};
    frame.sequence = sequence;
    frame.ms = now;
    frame.stage = walk.stage();
    frame.issued = walk.issued();
    frame.completed = walk.completed();
    frame.support = walk.supportMask();
    frame.mirror = mirror;
    for (int leg = 0; leg < 6; ++leg) {
        feet[leg] = walk.foot(leg);
        frame.requested[leg] = &feet[leg];
        float angles[2];
        assert(solve(feet[leg], angles));
        for (int joint = 0; joint < 2; ++joint) {
            const int ch = robot::SERVO_PINS[leg][joint];
            commands[ch] = command(leg, joint, angles[joint]);
            frame.channels[ch] = &commands[ch];
        }
    }
    char buffer[frameCapacity];
    formatFrame(buffer, frame);
    std::puts(buffer);
}

static void model() {
    std::printf("{\"version\":%u,\"channelCount\":%d,\"thigh\":%g,\"shin\":%g,\"stand\":[%g,%g],"
                "\"spacing\":%g,\"thighZero\":%g,\"limit\":[%g,%g],\"pins\":[",
                robot::MODEL_VERSION, robot::CHANNEL_COUNT, robot::LENA, robot::LENB, robot::STAND_X, robot::STAND_Y,
                robot::LEG_SPACING_DEG, robot::THIGH_ZERO_DEG, robot::SERVO_MIN_DEG, robot::SERVO_MAX_DEG);
    for (int i = 0; i < 6; ++i) std::printf("%s[%d,%d]", i ? "," : "", robot::SERVO_PINS[i][0], robot::SERVO_PINS[i][1]);
    std::printf("],\"offsets\":[");
    for (int i = 0; i < 6; ++i) std::printf("%s[%g,%g]", i ? "," : "", robot::ANGLE_OFFSET[i][0], robot::ANGLE_OFFSET[i][1]);
    std::puts("]}");
}

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "model") == 0) { model(); return 0; }
    const float expected[6][2] = {{83.38f,162.62f}, {67.38f,140.62f}, {67.38f,162.62f},
                                   {83.38f,162.62f}, {67.38f,157.62f}, {83.38f,162.62f}};
    float angles[2];
    assert(solve(stand[0], angles));
    for (int leg = 0; leg < 6; ++leg) {
        for (int j = 0; j < 2; ++j) assert(std::abs(command(leg, j, angles[j]).applied - expected[leg][j]) < 0.01);
        const FootPos point = footFromCommands(leg, command(leg, 0, angles[0]).applied, command(leg, 1, angles[1]).applied);
        assert(std::hypot(point.x - stand[leg].x, point.y - stand[leg].y) < 0.001);
    }
    assert(solve({-6, -7}, angles));
    assert(std::abs(angles[1] - 134.30) < 0.01);
    assert(command(3, 1, angles[1]).limited && command(3, 1, angles[1]).applied == 180);
    double peak = 0;
    assert(!feasible(3, {stand[3], stand[3], 3, 1, true, true}, 0, peak));
    assert(!solve({0, 0}, angles) && !solve({-20, -10}, angles));

    if (argc > 1) {
        Settings settings;
        settings.gait = std::strcmp(argv[1], "legacy") == 0 ? Gait::Legacy : Gait::Three;
        Walk walk;
        walk.start(stand, settings, settings.gait == Gait::Legacy ? 1 : 2);
        unsigned index = 0;
        for (unsigned now = 0; walk.active(); now += framePeriodMs) {
            const auto oldStage = walk.stage();
            const unsigned oldBeat = walk.completed();
            walk.tick(now);
            if (now % 100 == 0 || oldStage != walk.stage() || oldBeat != walk.completed() ||
                (settings.gait == Gait::Legacy && now == 420))
                printFrame(walk, index++, now, settings.mirror);
        }
        assert(walk.stage() == Stage::Idle);
        return 0;
    }

    for (const auto gait : {Gait::Five, Gait::Four, Gait::Three}) {
        const int beats = groups(gait);
        float maxSpeed = 0;
        int minimumSupport = 6;
        double minimumMargin = 100;
        for (int mirror : {-1, 1}) {
            for (int angle = 0; angle < 360; angle += 30) {
                Settings settings;
                settings.gait = gait;
                settings.mirror = mirror;
                settings.direction = angle;
                Walk walk;
                walk.start(stand, settings, beats + 1);
                float previous[6][2];
                for (auto& pair : previous) assert(solve(stand[0], pair));
                for (unsigned now = 0; walk.active() && now < 30000; now += framePeriodMs) {
                    walk.tick(now);
                    if (walk.stage() == Stage::Walk) {
                        int count = 0;
                        for (int i = 0; i < 6; ++i) if (walk.supportMask() & (1u << i)) ++count;
                        minimumSupport = std::min(minimumSupport, count);
                        minimumMargin = std::min(minimumMargin, supportMargin(walk, mirror));
                    }
                    for (int leg = 0; leg < 6; ++leg) {
                        assert(solve(walk.foot(leg), angles));
                        for (int j = 0; j < 2; ++j) {
                            assert(!command(leg, j, angles[j]).limited);
                            maxSpeed = std::max(maxSpeed, std::abs(angles[j] - previous[leg][j]) * 50);
                            previous[leg][j] = angles[j];
                        }
                    }
                }
                assert(walk.stage() == Stage::Idle);
                assert(walk.issued() == unsigned(beats + 1) && walk.completed() == walk.issued());
                assert(std::abs(walk.foot(3).x - stand[3].x) < 0.001);
            }
        }
        assert(minimumSupport == 6 - 6 / beats && maxSpeed <= 151 && minimumMargin > 0);
        std::printf("%s: minimum planned support %d, centered margin %.2f cm, peak sampled speed %.1f deg/s\n",
                    gaitName(gait), minimumSupport, minimumMargin, maxSpeed);
    }

    Walk transition;
    Settings settings;
    transition.start(stand, settings, 3);
    bool changed = false;
    for (unsigned now = 0; transition.active() && now < 30000; now += framePeriodMs) {
        FootPos last[6];
        for (int i = 0; i < 6; ++i) last[i] = transition.foot(i);
        transition.tick(now);
        if (!changed && transition.issued() == 1 && transition.supportMask() != 63) {
            settings.direction = 90;
            settings.gait = Gait::Four;
            transition.configure(settings);
            changed = true;
        }
        for (int i = 0; i < 6; ++i) {
            const auto foot = transition.foot(i);
            assert(std::hypot(foot.x - last[i].x, foot.y - last[i].y) < 0.5);
        }
    }
    assert(changed && transition.stage() == Stage::Idle && transition.completed() == 3);
    transition.start(stand, Settings{}, 0);
    for (unsigned now = 0; transition.active() && now < 15000; now += framePeriodMs) {
        transition.tick(now);
        if (transition.issued() == 1) transition.stop();
    }
    assert(transition.stage() == Stage::Idle && transition.completed() == 1);
    std::puts("Stand, legacy saturation, finite beats, direction/gait transition and stop passed.");
}
