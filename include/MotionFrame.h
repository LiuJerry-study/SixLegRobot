#pragma once

#include <stddef.h>
#include "WalkMotion.h"

namespace motion {
struct Frame {
    uint32_t sequence, ms;
    Stage stage;
    unsigned issued, completed, support;
    int mirror;
    const JointCommand* channels[robot::CHANNEL_COUNT];
    const FootPos* requested[robot::LEG_COUNT];
};

constexpr size_t frameCapacity = 640;
void formatFrame(char (&buffer)[frameCapacity], const Frame& frame);
}
