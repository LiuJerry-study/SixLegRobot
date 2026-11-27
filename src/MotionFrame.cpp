#include "MotionFrame.h"
#include <cstdio>

namespace motion {
void formatFrame(char (&buffer)[frameCapacity], const Frame& frame) {
    int n = std::snprintf(buffer, sizeof(buffer), "@FRAME %u %lu %lu %s %u %u %u %d",
                         robot::MODEL_VERSION, (unsigned long)frame.sequence, (unsigned long)frame.ms,
                         stageName(frame.stage), frame.issued, frame.completed, frame.support, frame.mirror);
    for (const auto* joint : frame.channels) {
        if (joint) n += std::snprintf(buffer + n, sizeof(buffer) - n, " %.1f,%.1f,%d",
                                     joint->logical, joint->applied, joint->limited);
        else n += std::snprintf(buffer + n, sizeof(buffer) - n, " -");
    }
    for (const auto* foot : frame.requested) {
        if (foot) n += std::snprintf(buffer + n, sizeof(buffer) - n, " %.2f,%.2f", foot->x, foot->y);
        else n += std::snprintf(buffer + n, sizeof(buffer) - n, " -");
    }
}
}
