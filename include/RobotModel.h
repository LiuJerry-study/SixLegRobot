#pragma once

namespace robot {
constexpr unsigned MODEL_VERSION = 1;
constexpr int LEG_COUNT = 6;
constexpr int CHANNEL_COUNT = 16;
constexpr double LENA = 6.5;
constexpr double LENB = 12.5;
constexpr double STAND_X = -6.0;
constexpr double STAND_Y = -10.0;
constexpr double LEG_SPACING_DEG = 60.0;
constexpr double THIGH_ZERO_DEG = 90.0;
constexpr float SERVO_MIN_DEG = 0.0f;
constexpr float SERVO_MAX_DEG = 180.0f;

// Legs 0–5, femur then tibia. Amend the version when changing the model.
constexpr int SERVO_PINS[LEG_COUNT][2] = {
    {0, 1}, {2, 3}, {4, 5}, {6, 7}, {8, 9}, {10, 11}
};
constexpr float ANGLE_OFFSET[LEG_COUNT][2] = {
    {16, 50}, {0, 28}, {0, 50}, {16, 50}, {0, 45}, {16, 50}
};
}
