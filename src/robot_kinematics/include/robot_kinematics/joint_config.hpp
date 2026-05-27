#pragma once
#include <array>
#include <cstdint>

namespace kvoy {

// Joint order convention (matches MCU and training):
// 0=FL_hip  1=FL_thigh  2=FL_calf
// 3=FR_hip  4=FR_thigh  5=FR_calf
// 6=RL_hip  7=RL_thigh  8=RL_calf
// 9=RR_hip 10=RR_thigh 11=RR_calf

static constexpr int NUM_JOINTS = 12;

// Default standing pose [rad] — tune to your robot's geometry
static constexpr std::array<float, NUM_JOINTS> STAND_POS = {
     0.0f,  0.8f, -1.5f,   // FL
     0.0f,  0.8f, -1.5f,   // FR
     0.0f,  1.0f, -1.5f,   // RL
     0.0f,  1.0f, -1.5f,   // RR
};

// Lie-down (folded) pose [rad]
static constexpr std::array<float, NUM_JOINTS> LIE_POS = {
     0.0f,  1.4f, -2.6f,   // FL
     0.0f,  1.4f, -2.6f,   // FR
     0.0f,  1.4f, -2.6f,   // RL
     0.0f,  1.4f, -2.6f,   // RR
};

// Interpolate between two joint position arrays.
// alpha=0 → from, alpha=1 → to
inline std::array<float, NUM_JOINTS> lerp_joints(
    const std::array<float, NUM_JOINTS>& from,
    const std::array<float, NUM_JOINTS>& to,
    float alpha)
{
    std::array<float, NUM_JOINTS> out;
    for (int i = 0; i < NUM_JOINTS; ++i)
        out[i] = from[i] + alpha * (to[i] - from[i]);
    return out;
}

} // namespace kvoy
