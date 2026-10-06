#pragma once
#include <array>
#include <cstdlib>
#include <stdexcept>
#include <string>

struct ImuOutputProfile {
    std::array<int, 3> hz;
    std::array<double, 3> max_gap_ms;
    int minimum_baud;
};

inline ImuOutputProfile imuOutputProfile(const std::string& name)
{
    if (name == "baseline") return {{100, 100, 100}, {20, 20, 20}, 115200};
    // High-rate mode is only enabled on the previously verified 2 Mbaud link.
    if (name == "fast") return {{100, 1000, 1000}, {20, 5, 5}, 2000000};
    throw std::invalid_argument("PINEAPPLE_IMU_PROFILE must be baseline or fast");
}

inline ImuOutputProfile imuOutputProfileFromEnvironment()
{
    const char* value = std::getenv("PINEAPPLE_IMU_PROFILE");
    return imuOutputProfile(value ? value : "baseline");
}

inline bool imuDeliveryWindowHealthy(const ImuOutputProfile& profile,
                                     size_t field, double hz, double age_ms,
                                     double max_gap_ms)
{
    return hz >= .9 * profile.hz.at(field) && hz <= 1.1 * profile.hz.at(field)
        && age_ms >= 0 && age_ms < profile.max_gap_ms.at(field)
        && max_gap_ms < profile.max_gap_ms.at(field);
}
