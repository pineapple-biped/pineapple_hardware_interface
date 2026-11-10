#pragma once
#include <algorithm>
#include <array>
#include <xstypes/xsbaud.h>
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
    if (name == "fast") return {{100, 1000, 1000}, {20, 5, 5}, 2000000};
    if (name == "fast500") return {{100, 500, 500}, {20, 5, 5}, 2000000};
    if (name == "fast250") return {{100, 250, 250}, {20, 5, 5}, 2000000};
    throw std::invalid_argument(
        "PINEAPPLE_IMU_PROFILE must be baseline, fast, fast500 or fast250");
}

// PINEAPPLE_IMU_QUAT_HZ overrides the profile's quaternion rate (e.g. 200 or 400, if
// the device accepts it; configureXsens refuses a rate the device does not read back).
inline ImuOutputProfile imuOutputProfileFromEnvironment()
{
    const char* value = std::getenv("PINEAPPLE_IMU_PROFILE");
    ImuOutputProfile profile = imuOutputProfile(value ? value : "baseline");
    if (const char* quat_hz = std::getenv("PINEAPPLE_IMU_QUAT_HZ")) {
        const int hz = std::atoi(quat_hz);
        if (hz <= 0 || hz > 2000)
            throw std::invalid_argument("PINEAPPLE_IMU_QUAT_HZ must be 1..2000");
        profile.hz[0] = hz;
        profile.max_gap_ms[0] = std::max(5.0, 2000.0 / hz);  // two periods, as 100 Hz -> 20 ms
    }
    return profile;
}

// PINEAPPLE_IMU_PROPAGATE=1: publish the device quaternion rolled forward with each gyro
// sample that arrives after it. Only useful when the gyro rate exceeds the quaternion rate.
inline bool imuPropagateFromEnvironment()
{
    const char* value = std::getenv("PINEAPPLE_IMU_PROPAGATE");
    return value && std::string(value) == "1";
}

inline bool imuDeliveryWindowHealthy(const ImuOutputProfile& profile,
                                     size_t field, double hz, double age_ms,
                                     double max_gap_ms)
{
    return hz >= .9 * profile.hz.at(field) && hz <= 1.1 * profile.hz.at(field)
        && age_ms >= 0 && age_ms < profile.max_gap_ms.at(field)
        && max_gap_ms < profile.max_gap_ms.at(field);
}

inline bool imuProfileSupportsBaud(const ImuOutputProfile& profile, XsBaudRate rate)
{
    return XsBaud::rateToNumeric(rate) >= profile.minimum_baud;
}
