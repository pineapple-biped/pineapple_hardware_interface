#pragma once
#include <algorithm>
#include <array>
#include <xstypes/xsbaud.h>
#include <cstdlib>
#include <iostream>
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
    // gyro/accel gaps up to ~5 ms occur from host scheduling at 1 kHz (seen on V3); FTDI
    // latency-timer buffering, what this check is for, shows as 16-50 ms gaps.
    if (name == "fast") return {{100, 1000, 1000}, {20, 10, 10}, 2000000};
    if (name == "fast500") return {{100, 500, 500}, {20, 5, 5}, 2000000};
    if (name == "fast250") return {{100, 250, 250}, {20, 5, 5}, 2000000};
    throw std::invalid_argument(
        "PINEAPPLE_IMU_PROFILE must be baseline, fast, fast500 or fast250");
}

// PINEAPPLE_IMU_QUAT_HZ overrides the profile's quaternion rate (e.g. 200 or 400, if
// the device accepts it; configureXsens refuses a rate the device does not read back).
// Default when PINEAPPLE_IMU_PROFILE is unset: "fast" = quaternion 100 Hz, gyro and
// acceleration 1000 Hz, the rates the bridge used before f1031cc (2026-09-15). That commit set
// all three to 100 Hz ("baseline"): the gyro the policy balances on got up to 10 ms older and,
// with the low byte rate, the FTDI adapter's 16 ms latency timer started holding samples back
// (30-50 ms delivery bursts). Policies were shaky from then on.
constexpr const char* kDefaultImuProfile = "fast";

inline bool imuProfileRequested() { return std::getenv("PINEAPPLE_IMU_PROFILE") != nullptr; }

inline ImuOutputProfile imuOutputProfileFromEnvironment()
{
    const char* value = std::getenv("PINEAPPLE_IMU_PROFILE");
    ImuOutputProfile profile = imuOutputProfile(value ? value : kDefaultImuProfile);
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

// The default profile needs the 2 Mbaud link V3 uses. If the detected link is slower and no
// profile was requested explicitly, fall back to "baseline" with a loud warning instead of
// refusing to start; an explicitly requested profile is never changed.
inline ImuOutputProfile imuProfileForLink(const ImuOutputProfile& profile, XsBaudRate rate)
{
    if (imuProfileRequested() || imuProfileSupportsBaud(profile, rate)) return profile;
    ImuOutputProfile fallback = imuOutputProfile("baseline");
    fallback.hz[0] = profile.hz[0];
    fallback.max_gap_ms[0] = profile.max_gap_ms[0];
    std::cerr << "[imu] WARNING: link is " << XsBaud::rateToNumeric(rate) << " baud, the default "
              << kDefaultImuProfile << " profile needs " << profile.minimum_baud
              << ". Using baseline (gyro/accel 100 Hz): expect older angular velocity and FTDI "
                 "buffering unless the latency timer is 1 ms. Set the IMU to 2000000 baud.\n";
    return fallback;
}
