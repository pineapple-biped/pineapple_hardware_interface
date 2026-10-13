#pragma once
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "output_profile.hpp"
#include "xsens_imu.hpp"

inline bool configureXsens(XsDevice& device, const ImuOutputProfile& profile, XsBaudRate rate,
                           bool verbose = true)
{
    const int baud = XsBaud::rateToNumeric(rate);
    if (!imuProfileSupportsBaud(profile, rate)) {
        std::cerr << "[imu] Selected profile requires baud >= " << profile.minimum_baud
                  << "; detected " << baud << ". No baud change was made.\n";
        return false;
    }
    if (!device.gotoConfig()) return false;
    device.readEmtsAndDeviceConfiguration();
    if (!(device.deviceId().isVru() || device.deviceId().isAhrs())) {
        std::cerr << "[imu] This device does not supply fused orientation.\n";
        return false;
    }
    XsOutputConfigurationArray config;
    config.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
    config.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
    const std::array<XsDataIdentifier, 3> ids = {
        XDI_Quaternion, XDI_RateOfTurnHR, XDI_AccelerationHR};
    for (size_t i = 0; i < ids.size(); ++i)
        config.push_back(XsOutputConfiguration(ids[i], profile.hz[i]));
    if (!device.setOutputConfiguration(config)) return false;
    const auto readback = device.outputConfiguration();
    for (size_t i = 0; i < ids.size(); ++i) {
        int configured = 0;
        for (const auto& item : readback)
            if ((item.m_dataIdentifier & XDI_FullTypeMask) == ids[i])
                configured = item.m_frequency;
        if (verbose) std::cout << "[imu] output_id=" << static_cast<unsigned>(ids[i])
                  << " configured_hz=" << configured << '\n';
        if (configured != profile.hz[i]) {
            std::cerr << "[imu] Refusing unexpected configured IMU rate.\n";
            return false;
        }
    }
    return device.gotoMeasurement();
}

inline bool verifyImuDelivery(ImuSharedData& data, const ImuOutputProfile& profile, double seconds)
{
    data.resetTimingStats();
    const auto before = data.load();
    const double start = imuMonotonicSeconds();
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
    const auto after = data.load();
    const double now = imuMonotonicSeconds();
    reportImu(before, after, now - start, now);
    bool healthy = after.invalid_quaternions == before.invalid_quaternions;
    for (size_t i = 0; i < 3; ++i) {
        const double hz = (after.updates[i] - before.updates[i]) / (now - start);
        const double age_ms = after.received_at[i] > 0 ? 1000 * (now - after.received_at[i]) : -1;
        healthy = healthy && imuDeliveryWindowHealthy(profile, i, hz, age_ms, after.max_field_gap_ms[i]);
    }
    return healthy;
}
