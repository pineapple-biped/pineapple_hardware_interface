#pragma once
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <iostream>
#include "output_profile.hpp"

// Same explicit profile is used by the probe and full hardware interface.
inline bool configureXsens(XsDevice& device, const ImuOutputProfile& profile, XsBaudRate rate)
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
        const auto id = ids[i];
        int rate = 0;
        for (const auto& item : readback)
            if ((item.m_dataIdentifier & XDI_FullTypeMask) == id)
                rate = item.m_frequency;
        std::cout << "[imu] output_id=" << static_cast<unsigned>(id)
                  << " configured_hz=" << rate << '\n';
        if (rate != profile.hz[i]) {
            std::cerr << "[imu] Refusing unexpected configured IMU rate.\n";
            return false;
        }
    }
    // Readback checks configuration, not actual delivery. The runtime monitor
    // separately reports received rates, timing gaps and device timestamps.
    return device.gotoMeasurement();
}
