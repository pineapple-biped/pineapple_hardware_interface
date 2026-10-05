#pragma once
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <iostream>

// 100 Hz full samples fit the documented 115200 baud link. Do not request
// 1000 Hz gyro/acceleration on that link or silently trust an accepted request.
inline bool configureXsens100Hz(XsDevice& device)
{
    if (!device.gotoConfig()) return false;
    device.readEmtsAndDeviceConfiguration();
    if (!(device.deviceId().isVru() || device.deviceId().isAhrs())) {
        std::cerr << "[imu] This device does not supply fused orientation.\n";
        return false;
    }
    XsOutputConfigurationArray config;
    config.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
    config.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
    for (auto id : {XDI_Quaternion, XDI_RateOfTurnHR, XDI_AccelerationHR})
        config.push_back(XsOutputConfiguration(id, 100));
    if (!device.setOutputConfiguration(config)) return false;
    const auto readback = device.outputConfiguration();
    for (auto id : {XDI_Quaternion, XDI_RateOfTurnHR, XDI_AccelerationHR}) {
        int rate = 0;
        for (const auto& item : readback)
            if ((item.m_dataIdentifier & XDI_FullTypeMask) == id)
                rate = item.m_frequency;
        std::cout << "[imu] output_id=" << static_cast<unsigned>(id)
                  << " configured_hz=" << rate << '\n';
        if (rate != 100) {
            std::cerr << "[imu] Refusing unexpected configured IMU rate.\n";
            return false;
        }
    }
    // Readback checks configuration, not actual delivery. The runtime monitor
    // separately reports received rates, timing gaps and device timestamps.
    return device.gotoMeasurement();
}
