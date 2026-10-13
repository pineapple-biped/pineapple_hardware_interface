#include "../imu/configure_xsens.hpp"
#include <string>
Journaller* gJournal = nullptr;

int main(int argc, char** argv)
{
    if (argc != 2 || std::string(argv[1]) != "--measure") {
        std::cout << "Usage: imu_probe --measure\n"
                  << "Configures the IMU with PINEAPPLE_IMU_PROFILE (default baseline) and measures\n"
                  << "delivery for 10 seconds; no motors.\n"
                  << "Stop the hardware interface first so only one process owns the IMU.\n";
        return argc == 2 && std::string(argv[1]) == "--help" ? 0 : 2;
    }
    ImuOutputProfile profile;
    try {
        profile = imuOutputProfileFromEnvironment();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 2;
    }
    XsControl* control = XsControl::construct();
    if (!control) return 1;
    XsPortInfo port;
    for (const auto& candidate : XsScanner::scanPorts()) {
        if (candidate.deviceId().isMti() || candidate.deviceId().isMtig()) {
            port = candidate;
            break;
        }
    }
    if (port.empty() || !control->openPort(port.portName().toStdString(), port.baudrate())) {
        std::cerr << "No usable MTi port.\n";
        control->destruct();
        return 1;
    }
    std::cout << "[imu] port=" << port.portName().toStdString()
              << " baud=" << XsBaud::rateToNumeric(port.baudrate()) << '\n';
    XsDevice* device = control->device(port.deviceId());
    auto shared = std::make_shared<ImuSharedData>();
    CallbackHandler callback;
    callback.setTarget(shared);
    device->addCallbackHandler(&callback);
    int result = 1;
    if (configureXsens(*device, profile, port.baudrate())) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        bool healthy = true;
        for (int i = 0; i < 10; ++i)
            healthy = verifyImuDelivery(*shared, profile, 1.0) && healthy;
        std::cout << (healthy ? "PASS" : "FAIL") << '\n';
        result = healthy ? 0 : 1;
    }
    control->closePort(port.portName().toStdString());
    control->destruct();
    return result;
}
