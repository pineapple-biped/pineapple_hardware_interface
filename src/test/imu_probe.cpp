// Standalone IMU configuration/measurement: no CAN, DDS or motor dependencies.
#include "../imu/configure_xsens.hpp"
#include "../imu/xsens_imu.hpp"
#include <thread>
#include <string>
Journaller* gJournal = nullptr;

int main(int argc, char** argv)
{
    if (argc != 2 || std::string(argv[1]) != "--measure") {
        std::cout << "Usage: imu_probe --measure\n"
                  << "PINEAPPLE_IMU_PROFILE=baseline (100/100/100 Hz) or fast (100/1000/1000 Hz).\n"
                  << "Measures for 10 seconds; no motors.\n"
                  << "Stop the hardware interface first so only one process owns the IMU.\n";
        return argc == 2 && std::string(argv[1]) == "--help" ? 0 : 2;
    }
    ImuOutputProfile profile;
    try { profile = imuOutputProfileFromEnvironment(); }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
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
              << " baud=" << port.baudrate() << '\n';
    XsDevice* device = control->device(port.deviceId());
    auto shared = std::make_shared<ImuSharedData>();
    CallbackHandler callback;
    callback.setTarget(shared);
    device->addCallbackHandler(&callback);
    int result = 1;
    if (configureXsens(*device, profile, static_cast<int>(port.baudrate()))) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        shared->resetTimingStats();
        auto before = shared->load();
        auto start = before;
        double last = imuMonotonicSeconds(), begin = last;
        bool healthy = true;
        for (int i = 0; i < 10; ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            const auto after = shared->load();
            const double now = imuMonotonicSeconds();
            reportImu(before, after, now - last, now);
            for (size_t j = 0; j < 3; ++j) {
                const double hz = (after.updates[j] - before.updates[j]) / (now - last);
                healthy &= after.received_at[j] > 0 && imuDeliveryWindowHealthy(
                    profile, j, hz, 1000 * (now - after.received_at[j]),
                    after.max_field_gap_ms[j]);
            }
            before = after;
            last = now;
        }
        // A burst stream can have correct average rate; reject large callback gaps too.
        healthy &= before.max_callback_gap_ms < 30;
        healthy &= before.missing_packets == start.missing_packets;
        healthy &= before.invalid_quaternions == start.invalid_quaternions;
        std::cout << (healthy ? "PASS" : "FAIL")
                  << ": host delivery check over " << last - begin << " s; "
                  << "does not measure absolute sensor/filter latency.\n";
        result = healthy ? 0 : 1;
    }
    control->closePort(port.portName().toStdString());
    control->destruct();
    return result;
}
