// Standalone IMU configuration/measurement: no CAN, DDS or motor dependencies.
#include "../imu/configure_xsens.hpp"
#include "../imu/xsens_imu.hpp"
#include <thread>
#include <csignal>
#include <memory>
#include "../imu/record_imu.hpp"
#include <string>
Journaller* gJournal = nullptr;

volatile std::sig_atomic_t stop_requested = 0;
void stopProbe(int) { stop_requested = 1; }

int main(int argc, char** argv)
{
    int seconds = 10;
    std::string output;
    bool measure = false;
    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--measure") measure = true;
            else if (arg == "--seconds" && i+1 < argc) {
                std::string value = argv[++i]; size_t used = 0;
                seconds = std::stoi(value, &used);
                if (used != value.size() || seconds < 1 || seconds > 120)
                    throw std::invalid_argument("seconds must be 1..120");
            } else if (arg == "--record" && i+1 < argc) output = argv[++i];
            else if (arg == "--help") {
                std::cout << "Usage: imu_probe --measure [--seconds 1..120] [--record NEW.csv]\n"
                          << "PINEAPPLE_IMU_PROFILE=baseline or fast. IMU only; no CAN, DDS or motors.\n"
                          << "Stop the hardware interface first. Recording begins after warmup.\n";
                return 0;
            } else throw std::invalid_argument("Unknown/missing argument: " + arg);
        }
        if (!measure) throw std::invalid_argument("Specify --measure; use --help for options");
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
    // Exclusive create before opening any device. Existing recordings are never overwritten.
    std::unique_ptr<FILE, decltype(&std::fclose)> recording(nullptr, &std::fclose);
    if (!output.empty()) {
        recording.reset(std::fopen(output.c_str(), "wx"));
        if (!recording) { std::perror("Cannot create new recording"); return 2; }
    }
    std::signal(SIGINT, stopProbe);
    std::signal(SIGTERM, stopProbe);
    ImuRecorder recorder(recording ? static_cast<size_t>(seconds + 2) * 4000 : 0);
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
    if (recording) device->addCallbackHandler(&recorder);
    int result = 1;
    if (configureXsens(*device, profile, port.baudrate())) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        shared->resetTimingStats();
        auto before = shared->load();
        auto start = before;
        double last = imuMonotonicSeconds(), begin = last;
        bool healthy = true;
        if (recording) recorder.start();
        std::cout << "[imu] BEGIN measurement/recording: " << seconds << " seconds\n" << std::flush;
        for (int i = 0; i < seconds && !stop_requested; ++i) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            const auto after = shared->load();
            const double now = imuMonotonicSeconds();
            std::cout << "[imu] elapsed_s=" << now - begin << ' ' ;
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
        recorder.stop();
        healthy &= !stop_requested && recorder.dropped() == 0;
        // A burst stream can have correct average rate; reject large callback gaps too.
        healthy &= before.max_callback_gap_ms < 30;
        healthy &= before.missing_packets == start.missing_packets;
        healthy &= before.invalid_quaternions == start.invalid_quaternions;
        std::cout << (healthy ? "PASS" : "FAIL")
                  << ": host delivery check over " << last - begin << " s; "
                  << "does not measure absolute sensor/filter latency.\n";
        result = healthy ? 0 : 1;
    }
    recorder.stop();
    control->closePort(port.portName().toStdString());
    if (recording) {
        const bool written = recorder.write(recording.get());
        const bool flushed = std::fflush(recording.get()) == 0;
        if (!written || !flushed) { std::cerr << "Recording write failed\n"; result = 1; }
        std::cout << "[imu] recording=" << output << " rows=" << recorder.size()
                  << " capture_dropped=" << recorder.dropped()
                  << " interrupted=" << stop_requested << '\n';
    }
    control->destruct();
    return result;
}
