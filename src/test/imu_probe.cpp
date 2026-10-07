// Standalone IMU configuration/measurement: no CAN, DDS or motor dependencies.
#include "../imu/configure_xsens.hpp"
#include "../imu/xsens_imu.hpp"
#include <thread>
#include <csignal>
#include <memory>
#include <sys/resource.h>
#include <xscontroller/pineapple_poll_wait.h>
#include <xscontroller/pineapple_readiness.h>
#include "../imu/record_imu.hpp"
#include <string>
Journaller* gJournal = nullptr;

volatile std::sig_atomic_t stop_requested = 0;
void stopProbe(int) { stop_requested = 1; }

int main(int argc, char** argv)
{
    int seconds = 10;
    std::string output, trace_output;
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
            else if (arg == "--trace" && i+1 < argc) trace_output = argv[++i];
            else if (arg == "--help") {
                std::cout << "Usage: imu_probe --measure [--seconds 1..120] [--record NEW.csv] [--trace NEW.trace.csv]\n"
                          << "PINEAPPLE_IMU_PROFILE=baseline, fast, fast500 or fast250. IMU only; no CAN, DDS or motors.\n"
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
    std::unique_ptr<FILE, decltype(&std::fclose)> trace_file(nullptr, &std::fclose);
    if (!trace_output.empty()) {
        if (output.empty()) { std::cerr << "--trace requires --record\n"; return 2; }
        trace_file.reset(std::fopen(trace_output.c_str(), "wx"));
        if (!trace_file) { std::perror("Cannot create new trace"); return 2; }
        pineappleDeliveryTrace().prepare(static_cast<size_t>(seconds + 2) * 24000);
    }
    std::signal(SIGINT, stopProbe);
    std::signal(SIGTERM, stopProbe);
    ImuRecorder recorder(recording ? static_cast<size_t>(seconds + 2) * 4000 : 0);
    ImuOutputProfile profile;
    try { profile = imuOutputProfileFromEnvironment(); }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 2; }
    std::cout << "[imu] SDK small-read wait=" << (pineappleLowLatencyPoll() ? 1 : 2)
              << " ms (rebuild Xsens libraries for the opt-in override)\n";
    std::cout << "[imu] readiness mode=" << (pineappleReadinessEnabled() ? "requested" : "off")
              << " (POSIX serial: wait up to 10 ms, wake on bytes; rebuild SDK)\n";
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
        rusage usage_before{}; getrusage(RUSAGE_SELF, &usage_before);
        bool healthy = true;
        if (recording) recorder.start();
        if (trace_file) pineappleDeliveryTrace().enabled.store(true);
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
            rusage usage_after{}; getrusage(RUSAGE_SELF, &usage_after);
            const auto cpu = [](const rusage& u) {
                return u.ru_utime.tv_sec + u.ru_stime.tv_sec
                    + 1e-6 * (u.ru_utime.tv_usec + u.ru_stime.tv_usec);
            };
            std::cout << "[host] elapsed_s=" << now - begin
                      << " process_cpu_pct=" << 100 * (cpu(usage_after) - cpu(usage_before)) / (now-last)
                      << " minor_faults=" << usage_after.ru_minflt - usage_before.ru_minflt
                      << " major_faults=" << usage_after.ru_majflt - usage_before.ru_majflt
                      << " voluntary_switches=" << usage_after.ru_nvcsw - usage_before.ru_nvcsw
                      << " involuntary_switches=" << usage_after.ru_nivcsw - usage_before.ru_nivcsw << '\n';
            usage_before = usage_after;
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
    pineappleDeliveryTrace().enabled.store(false);
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
    if (trace_file) {
        const bool written = pineappleDeliveryTrace().write(trace_file.get());
        const bool flushed = std::fflush(trace_file.get()) == 0;
        const auto dropped = pineappleDeliveryTrace().dropped();
        std::cout << "[trace] events=" << pineappleDeliveryTrace().next.load()
                  << " dropped=" << dropped << " file=" << trace_output << '\n';
        if (!written || !flushed || dropped) result = 1;
    }
    return result;
}
