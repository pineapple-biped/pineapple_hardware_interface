#include "../pineapple_hardware/motor_config.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

// The hardware interface runs its lowstate loop every 2 ms, so a reply must
// arrive within one period.
constexpr double kMaxRttMs = 2.0;
constexpr auto kCyclePeriod = std::chrono::microseconds(2000);
constexpr auto kReplyTimeout = std::chrono::milliseconds(10);
// After a lost reply, wait so a late one is not counted for the next command.
constexpr auto kLossSettle = std::chrono::milliseconds(5);
constexpr int kMeasureSeconds = 10;

std::atomic<bool> running(true);

double toMs(Clock::duration d) { return std::chrono::duration<double, std::milli>(d).count(); }

struct Stats {
    std::vector<double> samples;
    size_t lost = 0;

    double min() const { return samples.empty() ? NAN : *std::min_element(samples.begin(), samples.end()); }
    double max() const { return samples.empty() ? NAN : *std::max_element(samples.begin(), samples.end()); }
    double mean() const
    {
        return samples.empty() ? NAN
                               : std::accumulate(samples.begin(), samples.end(), 0.0) / samples.size();
    }
    // Nearest-rank percentile, p in (0, 1].
    double percentile(double p) const
    {
        if (samples.empty()) return NAN;
        std::vector<double> sorted = samples;
        std::sort(sorted.begin(), sorted.end());
        size_t rank = static_cast<size_t>(std::ceil(p * sorted.size()));
        return sorted[std::max<size_t>(rank, 1) - 1];
    }
};

const char* motorTypeName(int type)
{
    for (const auto& entry : kMotorTypeByName)
        if (entry.second == type) return entry.first.c_str();
    return "?";
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2 || argc > 3 || std::string(argv[1]) != "--measure") {
        std::cout << "Usage: motor_probe --measure [config.yaml]\n"
                  << "Enables the motors in the config (default ../config/config.yaml), sends\n"
                  << "zero-torque MIT commands for 10 seconds and measures each motor's reply time.\n"
                  << "The joints go limp: hang the robot up first.\n"
                  << "Stop the hardware interface first so only one process owns the USB2CANFD.\n";
        return argc == 2 && std::string(argv[1]) == "--help" ? 0 : 2;
    }
    const std::string config_path = argc == 3 ? argv[2] : "../config/config.yaml";
    MotorConfig cfg;
    try {
        if (!LoadMotorConfig(config_path, cfg)) return 2;
    } catch (const std::exception& e) {
        std::cerr << "Config error in " << config_path << ": " << e.what() << '\n';
        return 2;
    }
    std::cout << "[motor] config=" << config_path << " dev_sn=" << cfg.dev_sn
              << " motors=" << cfg.can_id_list.size() << '\n';

    std::vector<damiao::DmActData> dm_data;
    for (size_t i = 0; i < cfg.can_id_list.size(); i++) {
        dm_data.push_back(damiao::DmActData{.motorType = static_cast<damiao::DM_Motor_Type>(cfg.motor_type[i]),
                                            .mode = damiao::MIT_MODE,
                                            .can_id = cfg.can_id_list[i],
                                            .mst_id = cfg.mst_id_list[i]});
    }
    std::signal(SIGINT, [](int) { running = false; });

    std::unique_ptr<damiao::Motor_Control> control;
    try {
        control = std::make_unique<damiao::Motor_Control>(1000000, 5000000, cfg.dev_sn, &dm_data);
    } catch (const std::exception& e) {
        std::cerr << "USB2CANFD setup failed: " << e.what() << '\n';
        return 1;
    }
    std::vector<std::shared_ptr<damiao::Motor>> motors;
    for (auto can_id : cfg.can_id_list) motors.push_back(control->getMotor(can_id));
    // Let the replies to the enable and mode commands drain before timing.
    std::this_thread::sleep_for(std::chrono::seconds(1));

    const size_t n = motors.size();
    std::vector<Stats> rtt(n);
    Stats cycle;
    auto next = Clock::now();
    for (int second = 1; second <= kMeasureSeconds && running; ++second) {
        Stats window_rtt, window_cycle;
        const auto window_end = Clock::now() + std::chrono::seconds(1);
        while (running && Clock::now() < window_end) {
            const auto cycle_start = Clock::now();
            bool cycle_complete = true;
            for (size_t i = 0; i < n && running; ++i) {
                const uint64_t sequence = motors[i]->GetFeedback().sequence;
                const auto sent = Clock::now();
                control->control_mit(*motors[i], 0, 0, 0, 0, 0);
                double reply_ms = NAN;
                while (Clock::now() - sent < kReplyTimeout) {
                    if (motors[i]->GetFeedback().sequence != sequence) {
                        reply_ms = toMs(Clock::now() - sent);
                        break;
                    }
                    std::this_thread::yield();
                }
                if (std::isnan(reply_ms)) {
                    ++rtt[i].lost;
                    ++window_rtt.lost;
                    cycle_complete = false;
                    std::this_thread::sleep_for(kLossSettle);
                } else {
                    rtt[i].samples.push_back(reply_ms);
                    window_rtt.samples.push_back(reply_ms);
                }
            }
            if (cycle_complete && running) {
                const double cycle_ms = toMs(Clock::now() - cycle_start);
                cycle.samples.push_back(cycle_ms);
                window_cycle.samples.push_back(cycle_ms);
            }
            // Do not burst to catch up after a slow cycle.
            next = std::max(next + kCyclePeriod, Clock::now());
            std::this_thread::sleep_until(next);
        }
        std::printf("[motor] t=%ds rtt_ms mean=%.3f max=%.3f cycle_ms max=%.3f lost=%zu\n", second,
                    window_rtt.mean(), window_rtt.max(), window_cycle.max(), window_rtt.lost);
    }
    const bool interrupted = !running;
    control.reset(); // disables the motors

    bool healthy = !interrupted;
    std::printf("\n%-3s %-6s %-10s %6s %5s %8s %8s %8s %8s %8s %3s %5s\n", "idx", "can_id", "type", "sent",
                "lost", "min_ms", "mean_ms", "p50_ms", "p99_ms", "max_ms", "err", "t_mos");
    for (size_t i = 0; i < n; ++i) {
        const Stats& s = rtt[i];
        const bool ok = s.lost == 0 && !s.samples.empty() && s.max() < kMaxRttMs;
        healthy = healthy && ok;
        const auto feedback = motors[i]->GetFeedback();
        std::printf("%-3zu 0x%-4x %-10s %6zu %5zu %8.3f %8.3f %8.3f %8.3f %8.3f %3u %5.0f%s\n", i,
                    cfg.can_id_list[i], motorTypeName(cfg.motor_type[i]), s.samples.size() + s.lost, s.lost,
                    s.min(), s.mean(), s.percentile(0.5), s.percentile(0.99), s.max(),
                    unsigned(feedback.error), feedback.t_mos, ok ? "" : "  FAIL");
    }
    std::printf("bus cycle (%zu motors) mean_ms=%.3f p99_ms=%.3f max_ms=%.3f\n", n, cycle.mean(),
                cycle.percentile(0.99), cycle.max());
    if (interrupted) std::printf("interrupted\n");
    std::printf("%s (limit: no lost replies, max rtt < %.1f ms)\n", healthy ? "PASS" : "FAIL", kMaxRttMs);
    return healthy ? 0 : 1;
}
