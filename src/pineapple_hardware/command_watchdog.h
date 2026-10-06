#ifndef PINEAPPLE_COMMAND_WATCHDOG_H
#define PINEAPPLE_COMMAND_WATCHDOG_H

#include <chrono>
#include <cmath>

// Caller serializes this state machine and all motor writes with the same mutex.
class CommandWatchdog {
public:
    using Clock = std::chrono::steady_clock;
    using Time = Clock::time_point;
    static constexpr auto timeout = std::chrono::milliseconds(100);

    bool Accept(Time now) {
        Check(now);  // A late packet must not rescue an expired session.
        if (tripped_) return false;
        armed_ = true;
        last_command_ = now;
        return true;
    }
    bool Check(Time now) {
        if (armed_ && now - last_command_ >= timeout) Trip();
        return tripped_;
    }
    void Trip() { tripped_ = true; }
    bool Tripped() const { return tripped_; }

    // Limits of the MIT encoder, not a replacement for per-joint safety limits.
    static bool Valid(double q, double dq, double tau, double kp, double kd) {
        return std::isfinite(q) && std::isfinite(dq) && std::isfinite(tau) &&
               std::isfinite(kp) && std::isfinite(kd) &&
               kp >= 0 && kp <= 500 && kd >= 0 && kd <= 5;
    }
private:
    bool armed_ = false;
    bool tripped_ = false;
    Time last_command_{};
};
#endif
