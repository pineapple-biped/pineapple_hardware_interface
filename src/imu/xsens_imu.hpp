#pragma once

#include <xscontroller/xscallback.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xsquaternion.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

// Rotate a sensor-to-global quaternion {w,x,y,z} by a body-frame angular rate (rad/s)
// held for dt seconds: q <- q * exp(w*dt/2). Exact for a constant rate.
inline std::array<double, 4> integrateQuaternion(const std::array<double, 4>& q,
                                                 const std::array<double, 3>& w, double dt)
{
    const double n = std::sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    const double half = 0.5 * n * dt;
    const double c = std::cos(half);
    const double k = n > 1e-12 ? std::sin(half) / n : 0.5 * dt;
    const std::array<double, 4> d = {c, k * w[0], k * w[1], k * w[2]};
    std::array<double, 4> r = {
        q[0] * d[0] - q[1] * d[1] - q[2] * d[2] - q[3] * d[3],
        q[0] * d[1] + q[1] * d[0] + q[2] * d[3] - q[3] * d[2],
        q[0] * d[2] - q[1] * d[3] + q[2] * d[0] + q[3] * d[1],
        q[0] * d[3] + q[1] * d[2] - q[2] * d[1] + q[3] * d[0]};
    double norm = 0;
    for (double v : r) norm += v * v;
    norm = std::sqrt(norm);
    for (double& v : r) v /= norm;
    return r;
}

// Rotation angle between two unit quaternions, degrees.
inline double quaternionAngleDeg(const std::array<double, 4>& a, const std::array<double, 4>& b)
{
    double dot = std::fabs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
    return 2 * std::acos(std::min(1.0, dot)) * 180.0 / M_PI;
}

inline double imuMonotonicSeconds()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct ImuSample {
    std::array<double, 4> quaternion = {1, 0, 0, 0};
    std::array<double, 3> rpy = {0, 0, 0};
    std::array<double, 3> gyro = {0, 0, 0};
    std::array<double, 3> accel = {0, 0, 0};
    std::array<double, 3> received_at = {0, 0, 0};
    std::array<uint64_t, 3> updates = {0, 0, 0};
    uint64_t packets = 0, missing_packets = 0, invalid_quaternions = 0;
    std::array<double, 3> max_field_gap_ms = {0, 0, 0};
    double max_callback_gap_ms = 0;
    double device_interval_ms = 0;
    // Latency instrumentation: device SampleTimeFine (10 kHz) of the current quaternion.
    uint32_t quat_sample_time_fine = 0;
    // Orientation to publish: the device quaternion, rolled forward with every gyro sample
    // that arrives after it when propagation is on (PINEAPPLE_IMU_PROPAGATE=1).
    std::array<double, 4> quaternion_out = {1, 0, 0, 0};
    double quaternion_out_at = 0;     // host time quaternion_out last changed
    uint64_t propagation_steps = 0;   // gyro steps applied since startup
    // Propagation check: angle between the propagated estimate and the next device
    // quaternion it is replaced by (same device time). Mean over propagation_checks.
    uint64_t propagation_checks = 0;
    double propagation_error_sum_deg = 0, propagation_error_max_deg = 0;
};

class ImuSharedData {
public:
    ImuSample load() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return sample_;
    }

    // Roll the published orientation forward with gyro samples between device quaternions.
    void setPropagate(bool on)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        propagate_ = on;
    }

    void resetTimingStats()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sample_.max_callback_gap_ms = 0;
        sample_.max_field_gap_ms = {0, 0, 0};
        sample_.propagation_error_max_deg = 0;
        last_callback_ = imuMonotonicSeconds();
    }

    void ingest(const XsDataPacket& packet, double now)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (sample_.packets)
            sample_.max_callback_gap_ms = std::max(
                sample_.max_callback_gap_ms, (now - last_callback_) * 1000);
        last_callback_ = now;
        ++sample_.packets;
        if (packet.containsPacketCounter()) {
            const auto counter = packet.packetCounter();
            const uint16_t delta = static_cast<uint16_t>(counter - previous_counter_);
            if (have_counter_ && delta > 1 && delta < 32768)
                sample_.missing_packets += delta - 1;
            previous_counter_ = counter;
            have_counter_ = true;
        }
        const bool has_time = packet.containsSampleTimeFine();
        const uint32_t ticks = has_time ? packet.sampleTimeFine() : 0;
        if (has_time) {
            const uint32_t delta = ticks - previous_ticks_;
            if (have_ticks_ && delta < 100000)
                sample_.device_interval_ms = delta * 0.1;
            previous_ticks_ = ticks;
            have_ticks_ = true;
        }
        // Gyro first: in a packet that also carries a quaternion, propagating to this
        // packet's time and then comparing against that quaternion measures the error.
        if (packet.containsRateOfTurnHR()) {
            const auto g = packet.rateOfTurnHR();
            for (size_t i = 0; i < 3; ++i) sample_.gyro[i] = g[i];
            markFieldReceived(1, now);
            propagate(has_time, ticks, now);
        }
        if (packet.containsOrientation()) {
            const auto q = packet.orientationQuaternion();
            std::array<double, 4> value = {q.w(), q.x(), q.y(), q.z()};
            double norm = 0;
            for (double v : value) norm += v * v;
            if (std::isfinite(norm) && norm > 0.5 && norm < 1.5) {
                for (size_t i = 0; i < 4; ++i)
                    sample_.quaternion[i] = value[i] / std::sqrt(norm);
                const auto e = packet.orientationEuler();
                sample_.rpy = {e.roll(), e.pitch(), e.yaw()};
                if (has_time)
                    sample_.quat_sample_time_fine = ticks;
                if (propagated_since_quat_ && has_time && ticks == out_ticks_) {
                    const double err = quaternionAngleDeg(sample_.quaternion_out, sample_.quaternion);
                    ++sample_.propagation_checks;
                    sample_.propagation_error_sum_deg += err;
                    sample_.propagation_error_max_deg = std::max(sample_.propagation_error_max_deg, err);
                }
                sample_.quaternion_out = sample_.quaternion;
                sample_.quaternion_out_at = now;
                have_out_ = has_time;
                out_ticks_ = ticks;
                quat_ticks_ = ticks;
                propagated_since_quat_ = false;
                markFieldReceived(0, now);
            } else {
                ++sample_.invalid_quaternions;
            }
        }
        if (packet.containsAccelerationHR()) {
            const auto a = packet.accelerationHR();
            for (size_t i = 0; i < 3; ++i) sample_.accel[i] = a[i];
            markFieldReceived(2, now);
        }
    }

private:
    void markFieldReceived(size_t field, double now)
    {
        if (sample_.received_at[field] > 0)
            sample_.max_field_gap_ms[field] = std::max(sample_.max_field_gap_ms[field],
                1000 * (now - sample_.received_at[field]));
        sample_.received_at[field] = now;
        ++sample_.updates[field];
    }

    // Advance quaternion_out from its device time to `ticks` with the current gyro.
    // Holds instead when the device quaternion is older than kMaxHorizonTicks (a stalled
    // filter output must not turn into open-loop gyro integration).
    void propagate(bool has_time, uint32_t ticks, double now)
    {
        static constexpr uint32_t kMaxHorizonTicks = 500;  // 50 ms at 10 kHz
        if (!propagate_ || !have_out_ || !has_time) return;
        const uint32_t step = ticks - out_ticks_;
        if (step == 0 || step > kMaxHorizonTicks || ticks - quat_ticks_ > kMaxHorizonTicks) return;
        sample_.quaternion_out = integrateQuaternion(sample_.quaternion_out, sample_.gyro, step * 1e-4);
        sample_.quaternion_out_at = now;
        out_ticks_ = ticks;
        propagated_since_quat_ = true;
        ++sample_.propagation_steps;
    }

    mutable std::mutex mutex_;
    ImuSample sample_;
    bool propagate_ = false, have_out_ = false, propagated_since_quat_ = false;
    uint32_t out_ticks_ = 0, quat_ticks_ = 0;
    double last_callback_ = 0;
    uint16_t previous_counter_ = 0;
    uint32_t previous_ticks_ = 0;
    bool have_counter_ = false, have_ticks_ = false;
};

class CallbackHandler : public XsCallback {
public:
    void setTarget(std::shared_ptr<ImuSharedData> target) { target_ = std::move(target); }
protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
    {
        if (packet && target_) target_->ingest(*packet, imuMonotonicSeconds());
    }
private:
    std::shared_ptr<ImuSharedData> target_;
};

inline void reportImu(const ImuSample& before, const ImuSample& after,
                      double elapsed, double now)
{
    std::cout << "[imu] q/gyro/accel_hz=";
    for (size_t i = 0; i < 3; ++i)
        std::cout << (after.updates[i] - before.updates[i]) / elapsed << (i == 2 ? "" : "/");
    std::cout << " host_age_ms=";
    for (size_t i = 0; i < 3; ++i)
        std::cout << (after.received_at[i] ? 1000 * (now - after.received_at[i]) : -1)
                  << (i == 2 ? "" : "/");
    std::cout << " max_field_gap_ms=";
    for (size_t i = 0; i < 3; ++i)
        std::cout << after.max_field_gap_ms[i] << (i == 2 ? "" : "/");
    std::cout << " max_callback_gap_ms=" << after.max_callback_gap_ms
              << " last_device_interval_ms=" << after.device_interval_ms
              << " missing_packets=" << after.missing_packets
              << " invalid_quaternions=" << after.invalid_quaternions;
    const uint64_t checks = after.propagation_checks - before.propagation_checks;
    if (after.propagation_steps > before.propagation_steps)
        std::cout << " propagation_hz=" << (after.propagation_steps - before.propagation_steps) / elapsed
                  << " propagation_err_deg_mean/max="
                  << (checks ? (after.propagation_error_sum_deg - before.propagation_error_sum_deg) / checks : -1)
                  << "/" << after.propagation_error_max_deg;
    std::cout << '\n';
}
