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
    // Host acquisition times, NOT device capture timestamps.
    std::array<double, 3> received_at = {0, 0, 0};
    std::array<uint64_t, 3> updates = {0, 0, 0};
    uint64_t packets = 0, missing_packets = 0, invalid_quaternions = 0;
    std::array<double, 3> max_field_gap_ms = {0, 0, 0};
    double max_callback_gap_ms = 0;
    double device_interval_ms = 0;
};

// No seqlock over ordinary memory: reader/writer copies must both hold the lock.
class ImuSharedData {
public:
    ImuSample load() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return sample_;
    }

    void resetTimingStats()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sample_.max_callback_gap_ms = 0;
        sample_.max_field_gap_ms = {0, 0, 0};
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
            // Wrap is valid; large backwards jumps indicate a restart, not loss.
            if (have_counter_ && delta > 1 && delta < 32768)
                sample_.missing_packets += delta - 1;
            previous_counter_ = counter;
            have_counter_ = true;
        }
        if (packet.containsSampleTimeFine()) {
            const auto ticks = packet.sampleTimeFine();
            const uint32_t delta = ticks - previous_ticks_;
            // Xsens SampleTimeFine is in 0.1 ms ticks; unsigned wrap is valid.
            if (have_ticks_ && delta < 100000)
                sample_.device_interval_ms = delta * 0.1;
            previous_ticks_ = ticks;
            have_ticks_ = true;
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
                if (sample_.received_at[0] > 0)
                    sample_.max_field_gap_ms[0] = std::max(sample_.max_field_gap_ms[0],
                        1000 * (now - sample_.received_at[0]));
                sample_.received_at[0] = now;
                ++sample_.updates[0];
            } else {
                ++sample_.invalid_quaternions;
            }
        }
        if (packet.containsRateOfTurnHR()) {
            const auto g = packet.rateOfTurnHR();
            for (size_t i = 0; i < 3; ++i) sample_.gyro[i] = g[i];
            if (sample_.received_at[1] > 0)
                sample_.max_field_gap_ms[1] = std::max(sample_.max_field_gap_ms[1],
                    1000 * (now - sample_.received_at[1]));
            sample_.received_at[1] = now;
            ++sample_.updates[1];
        }
        if (packet.containsAccelerationHR()) {
            const auto a = packet.accelerationHR();
            for (size_t i = 0; i < 3; ++i) sample_.accel[i] = a[i];
            if (sample_.received_at[2] > 0)
                sample_.max_field_gap_ms[2] = std::max(sample_.max_field_gap_ms[2],
                    1000 * (now - sample_.received_at[2]));
            sample_.received_at[2] = now;
            ++sample_.updates[2];
        }
    }
private:
    mutable std::mutex mutex_;
    ImuSample sample_;
    double last_callback_ = 0;
    uint16_t previous_counter_ = 0;
    uint32_t previous_ticks_ = 0;
    bool have_counter_ = false, have_ticks_ = false;
};

class CallbackHandler : public XsCallback {
public:
    // Configure before registering the callback; target remains alive through closePort.
    void setTarget(std::shared_ptr<ImuSharedData> target) { target_ = std::move(target); }
protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
    {
        // Publish immediately; no five-packet queue or extra polling thread.
        // No I/O, sleeping, or motor operations in the SDK callback.
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
              << " invalid_quaternions=" << after.invalid_quaternions << '\n';
}
