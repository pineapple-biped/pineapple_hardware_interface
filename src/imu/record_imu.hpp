#pragma once
#include <xscontroller/xscallback.h>
#include <xstypes/xsdatapacket.h>
#include <array>
#include <cstdio>
#include <limits>
#include <mutex>
#include <vector>
#include "xsens_imu.hpp"

// Probe-only recorder. Capture individual SDK packets, not polled latest values.
// Preallocated bounded memory: no disk I/O or allocation in the callback.
struct ImuRecord {
    double host_s;
    long long counter, ticks;
    int mask; // bit 0 quaternion, bit 1 gyro, bit 2 acceleration
    std::array<double, 10> value;
};

class ImuRecorder : public XsCallback {
public:
    explicit ImuRecorder(size_t capacity) : capacity_(capacity) { rows_.reserve(capacity); }
    void start() { std::lock_guard<std::mutex> lock(mutex_); active_ = true; }
    void stop() { std::lock_guard<std::mutex> lock(mutex_); active_ = false; }
    size_t dropped() const { return dropped_; } // only after stop()
    size_t size() const { return rows_.size(); } // only after stop()
    void capture(const XsDataPacket& p, double now)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_) return;
        if (rows_.size() == capacity_) { ++dropped_; return; }
        ImuRecord r{now, p.containsPacketCounter() ? p.packetCounter() : -1,
                    p.containsSampleTimeFine() ? static_cast<long long>(p.sampleTimeFine()) : -1, 0, {}};
        r.value.fill(std::numeric_limits<double>::quiet_NaN());
        if (p.containsOrientation()) {
            const auto q = p.orientationQuaternion();
            r.mask |= 1;
            r.value[0] = q.w(); r.value[1] = q.x(); r.value[2] = q.y(); r.value[3] = q.z();
        }
        if (p.containsRateOfTurnHR()) {
            r.mask |= 2;
            const auto g = p.rateOfTurnHR();
            for (size_t j = 0; j < 3; ++j) r.value[4+j] = g[j];
        }
        if (p.containsAccelerationHR()) {
            r.mask |= 4;
            const auto a = p.accelerationHR();
            for (size_t j = 0; j < 3; ++j) r.value[7+j] = a[j];
        }
        rows_.push_back(r);
    }
    bool write(FILE* file) const // only after stop()
    {
        std::fprintf(file, "host_monotonic_s,packet_counter,sample_time_fine_ticks,field_mask,qw,qx,qy,qz,gx,gy,gz,ax,ay,az\n");
        for (const auto& r : rows_) {
            std::fprintf(file, "%.9f,%lld,%lld,%d", r.host_s, r.counter, r.ticks, r.mask);
            for (double v : r.value) std::fprintf(file, ",%.17g", v);
            std::fprintf(file, "\n");
        }
        return !std::ferror(file);
    }
protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* p) override
    {
        if (p) capture(*p, imuMonotonicSeconds());
    }
private:
    std::mutex mutex_;
    bool active_ = false;
    size_t capacity_, dropped_ = 0;
    std::vector<ImuRecord> rows_;
};
