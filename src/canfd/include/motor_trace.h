#ifndef PINEAPPLE_MOTOR_TRACE_H
#define PINEAPPLE_MOTOR_TRACE_H
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <vector>

// Opt-in, bounded in-memory trace. No file I/O or allocation in Add().
// Flush on clean shutdown; SIGKILL/power loss cannot preserve this capture.
class MotorTrace {
    struct Event {
        int64_t host_ns;
        uint32_t id, adapter_raw;
        unsigned event, size, dlc, direction;
        std::array<uint8_t, 8> data{};
    };
    std::ofstream file_;
    std::mutex mutex_;
    std::vector<Event> events_;
    const size_t capacity_;
    size_t dropped_ = 0;
public:
    static int64_t Now() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    explicit MotorTrace(const std::string& path, size_t capacity = 2000000)
        : file_(path), capacity_(capacity) {
        if (!file_) throw std::runtime_error("Cannot open motor trace: " + path);
        events_.reserve(capacity_);
    }
    void Add(unsigned event, uint32_t id, uint32_t adapter_raw,
             const uint8_t* data, size_t size, unsigned dlc = 0,
             unsigned direction = 0, int64_t host_ns = 0) {
        Event row{host_ns ? host_ns : Now(), id, adapter_raw, event,
                  static_cast<unsigned>(size), dlc, direction, {}};
        std::copy_n(data, std::min(size, size_t(8)), row.data.begin());
        std::lock_guard<std::mutex> lock(mutex_);
        if (events_.size() == capacity_) { ++dropped_; return; }
        events_.push_back(row);
    }
    ~MotorTrace() {
        file_ << "# event 0=adapter_callback,1=USB_submit_begin,2=USB_submit_return,3=decoded_feedback,4=DDS_snapshot_use\n"
              << "# adapter timestamp units/clock and direction semantics unverified; USB return is not motor application\n"
              << "# events 3/4: b0..b7 encode per-motor sequence little-endian\n"
              << "# dropped=" << dropped_ << "\n"
              << "host_monotonic_ns,event,can_id,adapter_timestamp_raw,size,dlc_raw,direction_raw,b0,b1,b2,b3,b4,b5,b6,b7\n";
        for (const auto& e : events_) {
            file_ << e.host_ns << ',' << e.event << ',' << e.id << ',' << e.adapter_raw
                  << ',' << e.size << ',' << e.dlc << ',' << e.direction;
            for (auto b : e.data) file_ << ',' << unsigned(b);
            file_ << '\n';
        }
        file_.flush();
        if (!file_) std::cerr << "Motor trace write failed; capture incomplete\n";
    }
};
#endif
