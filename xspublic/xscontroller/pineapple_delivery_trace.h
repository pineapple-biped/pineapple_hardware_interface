#ifndef PINEAPPLE_DELIVERY_TRACE_H
#define PINEAPPLE_DELIVERY_TRACE_H
// Probe-only bounded trace. Prepare before SDK threads start; dump only after
// SDK shutdown. No allocation, locks or file I/O in the event path.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <vector>

struct PineappleDeliveryTrace {
    struct Event {
        double time = 0;
        uintptr_t source = 0;
        long long value = 0;
        const char* stage = nullptr;
    };
    std::vector<Event> events;
    std::atomic<bool> enabled{false};
    std::atomic<size_t> next{0};
    void prepare(size_t capacity) {
        events.resize(capacity); // Value initialize/touch pages before capture.
        next.store(0);
    }
    void emit(const char* stage, const void* source, long long value) {
        if (!enabled.load(std::memory_order_relaxed)) return;
        const double time = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        const size_t i = next.fetch_add(1, std::memory_order_relaxed);
        if (i >= events.size()) return;
        Event& e = events[i];
        e.time = time; e.source = reinterpret_cast<uintptr_t>(source);
        e.value = value; e.stage = stage;
    }
    size_t dropped() const {
        const size_t n = next.load();
        return n > events.size() ? n - events.size() : 0;
    }
    // Caller must have joined all producing threads, not just disabled tracing.
    bool write(FILE* file) const {
        if (std::fprintf(file, "host_monotonic_s,stage,source,value\n") < 0) return false;
        const size_t n = next.load() < events.size() ? next.load() : events.size();
        for (size_t i = 0; i < n; ++i) {
            const Event& e = events[i];
            if (std::fprintf(file, "%.9f,%s,%llu,%lld\n", e.time, e.stage,
                    static_cast<unsigned long long>(e.source), e.value) < 0) return false;
        }
        return true;
    }
};
inline PineappleDeliveryTrace& pineappleDeliveryTrace() {
    static PineappleDeliveryTrace trace;
    return trace;
}
#endif
