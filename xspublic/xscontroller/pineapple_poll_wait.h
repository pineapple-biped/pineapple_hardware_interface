#pragma once
#include <cstddef>
#include <cstdlib>
#include <cstring>

// Opt-in comparison only. Keep vendor behavior unless explicitly requested.
inline bool pineappleLowLatencyPoll()
{
    static const bool enabled = [] {
        const char* value = std::getenv("PINEAPPLE_XSENS_LOW_LATENCY");
        return value && std::strcmp(value, "1") == 0;
    }();
    return enabled;
}

inline int pineapplePollWaitMs(std::size_t bytes, bool lowLatency)
{
    if (bytes == 0) return lowLatency ? 1 : 3;
    if (bytes < 256) return lowLatency ? 1 : 2;
    return 0; // drain large backlogs immediately, preserving vendor behavior
}
