#pragma once
#include <cstdlib>
#include <cstring>
#include <cstdint>

inline bool pineappleReadinessEnabled()
{
    static const bool enabled = [] {
        const char* value = std::getenv("PINEAPPLE_XSENS_READINESS");
        return value && std::strcmp(value, "1") == 0;
    }();
    return enabled;
}

// Reset before each poller read. Set only by the POSIX serial implementation
// after a bounded timeout or successful read. Other transports retain sleeps.
inline bool& pineappleReadinessApplied()
{
    static thread_local bool applied = false;
    return applied;
}

inline uint32_t pineappleReadinessTimeout(uint32_t original, bool enabled)
{
    return enabled && original == 0 ? 10 : original;
}

#ifndef _WIN32
#include <sys/select.h>
#include <cerrno>
inline int pineappleSelectReadable(int handle, uint32_t timeoutMs)
{
    if (handle < 0 || handle >= FD_SETSIZE) { errno = EINVAL; return -1; }
    fd_set readable, errors;
    FD_ZERO(&readable); FD_ZERO(&errors);
    FD_SET(handle, &readable); FD_SET(handle, &errors);
    timeval timeout;
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;
    int result = select(handle + 1, &readable, nullptr, &errors, &timeout);
    if (result < 0 || FD_ISSET(handle, &errors)) return -1;
    return result;
}
#endif
