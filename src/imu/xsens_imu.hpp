#pragma once

#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>
#include <iostream>
#include <iomanip>
#include <list>
#include <string>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <array>
#include <mutex>

// One internally consistent snapshot of the IMU state.
struct ImuSample {
    std::array<double, 4> quaternion = {1, 0, 0, 0};
    std::array<double, 3> rpy = {0, 0, 0};
    std::array<double, 3> gyro = {0, 0, 0};
    std::array<double, 3> accel = {0, 0, 0};
};


class ImuSharedData {
public:
    void store(const ImuSample &sample)
    {
        seq_.fetch_add(1, std::memory_order_relaxed);      
        std::atomic_thread_fence(std::memory_order_release);
        sample_ = sample;
        std::atomic_thread_fence(std::memory_order_release);
        seq_.fetch_add(1, std::memory_order_relaxed);      
    }

    ImuSample load() const
    {
        for (;;) {
            const uint32_t before = seq_.load(std::memory_order_relaxed);
            if (before & 1u) continue;                     // writer mid-update
            std::atomic_thread_fence(std::memory_order_acquire);
            ImuSample out = sample_;
            std::atomic_thread_fence(std::memory_order_acquire);
            if (seq_.load(std::memory_order_relaxed) == before) return out;
        }
    }

private:
    ImuSample sample_;
    std::atomic<uint32_t> seq_{0};
};

// Journaller* gJournal = 0;

class CallbackHandler : public XsCallback
{
public:
    CallbackHandler(size_t maxBufferSize = 5)
        : m_maxNumberOfPacketsInBuffer(maxBufferSize)
        , m_numberOfPacketsInBuffer(0)
    {}

    bool packetAvailable() const
    {
        xsens::Lock locky(&m_mutex);
        return m_numberOfPacketsInBuffer > 0;
    }

    XsDataPacket getNextPacket()
    {
        xsens::Lock locky(&m_mutex);
        XsDataPacket oldestPacket(m_packetBuffer.front());
        m_packetBuffer.pop_front();
        --m_numberOfPacketsInBuffer;
        return oldestPacket;
    }

protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
    {
        xsens::Lock locky(&m_mutex);
        if (!packet) return;
        while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
            (void)getNextPacket();
        m_packetBuffer.push_back(*packet);
        ++m_numberOfPacketsInBuffer;
    }
private:
    mutable xsens::Mutex m_mutex;
    size_t m_maxNumberOfPacketsInBuffer;
    size_t m_numberOfPacketsInBuffer;
    std::list<XsDataPacket> m_packetBuffer;
};
