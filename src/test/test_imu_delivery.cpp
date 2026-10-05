#include "../imu/xsens_imu.hpp"
#include <atomic>
#include <cassert>
#include <thread>

XsDataPacket packet(uint16_t counter, uint32_t ticks, double pitch)
{
    XsDataPacket p;
    p.setPacketCounter(counter);
    p.setSampleTimeFine(ticks);
    const XsQuaternion q(std::cos(pitch / 2), 0, std::sin(pitch / 2), 0);
    p.setOrientationQuaternion(q, XDI_CoordSysEnu);
    XsVector g(3), a(3);
    for (int i = 0; i < 3; ++i) { g[i] = pitch; a[i] = 9.81; }
    p.setRateOfTurnHR(g);
    p.setAccelerationHR(a);
    return p;
}

int main()
{
    ImuSharedData s;
    // Device and packet counter rollover must not cause a false loss spike.
    s.ingest(packet(65535, 0xffffffce, .1), 1.0);
    s.ingest(packet(0, 50, .2), 1.01);
    auto v = s.load();
    assert(v.missing_packets == 0 && v.device_interval_ms == 10);
    assert(v.updates[0] == 2 && v.updates[1] == 2 && v.updates[2] == 2);
    s.ingest(packet(3, 350, .3), 1.06);
    assert(s.load().missing_packets == 2);
    assert(s.load().max_callback_gap_ms > 49);
    // A packet without orientation must not refresh its age or clear its value.
    XsDataPacket partial;
    partial.setPacketCounter(4);
    s.ingest(partial, 1.07);
    assert(s.load().received_at[0] == 1.06 && s.load().updates[0] == 3);
    // Invalid quaternion is rejected; no new freshness is claimed.
    auto bad = packet(5, 450, .4);
    bad.setOrientationQuaternion(XsQuaternion(0, 0, 0, 0), XDI_CoordSysEnu);
    s.ingest(bad, 1.08);
    assert(s.load().invalid_quaternions == 1 && s.load().updates[0] == 3);
    // Exercise concurrent publication: every observed tuple comes from one sample.
    ImuSharedData concurrent;
    std::atomic<bool> done{false};
    std::thread writer([&] {
        for (int i = 1; i <= 10000; ++i)
            concurrent.ingest(packet(i, i * 100, .001 * (i % 100)), i * .01);
        done = true;
    });
    while (!done) {
        auto state = concurrent.load();
        assert(state.updates[0] == state.updates[1]);
        assert(state.updates[1] == state.updates[2]);
        assert(state.received_at[0] == state.received_at[1]);
        assert(state.received_at[1] == state.received_at[2]);
    }
    writer.join();
    assert(concurrent.load().updates[0] == 10000);
    std::cout << "IMU delivery tests passed\n";
}
