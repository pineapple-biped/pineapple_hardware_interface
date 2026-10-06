#include "../imu/xsens_imu.hpp"
#include "../imu/output_profile.hpp"
#include "../imu/record_imu.hpp"
#include <atomic>
#include <xscontroller/pineapple_poll_wait.h>
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
    // Concurrent producers reserve distinct slots; bounded overflow is explicit.
    auto& trace = pineappleDeliveryTrace();
    trace.prepare(2000);
    trace.emit("disabled", nullptr, 0);
    assert(trace.next.load() == 0);
    trace.enabled.store(true);
    std::thread trace_a([&] { for (int i=0; i<1500; ++i) trace.emit("a", nullptr, i); });
    std::thread trace_b([&] { for (int i=0; i<1500; ++i) trace.emit("b", nullptr, i); });
    trace_a.join(); trace_b.join();
    trace.enabled.store(false);
    assert(trace.next.load() == 3000 && trace.dropped() == 1000);
    for (const auto& event : trace.events) assert(event.stage && event.time > 0);
    FILE* trace_csv = std::tmpfile();
    assert(trace_csv && trace.write(trace_csv));
    std::rewind(trace_csv);
    char trace_line[256]; size_t lines = 0;
    while (std::fgets(trace_line, sizeof(trace_line), trace_csv)) ++lines;
    assert(lines == 2001);
    std::fclose(trace_csv);
    assert(pineapplePollWaitMs(0, false) == 3);
    assert(pineapplePollWaitMs(128, false) == 2);
    assert(pineapplePollWaitMs(128, true) == 1);
    assert(pineapplePollWaitMs(0, true) == 1);
    assert(pineapplePollWaitMs(256, false) == 0);
    assert(pineapplePollWaitMs(256, true) == 0);
    const auto fast = imuOutputProfile("fast");
    assert(fast.hz[0] == 100 && fast.hz[1] == 1000 && fast.minimum_baud == 2000000);
    assert(imuOutputProfile("baseline").hz[2] == 100);
    // Linux uses enum 4107 for 2 Mbaud; integer casts must never gate profiles.
    assert(imuProfileSupportsBaud(fast, XsBaud::numericToRate(2000000)));
    assert(!imuProfileSupportsBaud(fast, XsBaud::numericToRate(115200)));
    assert(imuProfileSupportsBaud(imuOutputProfile("baseline"), XsBaud::numericToRate(115200)));
    assert(!imuProfileSupportsBaud(imuOutputProfile("baseline"), XsBaud::numericToRate(9600)));
    bool rejected = false;
    try { imuOutputProfile("typo"); } catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    assert(imuDeliveryWindowHealthy(fast, 1, 1000, 1, 2));
    assert(!imuDeliveryWindowHealthy(fast, 1, 1000, 1, 50)); // bursts despite correct mean
    assert(!imuDeliveryWindowHealthy(fast, 1, 100, 1, 2));
    assert(!imuDeliveryWindowHealthy(fast, 0, 100, 25, 10));
    ImuRecorder recorder(2);
    recorder.capture(packet(1, 10, .1), 1); // disabled until explicitly started
    assert(recorder.size() == 0);
    recorder.start();
    recorder.capture(packet(1, 10, .1), 1);
    XsDataPacket partial_record;
    XsVector g(3); g[0] = .2; g[1] = 0; g[2] = 0;
    partial_record.setRateOfTurnHR(g);
    recorder.capture(partial_record, 1.001);
    recorder.capture(packet(2, 20, .1), 1.002);
    recorder.stop();
    assert(recorder.size() == 2 && recorder.dropped() == 1);
    FILE* csv = std::tmpfile(); assert(csv);
    assert(recorder.write(csv)); std::rewind(csv);
    char line[1024]; assert(std::fgets(line, sizeof(line), csv));
    assert(std::fgets(line, sizeof(line), csv));
    assert(std::string(line).find(",1,10,7,") != std::string::npos);
    assert(std::fgets(line, sizeof(line), csv));
    assert(std::string(line).find(",-1,-1,2,nan,nan,nan,nan,") != std::string::npos);
    std::fclose(csv);
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
    // A fast stream of partial packets cannot conceal a stale orientation field.
    assert(s.load().max_field_gap_ms[0] > 49);
    s.resetTimingStats();
    assert(s.load().max_field_gap_ms[0] == 0);
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
