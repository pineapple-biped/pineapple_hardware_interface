#include "damiao.h"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <thread>
#include <unistd.h>

int main() {
    // Motor alone never opens USB; only Motor_Control owns hardware.
    damiao::Motor motor(damiao::DM6006, damiao::MIT_MODE, 8, 24);
    assert(motor.GetFeedback().sequence == 0);
    std::atomic<bool> finished{false};
    std::thread writer([&] {
        for (int i = 1; i <= 100000; ++i)
            motor.receive_data(i, -i, 2*i, 0, 0, 0, i, i*100LL);
        finished = true;
    });
    uint64_t previous = 0;
    while (!finished) {
        const auto s = motor.GetFeedback();
        assert(s.sequence >= previous);
        assert(s.dq == -s.q && s.tau == 2*s.q);
        assert(s.sequence == s.adapter_timestamp_raw);
        assert(s.host_receive_ns == int64_t(s.sequence)*100);
        previous = s.sequence;
    }
    writer.join();
    assert(motor.GetFeedback().sequence == 100000);
    motor.set_param(21, 12.5f);
    assert(motor.is_have_param(21) && motor.get_param_as_float(21) == 12.5f);
    motor.clear_param(21);
    assert(!motor.is_have_param(21));
    motor.set_param(21, 0.0f);
    assert(motor.is_have_param(21)); // Valid zero differs from no reply.

    const auto path = "/tmp/pineapple-motor-trace-test-" + std::to_string(getpid()) + ".csv";
    {
        MotorTrace trace(path, 2);
        uint8_t data[8]{1,2,3,4,5,6,7,8};
        for (int i = 0; i < 3; ++i) trace.Add(0, 24, 1234, data, 8, 8, 0, 5678);
    }
    std::ifstream f(path);
    const std::string text((std::istreambuf_iterator<char>(f)), {});
    assert(MotorTrace::ParseCapacity("4000000") == 4000000);
    for (const auto* bad : {"0", "-1", "1.5", "10000001", "", "9999999999999999999999999"}) {
        bool rejected = false;
        try { MotorTrace::ParseCapacity(bad); } catch (const std::exception&) { rejected = true; }
        assert(rejected);
    }
    assert(text.find("# capacity=2") != std::string::npos);
    assert(text.find("# retained=2") != std::string::npos);
    assert(text.find("# first_drop_host_monotonic_ns=5678") != std::string::npos);
    assert(text.find("# dropped=1") != std::string::npos);
    assert(text.find("5678,0,24,1234,8,8,0,1,2,3,4,5,6,7,8") != std::string::npos);
    std::filesystem::remove(path);
}
