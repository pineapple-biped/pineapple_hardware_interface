#include "../pineapple_hardware/command_watchdog.h"
#include <cassert>
#include <limits>
#include <iostream>

int main() {
    using namespace std::chrono_literals;
    using Time = CommandWatchdog::Time;
    const auto t = Time{} + 1s;
    CommandWatchdog w;
    assert(!w.Check(t + 1h)); // Waiting for the first publisher is permitted.
    assert(w.Accept(t));
    assert(!w.Check(t + 99ms));
    assert(w.Accept(t + 99ms));
    assert(!w.Check(t + 198ms));
    assert(w.Check(t + 199ms)); // Exact boundary.
    assert(!w.Accept(t + 200ms)); // No automatic rearm.
    assert(w.Check(t + 1h));
    CommandWatchdog late;
    assert(late.Accept(t));
    assert(!late.Accept(t + 101ms)); // Callback can beat watchdog thread; still stops.
    CommandWatchdog invalid;
    invalid.Trip();
    assert(!invalid.Accept(t));
    CommandWatchdog healthy;
    for (int i = 0; i < 10000; ++i) {
        assert(healthy.Accept(t + i * 5ms));
        assert(!healthy.Check(t + i * 5ms + 2ms));
    }
    assert(CommandWatchdog::Valid(1, 2, 3, 40, .5));
    assert(!CommandWatchdog::Valid(std::numeric_limits<double>::quiet_NaN(),0,0,0,0));
    assert(!CommandWatchdog::Valid(0,std::numeric_limits<double>::infinity(),0,0,0));
    assert(!CommandWatchdog::Valid(0,0,0,-1,0));
    assert(!CommandWatchdog::Valid(0,0,0,501,0));
    assert(!CommandWatchdog::Valid(0,0,0,0,6));
    std::cout << "command watchdog tests passed\n";
}
