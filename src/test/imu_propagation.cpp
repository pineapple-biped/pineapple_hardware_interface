// Gyro propagation of the published IMU orientation (PINEAPPLE_IMU_PROPAGATE=1).
// Simulates an MTi stream: quaternion at 100 Hz, gyro at 500 Hz, constant body rate.
#include "../imu/xsens_imu.hpp"
#include <cstdio>
#include <stdexcept>

Journaller* gJournal = nullptr;

static void require(bool value, const char* what)
{
    if (!value) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        throw std::runtime_error(what);
    }
}

// Orientation of a body spinning at constant rate w (body frame) from identity, after t s.
static std::array<double, 4> truth(const std::array<double, 3>& w, double t)
{
    return integrateQuaternion({1, 0, 0, 0}, w, t);
}

static XsDataPacket packet(uint32_t ticks, const std::array<double, 3>& w, const std::array<double, 4>* q)
{
    XsDataPacket p;
    p.setSampleTimeFine(ticks);
    p.setRateOfTurnHR(XsVector3(w[0], w[1], w[2]));
    if (q) p.setOrientationQuaternion(XsQuaternion((*q)[0], (*q)[1], (*q)[2], (*q)[3]), XDI_CoordSysEnu);
    return p;
}

int main()
{
    // integrateQuaternion: 1 rad/s about z for 1 s -> 1 rad yaw.
    const auto yaw = integrateQuaternion({1, 0, 0, 0}, {0, 0, 1}, 1.0);
    require(std::fabs(yaw[0] - std::cos(0.5)) < 1e-12 && std::fabs(yaw[3] - std::sin(0.5)) < 1e-12,
            "integrateQuaternion yaw");
    require(quaternionAngleDeg(yaw, {1, 0, 0, 0}) - 180 / M_PI < 1e-9, "quaternionAngleDeg");

    const std::array<double, 3> w = {0.3, -1.2, 2.0};  // rad/s, ~135 deg/s
    for (bool on : {false, true}) {
        ImuSharedData data;
        data.setPropagate(on);
        double worst_held = 0, worst_live = 0;
        for (int k = 0; k <= 100; ++k) {           // 0.2 s at 500 Hz
            const uint32_t ticks = 1000 + 20 * k;  // 2 ms in 10 kHz ticks
            const double t = 20 * k * 1e-4;
            const auto q = truth(w, t);
            const XsDataPacket p = packet(ticks, w, k % 5 == 0 ? &q : nullptr);  // quat every 10 ms
            data.ingest(p, t);
            const ImuSample s = data.load();
            const double err = quaternionAngleDeg(s.quaternion_out, q);
            (k % 5 ? worst_live : worst_held) = std::max(k % 5 ? worst_live : worst_held, err);
        }
        const ImuSample s = data.load();
        if (on) {
            require(worst_live < 1e-4,  // packet stores single precision
                    "propagated orientation tracks a constant rate");
            require(s.propagation_steps == 100, "one propagation step per gyro sample");
            require(s.propagation_checks == 20 && s.propagation_error_max_deg < 1e-4,
                    "propagation check against the next device quaternion");
        } else {
            // Without propagation the output holds the last quaternion: up to 8 ms stale.
            require(worst_live > 1.0, "held orientation lags");
            require(s.propagation_steps == 0, "no propagation when off");
        }
        require(worst_held < 1e-4, "device quaternion published as-is");
    }

    // A stalled quaternion stream holds after 50 ms instead of integrating open loop.
    ImuSharedData data;
    data.setPropagate(true);
    const std::array<double, 4> id = {1, 0, 0, 0};
    data.ingest(packet(0, w, &id), 0);
    for (int k = 1; k <= 50; ++k) data.ingest(packet(20 * k, w, nullptr), 0.002 * k);  // 100 ms
    const ImuSample s = data.load();
    require(s.propagation_steps == 25, "propagation stops 50 ms after the last quaternion");
    std::puts("imu_propagation: PASS");
}
