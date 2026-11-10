#include "../imu/configure_xsens.hpp"
#include <stdexcept>

Journaller* gJournal = nullptr;

static void require(bool value)
{
    if (!value) throw std::runtime_error("IMU timing override regression");
}

static bool checkBuffered(bool skip)
{
    ImuSharedData data;
    std::thread source([&data] {
        for (int i = 0; i < 10; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            XsDataPacket packet;
            packet.setOrientationQuaternion(XsQuaternion(1, 0, 0, 0), XDI_CoordSysEnu);
            packet.setRateOfTurnHR(XsVector3(0, 0, 0));
            packet.setAccelerationHR(XsVector3(0, 0, 9.81));
            data.ingest(packet, imuMonotonicSeconds());
        }
    });
    const bool result = verifyImuDelivery(data, imuOutputProfile("fast500"), .2, skip);
    source.join();
    return result;
}

int main()
{
    require(!checkBuffered(false));
    require(checkBuffered(true));
    ImuSharedData empty;
    require(!verifyImuDelivery(empty, imuOutputProfile("fast500"), .01, true));
}
