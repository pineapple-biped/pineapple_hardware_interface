// Sensor-only diagnostic: no motor library, command subscriber or command topic.
#include "../imu/configure_xsens.hpp"
#include "../imu/xsens_imu.hpp"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <csignal>
#include <cstring>
#include <thread>
#include <string>
Journaller* gJournal = nullptr;
static volatile std::sig_atomic_t stopped = 0;
static void stop(int) { stopped = 1; }

// Explicit little-endian doubles, decoded with struct '<4sI4d' in Python.
static void putDouble(std::array<uint8_t,40>& bytes, size_t offset, double value) {
    uint64_t bits; std::memcpy(&bits, &value, 8);
    for (int i=0; i<8; ++i) bytes[offset+i] = (bits >> (8*i)) & 255;
}
int main(int argc, char** argv) {
    int seconds = 130;
    bool synthetic = false;
    try {
        for (int i=1;i<argc;++i) {
            std::string arg=argv[i];
            if (arg=="--synthetic") synthetic=true;
            else if (arg=="--seconds" && i+1<argc) {
                std::string value=argv[++i]; size_t used=0;
                seconds=std::stoi(value,&used);
                if (used!=value.size() || seconds<1 || seconds>300) throw std::runtime_error("seconds: 1..300");
            } else if (arg=="--help") {
                std::cout << "imu_dds_probe [--seconds 1..300] [--synthetic]\n"
                          << "IMU only; publishes rt/imu_probe/lowstate on domain 0, loopback. No motors.\n";
                return 0;
            } else throw std::runtime_error("Unknown argument: "+arg);
        }
    } catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 2; }
    std::signal(SIGINT,stop); std::signal(SIGTERM,stop);
    unitree::robot::ChannelFactory::Instance()->Init(0,"lo");
    unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowState_> pub("rt/imu_probe/lowstate");
    pub.InitChannel();
    XsControl* control=nullptr;
    XsPortInfo port;
    auto shared=std::make_shared<ImuSharedData>();
    CallbackHandler callback; callback.setTarget(shared);
    if (!synthetic) {
        control=XsControl::construct();
        if (!control) return 1;
        for (const auto& candidate: XsScanner::scanPorts()) {
            if (candidate.deviceId().isMti() || candidate.deviceId().isMtig()) { port=candidate; break; }
        }
        if (port.empty() || !control->openPort(port.portName().toStdString(),port.baudrate())) {
            std::cerr<<"No usable IMU\n"; control->destruct(); return 1;
        }
        auto device=control->device(port.deviceId());
        device->addCallbackHandler(&callback);
        try {
            if (!configureXsens(*device,imuOutputProfileFromEnvironment(),port.baudrate())) {
                control->destruct(); return 1;
            }
        } catch (const std::exception& e) {
            std::cerr<<e.what()<<'\n'; control->destruct(); return 1;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    const auto initial=shared->load();
    uint32_t sequence=0; unsigned failures=0, skipped=0;
    auto next=std::chrono::steady_clock::now();
    const double begin=imuMonotonicSeconds();
    std::cout<<"Publishing diagnostic LowState at 500 Hz on loopback; synthetic="<<synthetic<<std::endl;
    while (!stopped && imuMonotonicSeconds()-begin<seconds) {
        ImuSample imu=shared->load();
        if (synthetic) {
            double now=imuMonotonicSeconds();
            imu.received_at={begin+std::floor((now-begin)*100)/100,now,now};
            imu.accel={0,0,9.81}; imu.gyro={0.01,0.02,0.03};
        }
        if (*std::min_element(imu.received_at.begin(),imu.received_at.end())<=0) {
            std::cerr<<"Missing IMU field; stopping\n"; failures++; break;
        }
        unitree_go::msg::dds_::LowState_ message;
        for (int i=0;i<4;++i) message.imu_state().quaternion()[i]=imu.quaternion[i];
        for (int i=0;i<3;++i) {
            message.imu_state().gyroscope()[i]=imu.gyro[i];
            message.imu_state().accelerometer()[i]=imu.accel[i];
        }
        auto& meta=message.wireless_remote();
        // Diagnostic-only metadata; this message is NEVER sent to rt/lowstate.
        std::memcpy(meta.data(),synthetic ? "IMS1" : "IMD1",4);
        for (int i=0;i<4;++i) meta[4+i]=(sequence>>(8*i))&255;
        putDouble(meta,8,imuMonotonicSeconds());
        for (int i=0;i<3;++i) putDouble(meta,16+8*i,imu.received_at[i]);
        if (!pub.Write(message)) ++failures;
        ++sequence;
        next+=std::chrono::milliseconds(2);
        if (next<std::chrono::steady_clock::now()) { ++skipped; next=std::chrono::steady_clock::now()+std::chrono::milliseconds(2); }
        std::this_thread::sleep_until(next);
    }
    auto final=shared->load();
    if (control) control->destruct();
    std::cout<<"Published="<<sequence<<" write_failures="<<failures<<" late_ticks="<<skipped
             <<" missing_imu_packets="<<final.missing_packets-initial.missing_packets
             <<" invalid_quaternions="<<final.invalid_quaternions-initial.invalid_quaternions<<'\n';
    return failures ? 1 : 0;
}
