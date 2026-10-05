#ifndef PINEAPPLE_SDK2_BRIDGE_H
#define PINEAPPLE_SDK2_BRIDGE_H

#include <iostream>
#include <chrono>
#include <cstring>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include "../canfd/include/damiao.h"
#include "../imu/xsens_imu.hpp"

using namespace unitree::common;
using namespace unitree::robot;
using namespace std;

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_LOWCMD "rt/lowcmd"
#define MOTOR_SENSOR_NUM 3
#define NUM_MOTOR_IDL_GO 20

struct MotorPosLimit {
    double lower;
    double upper;
};

// One platform = one USB2CANFD device (e.g. the wheel biped or the arm).
struct MotorConfig {
    string dev_sn;  // USB2CANFD serial number, from scan_canfd_sn
    bool set_zero = false;
    bool have_imu = false;  // bring up the Xsens MTi and publish its orientation
    vector<uint16_t> can_id_list;
    vector<uint16_t> mst_id_list;
    vector<int> motor_type;
    vector<double> motor_offset;
    vector<double> direction;
    vector<MotorPosLimit> pos_limit;
};

class PineappleSdk2Bridge
{
public:
    // Platforms are concatenated: the joint index in LowCmd/LowState follows
    // the order of platform_configs (platform 0 motors first, then platform 1, ...).
    PineappleSdk2Bridge(const vector<MotorConfig> &platform_configs);
    ~PineappleSdk2Bridge();

    void LowCmdGoHandler(const void *msg);
    void PublishLowStateGo();

    // Motor related
    void SetMotorToZero();
    void AddMotorOffset();

    // IMU related
    bool InitXsensIMU();
    void ProcessXsensData();
    void CloseXsensIMU();

    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_go_suber_;

    unitree_go::msg::dds_::LowState_ low_state_go_{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowState_> low_state_go_puber_;

    ThreadPtr lowStatePuberThreadPtr;


    int dim_motor_sensor_ = 0;

    // Resolved from the platform configs in the ctor, then cleared again if
    // the MTi cannot actually be brought up.
    bool have_imu_ = false;

    // Concatenation of all platform configs, indexed by global joint index.
    int num_motor_ = 0;
    vector<uint16_t> can_id_list;
    vector<uint16_t> mst_id_list;
    vector<int> motor_type;
    vector<double> motor_offset;
    vector<double> direction;
    vector<MotorPosLimit> pos_limit_;

    private:

    void HandleMotorFault(int motor_idx);
    bool IsMotorConnected(int motor_idx) const;

    // Controller owning the given global joint index (one CAN bus per device).
    std::shared_ptr<damiao::Motor_Control> CtrlOf(int motor_idx) const
    {
        return motor_controls_[ctrl_of_motor_[motor_idx]];
    }

    std::atomic<bool> is_running{true};
    // Motor related: one Motor_Control per USB2CANFD device
    vector<std::shared_ptr<damiao::Motor_Control>> motor_controls_;
    vector<size_t> ctrl_of_motor_;  // global joint index -> motor_controls_ index
    uint32_t nom_baud =1000000;
    uint32_t dat_baud =5000000;
    vector<vector<damiao::DmActData>> dm_data_lists_;  // one per device
    vector<std::chrono::steady_clock::time_point> last_fault_recovery_time_;
    vector<std::chrono::steady_clock::time_point> last_disconnect_log_time_;
    double feedback_timeout_sec_ = 0.1;  // no feedback within this window => disconnected
    std::chrono::steady_clock::time_point init_time_;
    double init_grace_sec_ = 5.0;        // suppress disconnect reports during startup
    std::chrono::steady_clock::time_point last_poll_time_;
    double poll_interval_sec_ = 0.02;    // actively solicit feedback at 50 Hz

    // IMU related
    std::thread xsens_imu_thread;
    std::shared_ptr<ImuSharedData> xsens_imu_data;

    static constexpr int kImuOutputRateHz = 100;
    XsControl* xsens_control = nullptr;
    XsPortInfo xsens_mtPort;
    CallbackHandler xsens_callback;
    XsDevice* xsens_device = nullptr;

};

#endif
