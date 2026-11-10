#ifndef PINEAPPLE_SDK2_BRIDGE_H
#define PINEAPPLE_SDK2_BRIDGE_H

#include <iostream>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include "../canfd/include/damiao.h"
#include "../imu/xsens_imu.hpp"
#include "../imu/configure_xsens.hpp"
#include "motor_config.hpp"

using namespace unitree::common;
using namespace unitree::robot;
using namespace std;

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_LOWCMD "rt/lowcmd"
#define MOTOR_SENSOR_NUM 3
#define NUM_MOTOR_IDL_GO 20

class PineappleSdk2Bridge
{
public:
    // Platforms are concatenated: the joint index in LowCmd/LowState follows
    // the order of platform_configs (platform 0 motors first, then platform 1, ...).
    PineappleSdk2Bridge(const vector<MotorConfig> &platform_configs,
                        bool skip_imu_timing_check = false);
    ~PineappleSdk2Bridge();

    void LowCmdGoHandler(const void *msg);
    void PublishLowStateGo();   // legacy 500 Hz timer: poll, then publish


    // Motor related
    void SetMotorToZero();
    void AddMotorOffset();

    // IMU related
    bool InitXsensIMU();
    void CloseXsensIMU();

    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_go_suber_;

    unitree_go::msg::dds_::LowState_ low_state_go_{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowState_> low_state_go_puber_;

    ThreadPtr lowStatePuberThreadPtr;


    int dim_motor_sensor_ = 0;

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

    bool skip_imu_timing_check_ = false;

    // Joint-frame target for one motor, as received in LowCmd_.
    struct JointCmd { double kp = 0, kd = 0, q = 0, dq = 0, tau = 0; };
    // Clamp to the joint limits, convert to the motor frame and send one MIT frame.
    void SendJointCommand(int i, const JointCmd &c);
    void PollStaleMotors();     // solicit feedback from motors silent for poll_interval_sec_
    void FillAndWriteLowState();

    // ---- Command repeat (PINEAPPLE_CMD_REPEAT_HZ > 0) ----
    // Damiao motors only report state in reply to a command, so the state age is set by
    // the command rate. In this mode one control thread owns the CAN bus: it re-sends the
    // newest LowCmd_ targets at cmd_repeat_hz_, sends a new LowCmd_ the moment it
    // arrives, waits for the replies and then publishes LowState_. The handler only stores
    // the targets. Repeating stops cmd_timeout_sec_ after the last LowCmd_, which leaves
    // the motors exactly as the legacy path does when the deploy stops publishing.
    void ControlLoop();
    bool AllMotorsReplied(const vector<uint64_t> &seq_before) const;
    double ProcessCpuPercent();
    int cmd_repeat_hz_ = 0;
    int rt_priority_ = 0;                  // PINEAPPLE_CTRL_RT_PRIORITY; 0 = normal scheduling
    std::mutex reply_mutex_;               // pairs with reply_cv_ (motor reply wake-ups)
    std::condition_variable reply_cv_;
    double cpu_stat_proc_ = 0;
    std::chrono::steady_clock::time_point cpu_stat_wall_{};
    uint64_t legacy_publish_count_ = 0;
    double cmd_timeout_sec_ = 0.05;
    double reply_wait_sec_ = 0.0;          // set from the period in the constructor
    std::thread control_thread_;
    std::mutex cmd_mutex_;
    std::condition_variable cmd_cv_;
    vector<JointCmd> cmd_target_;          // guarded by cmd_mutex_
    bool cmd_valid_ = false, cmd_fresh_ = false;
    std::chrono::steady_clock::time_point cmd_received_;
    uint32_t pending_entry_ = 0, pending_seq_ = 0, pending_state_tick_ = 0;

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

    // ---- Latency instrumentation (pineapple_rl_deploy docs/latency_timestamps.md) ----
    // Every stamp is CLOCK_MONOTONIC microseconds mod 2^32 ("mono32"), the clock Python's
    // time.monotonic_ns() reads, so bridge, deploy and recorder times compare directly.
    // Carried in LowState_ fields this bridge does not otherwise use: `tick`, `reserve`,
    // each motor's `reserve[2]` / `lost`, and spare motor slots LAT_SLOT_A/B (only when
    // num_motor_ leaves them free).
    static constexpr int LAT_SLOT_A = 19;  // IMU stamps
    static constexpr int LAT_SLOT_B = 18;  // lowcmd-handler stamps + deploy echoes
    static uint32_t Mono32(std::chrono::steady_clock::time_point tp)
    {
        return static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch()).count());
    }
    static uint32_t Mono32Seconds(double s) { return s > 0 ? static_cast<uint32_t>(static_cast<int64_t>(s * 1e6)) : 0; }
    std::atomic<uint32_t> lat_cmd_entry_{0};       // LowCmdGoHandler entered
    std::atomic<uint32_t> lat_cmd_exit_{0};        // all MIT frames handed to USB
    std::atomic<uint32_t> lat_cmd_seq_{0};         // echo of LowCmd_.reserve (deploy sequence)
    std::atomic<uint32_t> lat_cmd_state_tick_{0};  // echo of the LowState tick the policy used
    std::atomic<uint32_t> lat_cmd_count_{0};       // lowcmd messages handled

    // IMU related
    std::shared_ptr<ImuSharedData> xsens_imu_data;

    XsControl* xsens_control = nullptr;
    XsPortInfo xsens_mtPort;
    CallbackHandler xsens_callback;
    XsDevice* xsens_device = nullptr;

};

#endif
