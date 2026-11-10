#include "pineapple_sdk2_bridge.h"

#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <pthread.h>
#include <stdexcept>

PineappleSdk2Bridge::PineappleSdk2Bridge(const vector<MotorConfig> &platform_configs,
                                         bool skip_imu_timing_check)
    : skip_imu_timing_check_(skip_imu_timing_check)
{
    xsens_imu_data = std::make_shared<ImuSharedData>();

    // The MTi is a robot-level device, not a per-platform one: any config that
    // declares it turns it on, so argument order cannot change whether
    // orientation gets published.
    have_imu_ = std::any_of(platform_configs.begin(), platform_configs.end(),
                            [](const MotorConfig &cfg) { return cfg.have_imu; });
    if (have_imu_ && !InitXsensIMU()) {
        CloseXsensIMU();
        throw std::runtime_error("IMU startup check failed; motors were not enabled");
    }
    if (!have_imu_) {
        // Publish a valid identity rather than the IDL's default {0,0,0,0},
        // which NaNs any consumer that normalizes the quaternion.
        low_state_go_.imu_state().quaternion()[0] = 1.0f;
    }

    dm_data_lists_.reserve(platform_configs.size());

    for (const auto &cfg : platform_configs)
    {
        size_t ctrl_idx = motor_controls_.size();
        dm_data_lists_.emplace_back();
        auto &dm_data = dm_data_lists_.back();
        for (size_t i = 0; i < cfg.can_id_list.size(); i++)
        {
            dm_data.push_back(damiao::DmActData{.motorType = static_cast<damiao::DM_Motor_Type>(cfg.motor_type[i]),
            .mode = damiao::MIT_MODE,
            .can_id=static_cast<uint16_t>(cfg.can_id_list[i]),
            .mst_id=static_cast<uint16_t>(cfg.mst_id_list[i])});
            ctrl_of_motor_.push_back(ctrl_idx);
        }
        motor_controls_.push_back(std::make_shared<damiao::Motor_Control>(nom_baud,dat_baud,
          cfg.dev_sn,&dm_data));

        can_id_list.insert(can_id_list.end(), cfg.can_id_list.begin(), cfg.can_id_list.end());
        mst_id_list.insert(mst_id_list.end(), cfg.mst_id_list.begin(), cfg.mst_id_list.end());
        motor_type.insert(motor_type.end(), cfg.motor_type.begin(), cfg.motor_type.end());
        motor_offset.insert(motor_offset.end(), cfg.motor_offset.begin(), cfg.motor_offset.end());
        direction.insert(direction.end(), cfg.direction.begin(), cfg.direction.end());
        pos_limit_.insert(pos_limit_.end(), cfg.pos_limit.begin(), cfg.pos_limit.end());

        if (cfg.set_zero)
        {
            for (auto can_id : cfg.can_id_list)
            {
                motor_controls_[ctrl_idx]->set_zero_position(*motor_controls_[ctrl_idx]->getMotor(can_id));
            }
        }
    }
    num_motor_ = can_id_list.size();
    // Before the lowcmd subscriber exists: the handler reads both.
    if (const char *hz = std::getenv("PINEAPPLE_CMD_REPEAT_HZ"))
        cmd_repeat_hz_ = std::atoi(hz);
    if (cmd_repeat_hz_ < 0 || cmd_repeat_hz_ > 2000)
        throw std::runtime_error("PINEAPPLE_CMD_REPEAT_HZ must be 0 (off) .. 2000");
    if (const char *t = std::getenv("PINEAPPLE_CMD_TIMEOUT_MS"))
        cmd_timeout_sec_ = std::atof(t) / 1000.0;
    if (const char *p = std::getenv("PINEAPPLE_CTRL_RT_PRIORITY"))
        rt_priority_ = std::atoi(p);
    cmd_target_.resize(num_motor_);

    last_fault_recovery_time_.resize(num_motor_, std::chrono::steady_clock::now());
    last_disconnect_log_time_.resize(num_motor_, std::chrono::steady_clock::now());

    low_cmd_go_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    low_cmd_go_suber_->InitChannel(bind(&PineappleSdk2Bridge::LowCmdGoHandler, this, placeholders::_1), 1);

    low_state_go_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    low_state_go_puber_->InitChannel();

    init_time_ = std::chrono::steady_clock::now();
    last_poll_time_ = init_time_;
    if (cmd_repeat_hz_ > 0) {
        reply_wait_sec_ = 0.8 / cmd_repeat_hz_;
        std::cout << "[ctrl] command repeat " << cmd_repeat_hz_ << " Hz, lowstate published after "
                  << "motor replies (wait <= " << reply_wait_sec_ * 1000 << " ms), repeat stops "
                  << cmd_timeout_sec_ * 1000 << " ms after the last lowcmd, SCHED_FIFO "
                  << (rt_priority_ > 0 ? std::to_string(rt_priority_) : std::string("off")) << std::endl;
        control_thread_ = std::thread(&PineappleSdk2Bridge::ControlLoop, this);
    } else {
        lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &PineappleSdk2Bridge::PublishLowStateGo, this);
    }

}

PineappleSdk2Bridge::~PineappleSdk2Bridge()
{
    is_running = false;


    if (low_cmd_go_suber_) {
        low_cmd_go_suber_->CloseChannel();
    }

    cmd_cv_.notify_all();
    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    if (lowStatePuberThreadPtr) {
        std::static_pointer_cast<RecurrentThread>(lowStatePuberThreadPtr)->Wait(1000000);
        lowStatePuberThreadPtr.reset();
    }

    CloseXsensIMU();

}

void PineappleSdk2Bridge::SetMotorToZero()
{
    for (int i = 0; i < num_motor_; i++)
    {
        CtrlOf(i)->set_zero_position(*CtrlOf(i)->getMotor(can_id_list[i]));
    }
}

void PineappleSdk2Bridge::SendJointCommand(int i, const JointCmd &c)
{
    auto motor = CtrlOf(i)->getMotor(can_id_list[i]);
    uint8_t err = motor->Get_ERR();
    if (err > 1) return;

    double cur_pos = (motor->Get_Position() - motor_offset[i]) * direction[i];

    // Only snap the command to the limit once both the current joint pos and
    // the incoming cmd have reached the same side of the constraint, so cmds
    // that move away from the limit are still passed through unclamped.
    double joint_cmd_q = c.q;
    if (cur_pos >= pos_limit_[i].upper && c.q >= pos_limit_[i].upper)
    {
        joint_cmd_q = pos_limit_[i].upper;
    }
    else if (cur_pos <= pos_limit_[i].lower && c.q <= pos_limit_[i].lower)
    {
        joint_cmd_q = pos_limit_[i].lower;
    }

    double cmd_q = joint_cmd_q * direction[i] + motor_offset[i];
    double cmd_dq = c.dq * direction[i];
    double cmd_tau = c.tau * direction[i];

    CtrlOf(i)->control_mit(*motor, c.kp, c.kd, cmd_q, cmd_dq, cmd_tau);
}

void PineappleSdk2Bridge::LowCmdGoHandler(const void *msg)
{
    if (!is_running) return;
    const uint32_t entry_us = Mono32(std::chrono::steady_clock::now());
    const unitree_go::msg::dds_::LowCmd_ *cmd = (const unitree_go::msg::dds_::LowCmd_ *)msg;
    const uint32_t state_tick = num_motor_ <= LAT_SLOT_B ? cmd->motor_cmd()[LAT_SLOT_B].reserve()[0] : 0;

    if (cmd_repeat_hz_ > 0)
    {
        // Store only; the control thread sends it (now, then repeatedly) and stamps exit.
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            for (int i = 0; i < num_motor_; i++)
            {
                const auto &m = cmd->motor_cmd()[i];
                cmd_target_[i] = JointCmd{m.kp(), m.kd(), m.q(), m.dq(), m.tau()};
            }
            cmd_valid_ = true;
            cmd_fresh_ = true;
            cmd_received_ = std::chrono::steady_clock::now();
            pending_entry_ = entry_us;
            pending_seq_ = cmd->reserve();
            pending_state_tick_ = state_tick;
        }
        cmd_cv_.notify_one();
        return;
    }

    for (int i = 0; i < num_motor_; i++)
    {
        const auto &m = cmd->motor_cmd()[i];
        SendJointCommand(i, JointCmd{m.kp(), m.kd(), m.q(), m.dq(), m.tau()});
    }
    // Latency instrumentation: handler entry/exit, and echoes of the deploy's stamps
    // (LowCmd_.reserve = deploy sequence; spare slot reserve[0] = LowState tick it used).
    lat_cmd_entry_ = entry_us;
    lat_cmd_exit_ = Mono32(std::chrono::steady_clock::now());
    lat_cmd_seq_ = cmd->reserve();
    lat_cmd_state_tick_ = state_tick;
    ++lat_cmd_count_;
}

bool PineappleSdk2Bridge::AllMotorsReplied(const vector<uint64_t> &seq_before) const
{
    for (int i = 0; i < num_motor_; i++)
    {
        auto motor = CtrlOf(i)->getMotor(can_id_list[i]);
        if (!IsMotorConnected(i) || motor->Get_ERR() > 1) continue;
        if (motor->GetFeedback().sequence == seq_before[i]) return false;
    }
    return true;
}

void PineappleSdk2Bridge::ControlLoop()
{
    using clock = std::chrono::steady_clock;
    if (rt_priority_ > 0)
    {
        sched_param sp{};
        sp.sched_priority = rt_priority_;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
            std::cerr << "[ctrl] could not set SCHED_FIFO " << rt_priority_ << " (run with sudo); using normal priority" << std::endl;
    }
    // Wake this thread on every motor reply instead of polling for them.
    for (auto &ctrl : motor_controls_)
        ctrl->setFeedbackListener([this] {
            { std::lock_guard<std::mutex> lock(reply_mutex_); }
            reply_cv_.notify_one();
        });

    const auto period = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(1.0 / cmd_repeat_hz_));
    const auto reply_wait = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(reply_wait_sec_));
    const auto timeout = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(cmd_timeout_sec_));

    vector<JointCmd> target(num_motor_);
    vector<uint64_t> seq_before(num_motor_);
    bool repeating = false;
    clock::time_point last_send{};
    bool fresh_since_tick = false;

    // Send every target and wait (sleeping) until all motors have replied or the deadline.
    auto send_and_wait = [&](clock::time_point start) {
        for (int i = 0; i < num_motor_; i++)
            seq_before[i] = CtrlOf(i)->getMotor(can_id_list[i])->GetFeedback().sequence;
        for (int i = 0; i < num_motor_; i++)
            SendJointCommand(i, target[i]);
        last_send = clock::now();
        std::unique_lock<std::mutex> lock(reply_mutex_);
        return reply_cv_.wait_until(lock, start + reply_wait,
                                    [&] { return AllMotorsReplied(seq_before); });
    };

    // Stats, printed every 5 s.
    uint64_t stat_sends = 0, stat_incomplete = 0, stat_fresh = 0, stat_pub = 0;
    double stat_wait_sum = 0, stat_wait_max = 0;
    auto stat_start = clock::now();
    timespec cpu0{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu0);

    // Lowstate goes out on a fixed grid at cmd_repeat_hz_. A new lowcmd is sent the moment
    // it arrives (off-grid, nothing published). A grid tick re-sends only if no new lowcmd
    // went out since the previous tick, so the bus carries at most cmd_repeat_hz_ sends/s
    // (one send of all motors takes ~1 ms over USB, so fresh + repeat could saturate it).
    auto next = clock::now() + period;
    while (is_running)
    {
        bool fresh = false, valid = false;
        uint32_t entry = 0, seq = 0, state_tick = 0;
        {
            std::unique_lock<std::mutex> lock(cmd_mutex_);
            cmd_cv_.wait_until(lock, next, [this] { return cmd_fresh_ || !is_running; });
            if (!is_running) break;
            fresh = cmd_fresh_;
            cmd_fresh_ = false;
            valid = cmd_valid_ && clock::now() - cmd_received_ < timeout;
            if (valid) target = cmd_target_;
            entry = pending_entry_;
            seq = pending_seq_;
            state_tick = pending_state_tick_;
        }
        const auto start = clock::now();
        const bool on_grid = start >= next;

        if (valid != repeating) {
            std::cerr << (valid ? "[ctrl] lowcmd received: sending at "
                                : "[ctrl] no lowcmd for timeout: stopped repeating, polling at ")
                      << (valid ? cmd_repeat_hz_ : static_cast<int>(1 / poll_interval_sec_)) << " Hz" << std::endl;
            repeating = valid;
        }

        if (valid && (fresh || (on_grid && !fresh_since_tick)))
        {
            const bool complete = send_and_wait(start);
            if (fresh)
            {
                lat_cmd_entry_ = entry;
                lat_cmd_exit_ = Mono32(last_send);
                lat_cmd_seq_ = seq;
                lat_cmd_state_tick_ = state_tick;
                ++lat_cmd_count_;
                ++stat_fresh;
            }
            const double wait_ms = std::chrono::duration<double, std::milli>(clock::now() - start).count();
            ++stat_sends;
            stat_incomplete += !complete;
            stat_wait_sum += wait_ms;
            stat_wait_max = std::max(stat_wait_max, wait_ms);
        }
        else if (!valid)
        {
            PollStaleMotors();
        }

        if (!on_grid) {  // off-grid wake-up for a new lowcmd: sent, not published
            fresh_since_tick = fresh_since_tick || (valid && fresh);
            continue;
        }
        fresh_since_tick = false;
        next += period;
        if (next < clock::now()) next = clock::now() + period;  // fell behind: no burst
        FillAndWriteLowState();
        ++stat_pub;

        const double stat_elapsed = std::chrono::duration<double>(clock::now() - stat_start).count();
        if (stat_elapsed >= 5.0)
        {
            timespec cpu1{};
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu1);
            const double thread_cpu = (cpu1.tv_sec - cpu0.tv_sec) + 1e-9 * (cpu1.tv_nsec - cpu0.tv_nsec);
            std::cout << "[ctrl] publish_hz=" << stat_pub / stat_elapsed
                      << " send_hz=" << stat_sends / stat_elapsed
                      << " lowcmd_hz=" << stat_fresh / stat_elapsed
                      << " send_to_all_replies_ms mean/max="
                      << (stat_sends ? stat_wait_sum / stat_sends : 0) << "/" << stat_wait_max
                      << " cycles_missing_a_reply=" << stat_incomplete << "/" << stat_sends
                      << " ctrl_thread_cpu=" << 100 * thread_cpu / stat_elapsed << "%"
                      << " bridge_cpu=" << ProcessCpuPercent() << "%" << std::endl;
            stat_sends = stat_incomplete = stat_fresh = stat_pub = 0;
            stat_wait_sum = stat_wait_max = 0;
            stat_start = clock::now();
            cpu0 = cpu1;
        }
    }
    for (auto &ctrl : motor_controls_)
        ctrl->setFeedbackListener(nullptr);
}

// Whole-process CPU use (all threads, % of one core) since the previous call.
double PineappleSdk2Bridge::ProcessCpuPercent()
{
    timespec cpu{};
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu);
    const auto now = std::chrono::steady_clock::now();
    const double c = cpu.tv_sec + 1e-9 * cpu.tv_nsec;
    double pct = 0;
    if (cpu_stat_wall_.time_since_epoch().count() != 0)
        pct = 100 * (c - cpu_stat_proc_) / std::chrono::duration<double>(now - cpu_stat_wall_).count();
    cpu_stat_proc_ = c;
    cpu_stat_wall_ = now;
    return pct;
}

void PineappleSdk2Bridge::PollStaleMotors()
{
    auto poll_now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(poll_now - last_poll_time_).count() >= poll_interval_sec_)
    {
        last_poll_time_ = poll_now;
        for (int i = 0; i < num_motor_; i++)
        {
            auto motor = CtrlOf(i)->getMotor(can_id_list[i]);
            if (motor->GetTimeSinceLastFeedback() >= poll_interval_sec_)
            {
                CtrlOf(i)->refresh_motor_status(*motor);
            }
        }
    }
}

void PineappleSdk2Bridge::PublishLowStateGo()
{
    if (!is_running) return;
    PollStaleMotors();
    FillAndWriteLowState();
    if (++legacy_publish_count_ % 2500 == 0)  // every 5 s at 500 Hz
        std::cout << "[bridge] bridge_cpu=" << ProcessCpuPercent() << "%" << std::endl;
}

void PineappleSdk2Bridge::FillAndWriteLowState()
{
    for (uint16_t i = 0;i < num_motor_; i++)
    {
        auto motor = CtrlOf(i)->getMotor(can_id_list[i]);
        const auto feedback = motor->GetFeedback();
        double pos = feedback.q;
        double vel = feedback.dq;
        double tau = feedback.tau;
        uint8_t err = feedback.error;

        low_state_go_.motor_state()[i].q() = (pos - motor_offset[i]) * direction[i];
        low_state_go_.motor_state()[i].dq() = vel * direction[i];
        low_state_go_.motor_state()[i].tau_est() = tau * direction[i];
        low_state_go_.motor_state()[i].mode() = err;
        low_state_go_.motor_state()[i].temperature() = static_cast<uint8_t>(feedback.t_mos);
        // Latency instrumentation: when this motor's state was received / last commanded,
        // and its feedback sequence (changes => fresh reply).
        low_state_go_.motor_state()[i].reserve()[0] = Mono32(motor->GetLastFeedbackTime());
        low_state_go_.motor_state()[i].reserve()[1] = Mono32(motor->GetLastCommandTime());
        low_state_go_.motor_state()[i].lost() = static_cast<uint32_t>(feedback.sequence);

        if (!IsMotorConnected(i)) {
            auto now = std::chrono::steady_clock::now();
            double since_init = std::chrono::duration<double>(now - init_time_).count();
            double log_elapsed = std::chrono::duration<double>(now - last_disconnect_log_time_[i]).count();
            // Stay quiet during the startup grace window so motors have time to
            // report in before we declare them disconnected.
            if (since_init >= init_grace_sec_ && log_elapsed >= 2.0) {
                last_disconnect_log_time_[i] = now;
                std::cerr << "[Motor " << i << " CAN_ID=0x" << std::hex << can_id_list[i]
                          << std::dec << "] not connected (no feedback received)" << std::endl;
            }
        } else if (err > 1) {
            HandleMotorFault(i);
        }
    }
    
    if (have_imu_)
    {
        // One snapshot, so quaternion components can never be mixed across
        // two different device samples.
        const ImuSample imu = xsens_imu_data->load();

        for (int i = 0; i < 4; i++) {
            low_state_go_.imu_state().quaternion()[i] = imu.quaternion_out[i];
        }
        for (int i = 0; i < 3; i++) {
            low_state_go_.imu_state().gyroscope()[i] = imu.gyro[i];
            low_state_go_.imu_state().accelerometer()[i] = imu.accel[i];
        }
        // Latency instrumentation: host arrival of the current quaternion / gyro, and the
        // quaternion's device SampleTimeFine. Spare slot only.
        if (num_motor_ <= LAT_SLOT_B)
        {
            auto &a = low_state_go_.motor_state()[LAT_SLOT_A];
            a.reserve()[0] = Mono32Seconds(imu.quaternion_out_at);  // == quaternion arrival unless propagating
            a.reserve()[1] = Mono32Seconds(imu.received_at[1]);
            a.lost() = imu.quat_sample_time_fine;
        }
    }
    if (num_motor_ <= LAT_SLOT_B)
    {
        auto &b = low_state_go_.motor_state()[LAT_SLOT_B];
        b.reserve()[0] = lat_cmd_entry_.load();
        b.reserve()[1] = lat_cmd_exit_.load();
        b.lost() = lat_cmd_seq_.load();
        b.mode() = static_cast<uint8_t>(lat_cmd_count_.load() & 0xFF);  // lowcmd counter (mod 256)
        low_state_go_.reserve() = lat_cmd_state_tick_.load();
    }
    low_state_go_.tick() = Mono32(std::chrono::steady_clock::now());
    low_state_go_puber_->Write(low_state_go_);

}

bool PineappleSdk2Bridge::InitXsensIMU()
{
    ImuOutputProfile profile;
    try {
        profile = imuOutputProfileFromEnvironment();
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return false;
    }
    xsens_control = XsControl::construct();
    XsPortInfoArray xsens_portInfoArray = XsScanner::scanPorts();
    for (auto const &portInfo : xsens_portInfoArray)
    {
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
        {
            xsens_mtPort = portInfo;
            break;
        }
    }
    if (xsens_mtPort.empty()) {
        std::cerr << "No MTi device found." << std::endl;
        return false;
    }
    if (!xsens_control->openPort(xsens_mtPort.portName().toStdString(), xsens_mtPort.baudrate())) {
        std::cerr << "Could not open IMU port." << std::endl;
        return false;
    }
    // Get the device object
    cout << "Found a device with ID: " << xsens_mtPort.deviceId().toString().toStdString() << " @ port: " << xsens_mtPort.portName().toStdString() << ", baudrate: " << XsBaud::rateToNumeric(xsens_mtPort.baudrate()) << endl;
	xsens_device = xsens_control->device(xsens_mtPort.deviceId());
	assert(xsens_device != 0);

	cout << "Device: " << xsens_device->productCode().toStdString() << ", with ID: " << xsens_device->deviceId().toString() << " opened." << endl;

    const bool propagate = imuPropagateFromEnvironment();
    if (propagate && profile.hz[1] <= profile.hz[0])
        std::cerr << "[imu] PINEAPPLE_IMU_PROPAGATE=1 has no effect: gyro rate " << profile.hz[1]
                  << " Hz is not above the quaternion rate " << profile.hz[0] << " Hz" << std::endl;
    std::cout << "[imu] profile q/gyro/accel_hz=" << profile.hz[0] << "/" << profile.hz[1] << "/"
              << profile.hz[2] << " propagate=" << (propagate ? "on" : "off") << std::endl;
    xsens_imu_data->setPropagate(propagate);
    xsens_callback.setTarget(xsens_imu_data);
    xsens_device->addCallbackHandler(&xsens_callback);
    if (!configureXsens(*xsens_device, profile, xsens_mtPort.baudrate())) {
        std::cerr << "IMU configuration/readback failed." << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (skip_imu_timing_check_) {
        std::cerr << "[imu] Startup timing check disabled for this run; "
                     "valid quaternion and incoming fields are still required.\n";
    }
    if (!verifyImuDelivery(*xsens_imu_data, profile, 1.0, skip_imu_timing_check_)) {
        std::cerr << "IMU delivery check failed." << std::endl;
        return false;
    }
    return true;
}

bool PineappleSdk2Bridge::IsMotorConnected(int motor_idx) const
{
    auto motor = CtrlOf(motor_idx)->getMotor(can_id_list[motor_idx]);
    return motor->GetTimeSinceLastFeedback() < feedback_timeout_sec_;
}

void PineappleSdk2Bridge::HandleMotorFault(int i)
{
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - last_fault_recovery_time_[i]).count();
    if (elapsed < 2.0) {
        return;
    }
    last_fault_recovery_time_[i] = now;

    uint8_t err = CtrlOf(i)->getMotor(can_id_list[i])->Get_ERR();
    std::cerr << "[Motor " << i << " CAN_ID=0x" << std::hex << can_id_list[i]
              << "] fault ERR=0x" << static_cast<int>(err) << std::dec
              << " — sending clear+enable" << std::endl;

    CtrlOf(i)->control_cmd(can_id_list[i], 0xFB);
    usleep(5000);
    CtrlOf(i)->control_cmd(can_id_list[i], 0xFC);
}

void PineappleSdk2Bridge::CloseXsensIMU()
{
    if (xsens_control == nullptr) return;
    if (!xsens_mtPort.empty()) {
        xsens_control->closePort(xsens_mtPort.portName().toStdString());
    }
    xsens_control->destruct();
    xsens_control = nullptr;
}

