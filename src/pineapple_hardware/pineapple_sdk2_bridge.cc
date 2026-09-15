#include "pineapple_sdk2_bridge.h"

#include <algorithm>

PineappleSdk2Bridge::PineappleSdk2Bridge(const vector<MotorConfig> &platform_configs)
{

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

    xsens_imu_data = std::make_shared<ImuSharedData>();
    last_fault_recovery_time_.resize(num_motor_, std::chrono::steady_clock::now());
    last_disconnect_log_time_.resize(num_motor_, std::chrono::steady_clock::now());

    low_cmd_go_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    low_cmd_go_suber_->InitChannel(bind(&PineappleSdk2Bridge::LowCmdGoHandler, this, placeholders::_1), 1);

    low_state_go_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    low_state_go_puber_->InitChannel();
    
    // The MTi is a robot-level device, not a per-platform one: any config that
    // declares it turns it on, so argument order cannot change whether
    // orientation gets published.
    have_imu_ = std::any_of(platform_configs.begin(), platform_configs.end(),
                            [](const MotorConfig &cfg) { return cfg.have_imu; });
    if (have_imu_) {

        have_imu_ = InitXsensIMU();
        if (have_imu_) {
            xsens_imu_thread = std::thread(&PineappleSdk2Bridge::ProcessXsensData, this);
        } else {
            std::cerr << "IMU requested in config but could not be started; "
                      << "orientation will not be published." << std::endl;
        }
    }
    if (!have_imu_) {
        // Publish a valid identity rather than the IDL's default {0,0,0,0},
        // which NaNs any consumer that normalizes the quaternion.
        low_state_go_.imu_state().quaternion()[0] = 1.0f;
    }

    init_time_ = std::chrono::steady_clock::now();
    last_poll_time_ = init_time_;
    lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &PineappleSdk2Bridge::PublishLowStateGo, this);

}

PineappleSdk2Bridge::~PineappleSdk2Bridge()
{
    is_running = false;


    if (low_cmd_go_suber_) {
        low_cmd_go_suber_->CloseChannel();
    }

    if (lowStatePuberThreadPtr) {
        std::static_pointer_cast<RecurrentThread>(lowStatePuberThreadPtr)->Wait(1000000);
        lowStatePuberThreadPtr.reset();
    }

    if (have_imu_) {
        xsens_imu_thread.join();
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

void PineappleSdk2Bridge::LowCmdGoHandler(const void *msg)
{
    if (!is_running) return;
    const unitree_go::msg::dds_::LowCmd_ *cmd = (const unitree_go::msg::dds_::LowCmd_ *)msg;
    for (int i = 0; i < num_motor_; i++)
    {
        auto motor = CtrlOf(i)->getMotor(can_id_list[i]);
        uint8_t err = motor->Get_ERR();
        if (err > 1) continue;

        double raw_cmd_q = cmd->motor_cmd()[i].q();
        double cur_pos = (motor->Get_Position() - motor_offset[i]) * direction[i];

        // Only snap the command to the limit once both the current joint pos and
        // the incoming cmd have reached the same side of the constraint, so cmds
        // that move away from the limit are still passed through unclamped.
        double joint_cmd_q = raw_cmd_q;
        if (cur_pos >= pos_limit_[i].upper && raw_cmd_q >= pos_limit_[i].upper)
        {
            joint_cmd_q = pos_limit_[i].upper;
        }
        else if (cur_pos <= pos_limit_[i].lower && raw_cmd_q <= pos_limit_[i].lower)
        {
            joint_cmd_q = pos_limit_[i].lower;
        }

        double cmd_q = joint_cmd_q * direction[i] + motor_offset[i];
        double cmd_dq = cmd->motor_cmd()[i].dq() * direction[i];
        double cmd_tau = cmd->motor_cmd()[i].tau() * direction[i];

        CtrlOf(i)->control_mit(*motor, cmd->motor_cmd()[i].kp(), cmd->motor_cmd()[i].kd(),
                                                            cmd_q, cmd_dq, cmd_tau);
    }
}

void PineappleSdk2Bridge::PublishLowStateGo()
{
    if (!is_running) return;

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

    for (uint16_t i = 0;i < num_motor_; i++)
    {
        auto motor = CtrlOf(i)->getMotor(can_id_list[i]);
        int motor_type = motor->GetMotorMode();
        double pos = motor->Get_Position();
        double vel = motor->Get_Velocity();
        double tau = motor->Get_tau();
        uint8_t err = motor->Get_ERR();

        low_state_go_.motor_state()[i].q() = (pos - motor_offset[i]) * direction[i];
        low_state_go_.motor_state()[i].dq() = vel * direction[i];
        low_state_go_.motor_state()[i].tau_est() = tau * direction[i];
        low_state_go_.motor_state()[i].mode() = err;
        low_state_go_.motor_state()[i].temperature() = static_cast<uint8_t>(motor->Get_T_MOS());

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
            low_state_go_.imu_state().quaternion()[i] = imu.quaternion[i];
        }
        for (int i = 0; i < 3; i++) {
            low_state_go_.imu_state().gyroscope()[i] = imu.gyro[i];
            low_state_go_.imu_state().accelerometer()[i] = imu.accel[i];
        }
    }
    low_state_go_puber_->Write(low_state_go_);

}

bool PineappleSdk2Bridge::InitXsensIMU()
{
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
        std::cerr << "No MTi device found. IMU thread exiting." << std::endl;
        return false;
    }
    if (!xsens_control->openPort(xsens_mtPort.portName().toStdString(), xsens_mtPort.baudrate())) {
        std::cerr << "Could not open IMU port. IMU thread exiting." << std::endl;
        return false;
    }
    // Get the device object
    cout << "Found a device with ID: " << xsens_mtPort.deviceId().toString().toStdString() << " @ port: " << xsens_mtPort.portName().toStdString() << ", baudrate: " << xsens_mtPort.baudrate() << endl;
	xsens_device = xsens_control->device(xsens_mtPort.deviceId());
	assert(xsens_device != 0);

	cout << "Device: " << xsens_device->productCode().toStdString() << ", with ID: " << xsens_device->deviceId().toString() << " opened." << endl;

    xsens_device->addCallbackHandler(&xsens_callback);
    if (!xsens_device->gotoConfig()) {
        std::cerr << "IMU refused to enter config mode." << std::endl;
        return false;
    }
    xsens_device->readEmtsAndDeviceConfiguration();
    XsOutputConfigurationArray configArray;
    configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
    if (xsens_device->deviceId().isVru() || xsens_device->deviceId().isAhrs())
	{

		configArray.push_back(XsOutputConfiguration(XDI_Quaternion, kImuOutputRateHz));
        configArray.push_back(XsOutputConfiguration(XDI_RateOfTurnHR, kImuOutputRateHz));
        configArray.push_back(XsOutputConfiguration(XDI_AccelerationHR, kImuOutputRateHz));
	}
    if (!xsens_device->setOutputConfiguration(configArray)) {
        std::cerr << "IMU rejected the output configuration." << std::endl;
        return false;
    }
    if (!xsens_device->gotoMeasurement()) {
        std::cerr << "IMU refused to enter measurement mode." << std::endl;
        return false;
    }
    return true;
}

void PineappleSdk2Bridge::ProcessXsensData()
{
    // Carried across packets so a field missing from one packet keeps its last
    // value instead of reverting.
    ImuSample sample = xsens_imu_data->load();

    uint16_t prev_counter = 0;
    bool have_prev_counter = false;
    uint64_t received = 0, dropped = 0, orientation_updates = 0;
    auto last_report = std::chrono::steady_clock::now();

    while (is_running) {

        while (xsens_callback.packetAvailable()) {
            XsDataPacket packet = xsens_callback.getNextPacket();
            ++received;

            // XDI_PacketCounter is already requested in InitXsensIMU(); read it
            // so loss is measured here rather than inferred from offline logs.
            if (packet.containsPacketCounter()) {
                const uint16_t counter = packet.packetCounter();
                if (have_prev_counter) {

                    const uint16_t gap = static_cast<uint16_t>(counter - prev_counter);
                    if (gap > 1) dropped += gap - 1;
                }
                prev_counter = counter;
                have_prev_counter = true;
            }

            if (packet.containsOrientation()) {
                XsQuaternion q = packet.orientationQuaternion();
                sample.quaternion[0] = q.w();
                sample.quaternion[1] = q.x();
                sample.quaternion[2] = q.y();
                sample.quaternion[3] = q.z();
                XsEuler euler = packet.orientationEuler();
                sample.rpy[0] = euler.roll();
                sample.rpy[1] = euler.pitch();
                sample.rpy[2] = euler.yaw();
            }
            if (packet.containsRateOfTurnHR()) {
                XsVector gyr_hr = packet.rateOfTurnHR();
                for (int i = 0; i < 3; ++i) {
                    sample.gyro[i] = gyr_hr[i];
                }
            }
            if (packet.containsAccelerationHR()) {
                XsVector acc_hr = packet.accelerationHR();
                for (int i = 0; i < 3; ++i) {
                    sample.accel[i] = acc_hr[i];
                }
            }

            xsens_imu_data->store(sample);
        }


        XsTime::msleep(1);
    }
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

