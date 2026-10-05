
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <csignal>
#include <unordered_map>
#include <vector>

#include "pineapple_hardware/pineapple_sdk2_bridge.h"
#include <pthread.h>
#include "yaml-cpp/yaml.h"

struct RealRobotConfig
  {
    int domain_id = 1;
    std::string interface = "lo";

  } config;

static const std::unordered_map<std::string, int> kMotorTypeByName = {
    {"DM3507", damiao::DM3507},
    {"DM4310", damiao::DM4310},
    {"DM4310_48V", damiao::DM4310_48V},
    {"DM4340", damiao::DM4340},
    {"DM4340_48V", damiao::DM4340_48V},
    {"DM6006", damiao::DM6006},
    {"DM6248", damiao::DM6248},
    {"DM8006", damiao::DM8006},
    {"DM8009", damiao::DM8009},
    {"DM10010L", damiao::DM10010L},
    {"DM10010", damiao::DM10010},
    {"DMH3510", damiao::DMH3510},
    {"DMH6215", damiao::DMH6215},
    {"DMS3519", damiao::DMS3519},
    {"DMG6220", damiao::DMG6220},
};

std::atomic<bool> running(true);

void signalHandler(int signum) {
    running = false;
    std::cerr << "\nInterrupt signal (" << signum << ") received.\n";
}

Journaller* gJournal = 0; // Xsens IMU related

// Load one platform (one USB2CANFD device) from its config yaml.
// Returns false on any config error.
static bool LoadMotorConfig(const std::string &config_path, MotorConfig &motor_config)
{
    YAML::Node yaml_node = YAML::LoadFile(config_path);

    motor_config.dev_sn = yaml_node["dev_sn"] ? yaml_node["dev_sn"].as<std::string>() : "";
    if (motor_config.dev_sn.empty())
    {
        std::cerr << "Config error in " << config_path << ": dev_sn is missing or empty. "
                  << "Run ./scan_canfd_sn to list connected USB2CANFD serial numbers "
                  << "and fill the dev_sn field." << std::endl;
        return false;
    }

    motor_config.set_zero = yaml_node["set_zero"].as<bool>();
    // Optional: older configs predate the field, and a robot without the MTi
    // fitted should still come up.
    motor_config.have_imu = yaml_node["have_imu"] ? yaml_node["have_imu"].as<bool>() : false;
    motor_config.can_id_list = yaml_node["can_id"].as<std::vector<uint16_t>>();
    motor_config.mst_id_list = yaml_node["mst_id"].as<std::vector<uint16_t>>();
    for (const auto &type_node : yaml_node["motor_type"])
    {
        auto type_name = type_node.as<std::string>();
        auto it = kMotorTypeByName.find(type_name);
        if (it == kMotorTypeByName.end())
        {
            std::cerr << "Unknown motor_type \"" << type_name << "\" in " << config_path << std::endl;
            return false;
        }
        motor_config.motor_type.push_back(it->second);
    }
    motor_config.motor_offset = yaml_node["motor_offset"].as<std::vector<double>>();
    motor_config.direction = yaml_node["direction"].as<std::vector<double>>();
    for (const auto &limit : yaml_node["pos_limit"])
    {
        auto limit_pair = limit.as<std::vector<double>>();
        motor_config.pos_limit.push_back({limit_pair[0], limit_pair[1]});
    }

    size_t num_motor = motor_config.can_id_list.size();
    if (motor_config.mst_id_list.size() != num_motor ||
        motor_config.motor_type.size() != num_motor ||
        motor_config.motor_offset.size() != num_motor ||
        motor_config.direction.size() != num_motor ||
        motor_config.pos_limit.size() != num_motor)
    {
        std::cerr << "Config error in " << config_path << ": can_id, mst_id, motor_type, "
                  << "motor_offset, direction and pos_limit must all have the same length" << std::endl;
        return false;
    }
    return true;
}

int main(int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);

    // Each config file describes one platform on its own USB2CANFD device.
    // Joint index order in LowCmd/LowState follows the argument order
    // (e.g. wheel biped config first -> joints 0-7, arm config second -> joints 8-13).
    std::vector<std::string> config_paths;
    if (argc > 1)
    {
        for (int i = 1; i < argc; i++) config_paths.push_back(argv[i]);
    }
    else
    {
        config_paths.push_back("../config/config.yaml");
    }

    std::vector<MotorConfig> platform_configs;
    size_t joint_base = 0;
    for (const auto &path : config_paths)
    {
        MotorConfig motor_config;
        if (!LoadMotorConfig(path, motor_config))
        {
            return 1;
        }
        size_t n = motor_config.can_id_list.size();
        std::cout << "Platform " << platform_configs.size() << ": " << path
                  << ", joints " << joint_base << "-" << (joint_base + n - 1)
                  << ", dev_sn=" << motor_config.dev_sn << std::endl;
        joint_base += n;
        platform_configs.push_back(std::move(motor_config));
    }
    std::cout << "Total joints: " << joint_base << std::endl;

    // DDS domain settings come from the first config
    YAML::Node yaml_node = YAML::LoadFile(config_paths[0]);
    config.domain_id = yaml_node["domain_id"].as<int>();
    config.interface = yaml_node["interface"].as<std::string>();

    // Main function
    ChannelFactory::Instance()->Init(config.domain_id, config.interface);
    PineappleSdk2Bridge pineapple_interface(platform_configs);

    while (running)
    {
        sleep(2);
    }

    return 0;
}
