
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

std::atomic<bool> running(true);

void signalHandler(int signum) {
    running = false;
    std::cerr << "\nInterrupt signal (" << signum << ") received.\n";
}

Journaller* gJournal = 0; // Xsens IMU related

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
    std::unique_ptr<PineappleSdk2Bridge> pineapple_interface;
    try {
        pineapple_interface = std::make_unique<PineappleSdk2Bridge>(platform_configs);
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    while (running)
    {
        sleep(2);
    }

    return 0;
}
