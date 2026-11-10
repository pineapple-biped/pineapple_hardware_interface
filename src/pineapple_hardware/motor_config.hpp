#ifndef PINEAPPLE_MOTOR_CONFIG_HPP
#define PINEAPPLE_MOTOR_CONFIG_HPP

#include <cstdint>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "../canfd/include/damiao.h"

struct MotorPosLimit {
    double lower;
    double upper;
};

// One platform = one USB2CANFD device (e.g. the wheel biped or the arm).
struct MotorConfig {
    std::string dev_sn;  // USB2CANFD serial number, from scan_canfd_sn
    bool set_zero = false;
    bool have_imu = false;  // bring up the Xsens MTi and publish its orientation
    std::vector<uint16_t> can_id_list;
    std::vector<uint16_t> mst_id_list;
    std::vector<int> motor_type;
    std::vector<double> motor_offset;
    std::vector<double> direction;
    std::vector<MotorPosLimit> pos_limit;
};

inline const std::unordered_map<std::string, int> kMotorTypeByName = {
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

// Load one platform (one USB2CANFD device) from its config yaml.
// Returns false on any config error.
inline bool LoadMotorConfig(const std::string &config_path, MotorConfig &motor_config)
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

#endif
