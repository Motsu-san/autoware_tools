// Copyright 2026 scripts_for_autoware contributors
// SPDX-License-Identifier: Apache-2.0

#include "ndt_direct_measure/pose_yaml.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

namespace ndt_direct_measure
{

geometry_msgs::msg::Pose load_pose_from_ndt_start_yaml(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to load yaml: " + yaml_path + ": " + e.what());
  }

  const YAML::Node pose_node = root["pose"]["pose"];
  if (!pose_node) {
    throw std::runtime_error("Missing pose.pose in yaml: " + yaml_path);
  }

  geometry_msgs::msg::Pose pose;
  pose.position.x = pose_node["position"]["x"].as<double>();
  pose.position.y = pose_node["position"]["y"].as<double>();
  pose.position.z = pose_node["position"]["z"].as<double>();
  pose.orientation.x = pose_node["orientation"]["x"].as<double>();
  pose.orientation.y = pose_node["orientation"]["y"].as<double>();
  pose.orientation.z = pose_node["orientation"]["z"].as<double>();
  pose.orientation.w = pose_node["orientation"]["w"].as<double>();
  return pose;
}

}  // namespace ndt_direct_measure
