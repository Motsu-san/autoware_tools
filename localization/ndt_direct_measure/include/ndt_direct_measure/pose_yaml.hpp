// Copyright 2026 scripts_for_autoware contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef NDT_DIRECT_MEASURE__POSE_YAML_HPP_
#define NDT_DIRECT_MEASURE__POSE_YAML_HPP_

#include <geometry_msgs/msg/pose.hpp>

#include <string>

namespace ndt_direct_measure
{

geometry_msgs::msg::Pose load_pose_from_ndt_start_yaml(const std::string & yaml_path);

}  // namespace ndt_direct_measure

#endif  // NDT_DIRECT_MEASURE__POSE_YAML_HPP_
