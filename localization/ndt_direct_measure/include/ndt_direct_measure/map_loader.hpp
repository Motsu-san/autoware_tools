// Copyright 2026 scripts_for_autoware contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef NDT_DIRECT_MEASURE__MAP_LOADER_HPP_
#define NDT_DIRECT_MEASURE__MAP_LOADER_HPP_

#include "ndt_direct_measure/types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace ndt_direct_measure
{

MapLoadInfo load_map_into_ndt(
  NormalDistributionsTransform & registration, const std::string & map_path,
  const std::string & map_load_mode, double center_x, double center_y, double map_radius_m,
  const std::string & metadata_yaml_path, rclcpp::Logger logger);

}  // namespace ndt_direct_measure

#endif  // NDT_DIRECT_MEASURE__MAP_LOADER_HPP_
