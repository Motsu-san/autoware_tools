// Copyright 2026 scripts_for_autoware contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef NDT_DIRECT_MEASURE__BAG_READER_HPP_
#define NDT_DIRECT_MEASURE__BAG_READER_HPP_

#include "ndt_direct_measure/types.hpp"

#include <string>

namespace ndt_direct_measure
{

PointCloudSelection find_nearest_pointcloud(
  const std::string & bag_path, const std::string & topic, double target_unix_sec);

/** target に最も近い点群と、その前後 neighbor_count 件（時系列順）を返す。 */
std::vector<PointCloudSelection> find_pointclouds_around_target(
  const std::string & bag_path, const std::string & topic, double target_unix_sec,
  int neighbor_count);

}  // namespace ndt_direct_measure

#endif  // NDT_DIRECT_MEASURE__BAG_READER_HPP_
