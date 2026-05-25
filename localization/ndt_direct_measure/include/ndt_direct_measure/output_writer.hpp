// Copyright 2026 scripts_for_autoware contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef NDT_DIRECT_MEASURE__OUTPUT_WRITER_HPP_
#define NDT_DIRECT_MEASURE__OUTPUT_WRITER_HPP_

#include "ndt_direct_measure/types.hpp"

#include <string>
#include <vector>

namespace ndt_direct_measure
{

void write_csv(
  const std::string & path, const std::vector<ScanMatcherResult> & runs, double stamp_sec,
  const PointCloudSelection & cloud_sel, const MapLoadInfo & map_info);

void write_json(
  const std::string & path, const std::vector<ScanMatcherResult> & runs, double target_unix_sec,
  const std::string & source_bag, const std::string & initial_pose_yaml,
  const std::string & pointcloud_topic, const PointCloudSelection & cloud_sel,
  const MapLoadInfo & map_info, const geometry_msgs::msg::Pose & initial_pose);

}  // namespace ndt_direct_measure

#endif  // NDT_DIRECT_MEASURE__OUTPUT_WRITER_HPP_
