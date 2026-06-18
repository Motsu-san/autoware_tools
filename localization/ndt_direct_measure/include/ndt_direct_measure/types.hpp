// Copyright 2026 scripts_for_autoware contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef NDT_DIRECT_MEASURE__TYPES_HPP_
#define NDT_DIRECT_MEASURE__TYPES_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <autoware/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace ndt_direct_measure
{

using PointType = pcl::PointXYZ;
using NormalDistributionsTransform =
  pclomp::MultiGridNormalDistributionsTransform<PointType, PointType>;

struct ScanMatcherResult
{
  bool has_converged{false};
  bool passes_score_threshold{false};
  geometry_msgs::msg::Pose initial_pose{};
  geometry_msgs::msg::Pose scan_matching_pose{};
  double score_nvtl{0.0};
  double score_tp{0.0};
  int iteration{0};
  double score_threshold{0.0};
  std::string score_type{"nearest_voxel_transformation_likelihood"};
};

struct MapLoadInfo
{
  std::string mode;
  size_t pcd_file_count{0};
  size_t point_count{0};
  std::vector<std::string> loaded_cell_ids;
};

struct PointCloudSelection
{
  sensor_msgs::msg::PointCloud2 cloud;
  double header_stamp_sec{0.0};
  double dt_from_target_sec{0.0};
  std::string frame_id;
  /** 最近傍スキャンからの時系列オフセット（0=最近傍, -1=1つ前, +1=1つ後） */
  int offset_from_nearest{0};
};

struct ScanInitialPoseInfo
{
  geometry_msgs::msg::Pose pose{};
  std::string source{"ndt_start_pose_yaml"};
  bool extrapolated{false};
  int ref_offset_a{0};
  int ref_offset_b{0};
  double extrapolation_alpha{0.0};
};

struct ScanGroupResult
{
  PointCloudSelection cloud_sel;
  std::vector<ScanMatcherResult> runs;
  ScanInitialPoseInfo initial_pose_info{};
};

}  // namespace ndt_direct_measure

#endif  // NDT_DIRECT_MEASURE__TYPES_HPP_
