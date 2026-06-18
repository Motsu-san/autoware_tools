// Copyright 2026 scripts_for_autoware contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef NDT_DIRECT_MEASURE__POSE_EXTRAPOLATION_HPP_
#define NDT_DIRECT_MEASURE__POSE_EXTRAPOLATION_HPP_

#include "ndt_direct_measure/types.hpp"

#include <optional>

namespace ndt_direct_measure
{

struct OffsetAnchor
{
  bool usable_for_extrapolation{false};
  geometry_msgs::msg::Pose mean_pose{};
  double stamp_sec{0.0};
};

struct ExtrapolatedInitialPose
{
  geometry_msgs::msg::Pose pose{};
  bool extrapolated{false};
  int ref_offset_a{0};
  int ref_offset_b{0};
  double alpha{0.0};
};

bool is_good_alignment_result(const ScanMatcherResult & result);

OffsetAnchor make_offset_anchor(const ScanGroupResult & group);

geometry_msgs::msg::Pose mean_pose_from_runs(const std::vector<ScanMatcherResult> & runs);

/** P_target = P_b + alpha * (P_b - P_a), alpha = (t_target - t_b) / (t_b - t_a) */
std::optional<ExtrapolatedInitialPose> extrapolate_initial_pose(
  const OffsetAnchor & anchor_a, const OffsetAnchor & anchor_b, double target_stamp_sec);

}  // namespace ndt_direct_measure

#endif  // NDT_DIRECT_MEASURE__POSE_EXTRAPOLATION_HPP_
