// Copyright 2026 scripts_for_autoware contributors
// SPDX-License-Identifier: Apache-2.0

#include "ndt_direct_measure/pose_extrapolation.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <limits>
#include <optional>

namespace ndt_direct_measure
{
namespace
{

constexpr double kMinStampDeltaSec = 1e-6;

double normalize_angle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

double yaw_from_pose(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion q(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion quat_from_rpy(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

}  // namespace

bool is_good_alignment_result(const ScanMatcherResult & result)
{
  return result.has_converged && result.passes_score_threshold;
}

geometry_msgs::msg::Pose mean_pose_from_runs(const std::vector<ScanMatcherResult> & runs)
{
  geometry_msgs::msg::Pose out;
  if (runs.empty()) {
    return out;
  }

  std::vector<const ScanMatcherResult *> selected;
  selected.reserve(runs.size());
  for (const auto & run : runs) {
    if (is_good_alignment_result(run)) {
      selected.push_back(&run);
    }
  }
  if (selected.empty()) {
    for (const auto & run : runs) {
      selected.push_back(&run);
    }
  }

  double sx = 0.0;
  double sy = 0.0;
  double sz = 0.0;
  double sin_yaw = 0.0;
  double cos_yaw = 0.0;
  double roll = 0.0;
  double pitch = 0.0;
  bool roll_pitch_set = false;
  for (const auto * run : selected) {
    const auto & p = run->scan_matching_pose;
    sx += p.position.x;
    sy += p.position.y;
    sz += p.position.z;
    const double yaw = yaw_from_pose(p);
    sin_yaw += std::sin(yaw);
    cos_yaw += std::cos(yaw);
    if (!roll_pitch_set) {
      tf2::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
      double yaw_tmp = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_tmp);
      roll_pitch_set = true;
    }
  }
  const double n = static_cast<double>(selected.size());
  out.position.x = sx / n;
  out.position.y = sy / n;
  out.position.z = sz / n;
  const double mean_yaw = std::atan2(sin_yaw / n, cos_yaw / n);
  out.orientation = quat_from_rpy(roll, pitch, mean_yaw);
  return out;
}

OffsetAnchor make_offset_anchor(const ScanGroupResult & group)
{
  OffsetAnchor anchor;
  anchor.stamp_sec = group.cloud_sel.header_stamp_sec;
  anchor.mean_pose = mean_pose_from_runs(group.runs);
  for (const auto & run : group.runs) {
    if (is_good_alignment_result(run)) {
      anchor.usable_for_extrapolation = true;
      break;
    }
  }
  return anchor;
}

std::optional<ExtrapolatedInitialPose> extrapolate_initial_pose(
  const OffsetAnchor & anchor_a, const OffsetAnchor & anchor_b, double target_stamp_sec)
{
  if (!anchor_a.usable_for_extrapolation || !anchor_b.usable_for_extrapolation) {
    return std::nullopt;
  }

  const double dt_ab = anchor_b.stamp_sec - anchor_a.stamp_sec;
  if (std::abs(dt_ab) < kMinStampDeltaSec) {
    return std::nullopt;
  }

  const double alpha = (target_stamp_sec - anchor_b.stamp_sec) / dt_ab;

  const auto & pa = anchor_a.mean_pose;
  const auto & pb = anchor_b.mean_pose;

  ExtrapolatedInitialPose out;
  out.extrapolated = true;
  out.alpha = alpha;
  out.pose.position.x = pb.position.x + alpha * (pb.position.x - pa.position.x);
  out.pose.position.y = pb.position.y + alpha * (pb.position.y - pa.position.y);
  out.pose.position.z = pb.position.z + alpha * (pb.position.z - pa.position.z);

  const double yaw_a = yaw_from_pose(pa);
  const double yaw_b = yaw_from_pose(pb);
  const double dyaw = normalize_angle(yaw_b - yaw_a);
  tf2::Quaternion qb(
    pb.orientation.x, pb.orientation.y, pb.orientation.z, pb.orientation.w);
  double roll{0.0};
  double pitch{0.0};
  double yaw_unused{0.0};
  tf2::Matrix3x3(qb).getRPY(roll, pitch, yaw_unused);
  out.pose.orientation = quat_from_rpy(roll, pitch, yaw_b + alpha * dyaw);
  return out;
}

}  // namespace ndt_direct_measure
