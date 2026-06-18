// Copyright 2026 scripts_for_autoware contributors
// SPDX-License-Identifier: Apache-2.0

#include "ndt_direct_measure/output_writer.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace ndt_direct_measure
{
namespace
{

std::string format_unix_sec(double t)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(9) << t;
  return oss.str();
}

std::string pose_to_json(const geometry_msgs::msg::Pose & p)
{
  std::ostringstream oss;
  oss << std::setprecision(10);
  oss << "{\"position\":{\"x\":" << p.position.x << ",\"y\":" << p.position.y << ",\"z\":"
      << p.position.z << "},\"orientation\":{\"x\":" << p.orientation.x << ",\"y\":"
      << p.orientation.y << ",\"z\":" << p.orientation.z << ",\"w\":" << p.orientation.w << "}}";
  return oss.str();
}

const PointCloudSelection * find_nearest_scan(const std::vector<ScanGroupResult> & scan_groups)
{
  for (const auto & group : scan_groups) {
    if (group.cloud_sel.offset_from_nearest == 0) {
      return &group.cloud_sel;
    }
  }
  return scan_groups.empty() ? nullptr : &scan_groups.front().cloud_sel;
}

void write_scan_group_json(
  std::ostringstream & oss, const ScanGroupResult & group, bool include_runs)
{
  const auto & cloud_sel = group.cloud_sel;
  oss << "    {\n";
  oss << "      \"offset_from_nearest\": " << cloud_sel.offset_from_nearest << ",\n";
  oss << "      \"pointcloud_header\": {\n";
  oss << "        \"stamp_sec\": " << format_unix_sec(cloud_sel.header_stamp_sec) << ",\n";
  oss << "        \"frame_id\": \"" << cloud_sel.frame_id << "\",\n";
  oss << "        \"dt_from_target_sec\": " << format_unix_sec(cloud_sel.dt_from_target_sec)
      << "\n";
  oss << "      },\n";
  oss << "      \"scan_initial_pose\": {\n";
  oss << "        \"source\": \"" << group.initial_pose_info.source << "\",\n";
  oss << "        \"extrapolated\": "
      << (group.initial_pose_info.extrapolated ? "true" : "false") << ",\n";
  oss << "        \"ref_offset_a\": " << group.initial_pose_info.ref_offset_a << ",\n";
  oss << "        \"ref_offset_b\": " << group.initial_pose_info.ref_offset_b << ",\n";
  oss << "        \"extrapolation_alpha\": "
      << format_unix_sec(group.initial_pose_info.extrapolation_alpha) << ",\n";
  oss << "        \"pose\": " << pose_to_json(group.initial_pose_info.pose) << "\n";
  oss << "      },\n";
  oss << "      \"n_runs\": " << group.runs.size();
  if (include_runs) {
    oss << ",\n      \"per_run\": [\n";
    for (size_t i = 0; i < group.runs.size(); ++i) {
      const auto & r = group.runs[i];
      if (i > 0) {
        oss << ",\n";
      }
      oss << "        {\n";
      oss << "          \"run_index\": " << i << ",\n";
      oss << "          \"has_converged\": " << (r.has_converged ? "true" : "false") << ",\n";
      oss << "          \"passes_score_threshold\": "
          << (r.passes_score_threshold ? "true" : "false") << ",\n";
      oss << "          \"iteration\": " << r.iteration << ",\n";
      oss << "          \"score_nvtl\": " << r.score_nvtl << ",\n";
      oss << "          \"score_tp\": " << r.score_tp << ",\n";
      oss << "          \"score_threshold\": " << r.score_threshold << ",\n";
      oss << "          \"score_type\": \"" << r.score_type << "\",\n";
      oss << "          \"initial_pose\": " << pose_to_json(r.initial_pose) << ",\n";
      oss << "          \"scan_matching_pose\": " << pose_to_json(r.scan_matching_pose) << "\n";
      oss << "        }";
    }
    oss << "\n      ]\n";
  } else {
    oss << "\n";
  }
  oss << "    }";
}

}  // namespace

void write_csv(
  const std::string & path, const std::vector<ScanGroupResult> & scan_groups, double stamp_sec,
  const MapLoadInfo & map_info)
{
  std::ofstream ofs(path);
  if (!ofs) {
    throw std::runtime_error("Failed to open csv: " + path);
  }

  ofs << "scan_offset_from_nearest,run_index,stamp,pointcloud_header_stamp_sec,"
         "dt_pointcloud_from_target_sec,pointcloud_frame_id,has_converged,"
         "passes_score_threshold,iteration,score_nvtl,score_tp,score_threshold,score_type,"
         "initial_pose.position.x,initial_pose.position.y,initial_pose.position.z,"
         "scan_matching_pose.position.x,scan_matching_pose.position.y,scan_matching_pose.position.z,"
         "map_load_mode,map_pcd_file_count,map_point_count\n";

  for (const auto & group : scan_groups) {
    const auto & cloud_sel = group.cloud_sel;
    for (size_t i = 0; i < group.runs.size(); ++i) {
      const auto & r = group.runs[i];
      ofs << cloud_sel.offset_from_nearest << "," << i << "," << format_unix_sec(stamp_sec) << ","
          << format_unix_sec(cloud_sel.header_stamp_sec) << ","
          << format_unix_sec(cloud_sel.dt_from_target_sec) << "," << cloud_sel.frame_id << ","
          << (r.has_converged ? 1 : 0) << "," << (r.passes_score_threshold ? 1 : 0) << ","
          << r.iteration << "," << r.score_nvtl << "," << r.score_tp << "," << r.score_threshold
          << "," << r.score_type << "," << r.initial_pose.position.x << ","
          << r.initial_pose.position.y << "," << r.initial_pose.position.z << ","
          << r.scan_matching_pose.position.x << "," << r.scan_matching_pose.position.y << ","
          << r.scan_matching_pose.position.z << "," << map_info.mode << ","
          << map_info.pcd_file_count << "," << map_info.point_count << "\n";
    }
  }
}

void write_json(
  const std::string & path, const std::vector<ScanGroupResult> & scan_groups,
  double target_unix_sec, const std::string & source_bag, const std::string & initial_pose_yaml,
  const std::string & pointcloud_topic, int neighbor_scans, const MapLoadInfo & map_info,
  const geometry_msgs::msg::Pose & initial_pose)
{
  const PointCloudSelection * nearest = find_nearest_scan(scan_groups);
  if (nearest == nullptr) {
    throw std::runtime_error("No scan groups to write");
  }

  const ScanGroupResult * nearest_group = nullptr;
  for (const auto & group : scan_groups) {
    if (group.cloud_sel.offset_from_nearest == 0) {
      nearest_group = &group;
      break;
    }
  }
  if (nearest_group == nullptr) {
    nearest_group = &scan_groups.front();
  }

  std::ostringstream oss;
  oss << "{\n";
  oss << "  \"status\": \"ok\",\n";
  oss << "  \"method\": \"ndt_direct_align_cascade_scan\",\n";
  oss << "  \"target_unix_sec\": " << format_unix_sec(target_unix_sec) << ",\n";
  oss << "  \"source_rosbag\": \"" << source_bag << "\",\n";
  oss << "  \"initial_pose_yaml\": \"" << initial_pose_yaml << "\",\n";
  oss << "  \"pointcloud_topic\": \"" << pointcloud_topic << "\",\n";
  oss << "  \"neighbor_scans\": " << neighbor_scans << ",\n";
  oss << "  \"pointcloud_header\": {\n";
  oss << "    \"stamp_sec\": " << format_unix_sec(nearest->header_stamp_sec) << ",\n";
  oss << "    \"frame_id\": \"" << nearest->frame_id << "\",\n";
  oss << "    \"dt_from_target_sec\": " << format_unix_sec(nearest->dt_from_target_sec) << "\n";
  oss << "  },\n";
  oss << "  \"initial_pose\": " << pose_to_json(initial_pose) << ",\n";
  oss << "  \"map_load\": {\n";
  oss << "    \"mode\": \"" << map_info.mode << "\",\n";
  oss << "    \"pcd_file_count\": " << map_info.pcd_file_count << ",\n";
  oss << "    \"point_count\": " << map_info.point_count << ",\n";
  oss << "    \"loaded_cell_ids\": [";
  for (size_t i = 0; i < map_info.loaded_cell_ids.size(); ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << "\"" << map_info.loaded_cell_ids[i] << "\"";
  }
  oss << "]\n  },\n";
  oss << "  \"n_runs\": " << nearest_group->runs.size() << ",\n";
  oss << "  \"per_run\": [\n";
  for (size_t i = 0; i < nearest_group->runs.size(); ++i) {
    const auto & r = nearest_group->runs[i];
    if (i > 0) {
      oss << ",\n";
    }
    oss << "    {\n";
    oss << "      \"run_index\": " << i << ",\n";
    oss << "      \"has_converged\": " << (r.has_converged ? "true" : "false") << ",\n";
    oss << "      \"passes_score_threshold\": " << (r.passes_score_threshold ? "true" : "false")
        << ",\n";
    oss << "      \"iteration\": " << r.iteration << ",\n";
    oss << "      \"score_nvtl\": " << r.score_nvtl << ",\n";
    oss << "      \"score_tp\": " << r.score_tp << ",\n";
    oss << "      \"score_threshold\": " << r.score_threshold << ",\n";
    oss << "      \"score_type\": \"" << r.score_type << "\",\n";
    oss << "      \"initial_pose\": " << pose_to_json(r.initial_pose) << ",\n";
    oss << "      \"scan_matching_pose\": " << pose_to_json(r.scan_matching_pose) << "\n";
    oss << "    }";
  }
  oss << "\n  ],\n";
  oss << "  \"per_scan\": [\n";
  for (size_t i = 0; i < scan_groups.size(); ++i) {
    if (i > 0) {
      oss << ",\n";
    }
    write_scan_group_json(oss, scan_groups[i], true);
  }
  oss << "\n  ]\n";
  oss << "}\n";

  std::ofstream ofs(path);
  if (!ofs) {
    throw std::runtime_error("Failed to open json: " + path);
  }
  ofs << oss.str();
}

}  // namespace ndt_direct_measure
