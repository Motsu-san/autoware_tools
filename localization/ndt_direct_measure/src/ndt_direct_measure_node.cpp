// Copyright 2026 scripts_for_autoware contributors
// SPDX-License-Identifier: Apache-2.0

#include "ndt_direct_measure/bag_reader.hpp"
#include "ndt_direct_measure/map_loader.hpp"
#include "ndt_direct_measure/output_writer.hpp"
#include "ndt_direct_measure/pose_extrapolation.hpp"
#include "ndt_direct_measure/pose_yaml.hpp"
#include "ndt_direct_measure/types.hpp"

#include <autoware/ndt_scan_matcher/hyper_parameters.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ndt_direct_measure
{
namespace
{

struct CliOptions
{
  std::string map_path;
  std::string source_bag;
  double target_unix_sec{0.0};
  std::string pointcloud_topic{"/sensing/lidar/concatenated/pointcloud"};
  std::string initial_pose_yaml;
  std::string ndt_param_yaml;
  std::string output_json;
  std::string output_csv;
  int n_runs{1};
  int neighbor_scans{0};
  std::string map_load_mode{"metadata_radius"};
  double map_radius_m{150.0};
  std::string metadata_yaml_path;
  bool remove_nan{true};
};

void print_usage(const char * prog)
{
  std::cerr
    << "Usage: " << prog
    << " --map-path PATH --source-bag PATH --target-unix-sec SEC"
    << " --initial-pose-yaml PATH [options]\n"
    << "Options:\n"
    << "  --ndt-param-yaml PATH     NDT params (default: autoware_ndt_scan_matcher share)\n"
    << "  --pointcloud-topic TOPIC  (default: /sensing/lidar/concatenated/pointcloud)\n"
    << "  --output-json PATH\n"
    << "  --output-csv PATH\n"
    << "  --n-runs N                (default: 1)\n"
    << "  --neighbor-scans N        also align ±N scans around nearest (default: 0)\n"
    << "  --map-load-mode MODE      all | metadata_radius (default: metadata_radius)\n"
    << "  --map-radius-m M          (default: 150)\n"
    << "  --metadata-yaml PATH      (default: <map-parent>/pointcloud_map_metadata.yaml)\n"
    << "  --no-remove-nan           keep NaN points in source cloud\n";
}

std::string find_param_yaml_in_prefixes(
  const std::string & prefixes, const std::string & relative_path)
{
  size_t start = 0;
  while (start < prefixes.size()) {
    const size_t end = prefixes.find(':', start);
    const std::string prefix =
      prefixes.substr(start, end == std::string::npos ? std::string::npos : end - start);
    const std::string candidate = prefix + relative_path;
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  return "";
}

std::string find_default_ndt_param_yaml()
{
  const char * ament_prefix = std::getenv("AMENT_PREFIX_PATH");
  if (ament_prefix == nullptr) {
    return "";
  }

  const std::string prefixes(ament_prefix);
  const std::string direct_param = find_param_yaml_in_prefixes(
    prefixes, "/share/ndt_direct_measure/config/ndt_direct_measure.param.yaml");
  if (!direct_param.empty()) {
    return direct_param;
  }
  return find_param_yaml_in_prefixes(
    prefixes, "/share/autoware_ndt_scan_matcher/config/ndt_scan_matcher.param.yaml");
}

bool parse_args(int argc, char ** argv, CliOptions & opts)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need_value = [&](const char * name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + name);
      }
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return false;
    }
    if (arg == "--map-path") {
      opts.map_path = need_value(arg.c_str());
    } else if (arg == "--source-bag") {
      opts.source_bag = need_value(arg.c_str());
    } else if (arg == "--target-unix-sec") {
      opts.target_unix_sec = std::stod(need_value(arg.c_str()));
    } else if (arg == "--initial-pose-yaml") {
      opts.initial_pose_yaml = need_value(arg.c_str());
    } else if (arg == "--ndt-param-yaml") {
      opts.ndt_param_yaml = need_value(arg.c_str());
    } else if (arg == "--pointcloud-topic") {
      opts.pointcloud_topic = need_value(arg.c_str());
    } else if (arg == "--output-json") {
      opts.output_json = need_value(arg.c_str());
    } else if (arg == "--output-csv") {
      opts.output_csv = need_value(arg.c_str());
    } else if (arg == "--n-runs") {
      opts.n_runs = std::max(1, std::stoi(need_value(arg.c_str())));
    } else if (arg == "--neighbor-scans") {
      opts.neighbor_scans = std::max(0, std::stoi(need_value(arg.c_str())));
    } else if (arg == "--map-load-mode") {
      opts.map_load_mode = need_value(arg.c_str());
    } else if (arg == "--map-radius-m") {
      opts.map_radius_m = std::stod(need_value(arg.c_str()));
    } else if (arg == "--metadata-yaml") {
      opts.metadata_yaml_path = need_value(arg.c_str());
    } else if (arg == "--no-remove-nan") {
      opts.remove_nan = false;
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (
    opts.map_path.empty() || opts.source_bag.empty() || opts.initial_pose_yaml.empty() ||
    opts.target_unix_sec <= 0.0) {
    print_usage(argv[0]);
    throw std::runtime_error(
      "Required: --map-path, --source-bag, --target-unix-sec, --initial-pose-yaml");
  }

  if (opts.ndt_param_yaml.empty()) {
    opts.ndt_param_yaml = find_default_ndt_param_yaml();
    if (opts.ndt_param_yaml.empty()) {
      throw std::runtime_error(
        "Could not find ndt_scan_matcher.param.yaml; pass --ndt-param-yaml");
    }
  }

  if (opts.metadata_yaml_path.empty()) {
    const std::filesystem::path map_p(opts.map_path);
    opts.metadata_yaml_path = (map_p.parent_path() / "pointcloud_map_metadata.yaml").string();
  }

  if (opts.map_load_mode != "all" && opts.map_load_mode != "metadata_radius") {
    throw std::runtime_error("map_load_mode must be 'all' or 'metadata_radius'");
  }
  return true;
}

pcl::PointCloud<PointType>::Ptr to_point_cloud(
  const sensor_msgs::msg::PointCloud2 & msg, bool remove_nan)
{
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(msg, *cloud);
  if (!remove_nan) {
    return cloud;
  }

  pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>);
  filtered->reserve(cloud->size());
  for (const auto & pt : cloud->points) {
    if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
      filtered->push_back(pt);
    }
  }
  return filtered;
}

ScanMatcherResult run_scan_matching(
  NormalDistributionsTransform & registration, const geometry_msgs::msg::Pose & initial_pose,
  const pcl::PointCloud<PointType>::Ptr & input_points,
  const autoware::ndt_scan_matcher::HyperParameters & hp)
{
  Eigen::Affine3d affine_matrix;
  tf2::fromMsg(initial_pose, affine_matrix);
  const Eigen::Matrix4f matrix = affine_matrix.matrix().cast<float>();

  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  registration.align(*output_cloud, matrix, input_points);

  const Eigen::Matrix4f final_transformation = registration.getFinalTransformation();
  Eigen::Affine3d result_affine;
  result_affine.matrix() = final_transformation.cast<double>();

  ScanMatcherResult result;
  result.initial_pose = initial_pose;
  result.scan_matching_pose = tf2::toMsg(result_affine);
  result.iteration = registration.getFinalNumIteration();
  result.score_nvtl = registration.getNearestVoxelTransformationLikelihood();
  result.score_tp = registration.getTransformationProbability();
  // NDT は |delta_p_norm| < trans_epsilon で早期終了する。上限到達時のみ iteration == max。
  result.has_converged = result.iteration < registration.getMaximumIterations();

  using autoware::ndt_scan_matcher::ConvergedParamType;
  if (hp.score_estimation.converged_param_type == ConvergedParamType::TRANSFORM_PROBABILITY) {
    result.score_type = "transform_probability";
    result.score_threshold = hp.score_estimation.converged_param_transform_probability;
    result.passes_score_threshold = result.score_tp > result.score_threshold;
  } else {
    result.score_type = "nearest_voxel_transformation_likelihood";
    result.score_threshold =
      hp.score_estimation.converged_param_nearest_voxel_transformation_likelihood;
    result.passes_score_threshold = result.score_nvtl > result.score_threshold;
  }
  return result;
}

}  // namespace

ScanGroupResult run_scan_group(
  NormalDistributionsTransform & registration,
  const PointCloudSelection & cloud_sel, const geometry_msgs::msg::Pose & scan_initial_pose,
  const ScanInitialPoseInfo & initial_pose_info, int n_runs,
  const autoware::ndt_scan_matcher::HyperParameters & hp, bool remove_nan,
  rclcpp::Logger logger)
{
  const auto input_points = to_point_cloud(cloud_sel.cloud, remove_nan);
  if (input_points->empty()) {
    throw std::runtime_error("Input point cloud is empty after conversion");
  }

  ScanGroupResult group;
  group.cloud_sel = cloud_sel;
  group.initial_pose_info = initial_pose_info;
  group.runs.reserve(static_cast<size_t>(n_runs));
  for (int i = 0; i < n_runs; ++i) {
    RCLCPP_INFO(
      logger, "NDT align offset=%d run %d/%d initial_source=%s", cloud_sel.offset_from_nearest,
      i + 1, n_runs, initial_pose_info.source.c_str());
    group.runs.push_back(
      run_scan_matching(registration, scan_initial_pose, input_points, hp));
    const auto & r = group.runs.back();
        RCLCPP_INFO(
          logger,
          "  converged=%s score_ok=%s iter=%d nvtl=%.4f tp=%.4f pose=(%.3f, %.3f, %.3f)",
          r.has_converged ? "true" : "false", r.passes_score_threshold ? "true" : "false",
          r.iteration, r.score_nvtl, r.score_tp, r.scan_matching_pose.position.x,
          r.scan_matching_pose.position.y, r.scan_matching_pose.position.z);
  }
  return group;
}

ScanInitialPoseInfo make_yaml_initial_pose_info(const geometry_msgs::msg::Pose & pose)
{
  ScanInitialPoseInfo info;
  info.pose = pose;
  info.source = "ndt_start_pose_yaml";
  info.extrapolated = false;
  return info;
}

ScanInitialPoseInfo make_extrapolated_initial_pose_info(const ExtrapolatedInitialPose & extrap)
{
  ScanInitialPoseInfo info;
  info.pose = extrap.pose;
  info.source = "ndt_pose_extrapolation";
  info.extrapolated = true;
  info.ref_offset_a = extrap.ref_offset_a;
  info.ref_offset_b = extrap.ref_offset_b;
  info.extrapolation_alpha = extrap.alpha;
  return info;
}

std::vector<ScanGroupResult> run_cascade_alignment(
  NormalDistributionsTransform & registration, const std::vector<PointCloudSelection> & selections,
  const geometry_msgs::msg::Pose & ndt_start_pose, int neighbor_scans, int n_runs,
  const autoware::ndt_scan_matcher::HyperParameters & hp, bool remove_nan, rclcpp::Logger logger)
{
  std::map<int, PointCloudSelection> by_offset;
  for (const auto & sel : selections) {
    by_offset[sel.offset_from_nearest] = sel;
  }

  std::map<int, ScanGroupResult> completed;
  std::map<int, OffsetAnchor> anchors;

  auto run_offset = [&](int offset, const ScanInitialPoseInfo & initial_info) {
    const auto it = by_offset.find(offset);
    if (it == by_offset.end()) {
      return;
    }
    const auto & cloud_sel = it->second;
    RCLCPP_INFO(
      logger, "Selected cloud: offset=%d stamp=%.9f dt=%.6f frame_id=%s width=%u",
      cloud_sel.offset_from_nearest, cloud_sel.header_stamp_sec, cloud_sel.dt_from_target_sec,
      cloud_sel.frame_id.c_str(), cloud_sel.cloud.width);
    completed[offset] = run_scan_group(
      registration, cloud_sel, initial_info.pose, initial_info, n_runs, hp, remove_nan, logger);
    anchors[offset] = make_offset_anchor(completed[offset]);
  };

  const auto yaml_info = make_yaml_initial_pose_info(ndt_start_pose);

  run_offset(0, yaml_info);
  if (neighbor_scans >= 1) {
    run_offset(1, yaml_info);
    run_offset(-1, yaml_info);
  }

  if (neighbor_scans >= 2) {
    auto run_extrapolated = [&](int target_offset, int ref_a, int ref_b) {
      const auto target_it = by_offset.find(target_offset);
      if (target_it == by_offset.end()) {
        return;
      }
      ScanInitialPoseInfo initial_info = yaml_info;
      const auto anchor_a_it = anchors.find(ref_a);
      const auto anchor_b_it = anchors.find(ref_b);
      if (anchor_a_it != anchors.end() && anchor_b_it != anchors.end()) {
        auto extrap = extrapolate_initial_pose(
          anchor_a_it->second, anchor_b_it->second, target_it->second.header_stamp_sec);
        if (extrap.has_value()) {
          extrap->ref_offset_a = ref_a;
          extrap->ref_offset_b = ref_b;
          initial_info = make_extrapolated_initial_pose_info(extrap.value());
          RCLCPP_INFO(
            logger,
            "Extrapolated initial pose for offset=%d from offsets %d,%d alpha=%.6f -> (%.3f, %.3f)",
            target_offset, ref_a, ref_b, initial_info.extrapolation_alpha,
            initial_info.pose.position.x, initial_info.pose.position.y);
        } else {
          RCLCPP_WARN(
            logger,
            "Fallback to ndt_start_pose for offset=%d (anchors %d,%d not usable for extrapolation)",
            target_offset, ref_a, ref_b);
        }
      } else {
        RCLCPP_WARN(
          logger, "Fallback to ndt_start_pose for offset=%d (missing anchor offsets %d,%d)",
          target_offset, ref_a, ref_b);
      }
      run_offset(target_offset, initial_info);
    };

    run_extrapolated(2, 0, 1);
    run_extrapolated(-2, -1, 0);
  }

  std::vector<ScanGroupResult> ordered;
  ordered.reserve(selections.size());
  for (const auto & sel : selections) {
    const auto it = completed.find(sel.offset_from_nearest);
    if (it != completed.end()) {
      ordered.push_back(it->second);
    }
  }
  return ordered;
}

}  // namespace ndt_direct_measure

int main(int argc, char ** argv)
{
  ndt_direct_measure::CliOptions opts;
  try {
    if (!ndt_direct_measure::parse_args(argc, argv, opts)) {
      return 0;
    }
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  rclcpp::init(argc, argv);

  try {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(false);
    node_options.arguments(
      {"--ros-args", "--params-file", opts.ndt_param_yaml, "-r", "__node:=ndt_direct_measure"});

    auto node = rclcpp::Node::make_shared("ndt_direct_measure", node_options);
    autoware::ndt_scan_matcher::HyperParameters hp(node.get());

    auto registration = std::make_unique<ndt_direct_measure::NormalDistributionsTransform>();
    registration->setParams(hp.ndt);

    const auto initial_pose =
      ndt_direct_measure::load_pose_from_ndt_start_yaml(opts.initial_pose_yaml);
    const auto map_info = ndt_direct_measure::load_map_into_ndt(
      *registration, opts.map_path, opts.map_load_mode, initial_pose.position.x,
      initial_pose.position.y, opts.map_radius_m, opts.metadata_yaml_path, node->get_logger());

    RCLCPP_INFO(node->get_logger(), "Finding point cloud(s) in bag...");
    const auto cloud_selections = ndt_direct_measure::find_pointclouds_around_target(
      opts.source_bag, opts.pointcloud_topic, opts.target_unix_sec, opts.neighbor_scans);

    const auto scan_groups = ndt_direct_measure::run_cascade_alignment(
      *registration, cloud_selections, initial_pose, opts.neighbor_scans, opts.n_runs, hp,
      opts.remove_nan, node->get_logger());

    if (!opts.output_csv.empty()) {
      ndt_direct_measure::write_csv(
        opts.output_csv, scan_groups, opts.target_unix_sec, map_info);
      RCLCPP_INFO(node->get_logger(), "Wrote CSV: %s", opts.output_csv.c_str());
    }
    if (!opts.output_json.empty()) {
      ndt_direct_measure::write_json(
        opts.output_json, scan_groups, opts.target_unix_sec, opts.source_bag, opts.initial_pose_yaml,
        opts.pointcloud_topic, opts.neighbor_scans, map_info, initial_pose);
      RCLCPP_INFO(node->get_logger(), "Wrote JSON: %s", opts.output_json.c_str());
    }

    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("ndt_direct_measure"), "%s", e.what());
    rclcpp::shutdown();
    return 2;
  }
}
