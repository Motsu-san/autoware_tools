// Copyright 2026 scripts_for_autoware contributors
// SPDX-License-Identifier: Apache-2.0

#include "ndt_direct_measure/bag_reader.hpp"

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>

namespace ndt_direct_measure
{
namespace
{

std::string resolve_bag_uri(const std::string & bag_path)
{
  namespace fs = std::filesystem;
  const fs::path p(bag_path);
  if (fs::is_directory(p)) {
    return p.string();
  }
  if (p.extension() == ".db3") {
    return p.parent_path().string();
  }
  return bag_path;
}

double stamp_to_sec(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

PointCloudSelection find_nearest_pointcloud(
  const std::string & bag_path, const std::string & topic, double target_unix_sec)
{
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = resolve_bag_uri(bag_path);
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  rosbag2_cpp::Reader reader;
  try {
    reader.open(storage_options, converter_options);
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to open bag: " + storage_options.uri + ": " + e.what());
  }

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
  bool found = false;
  double best_dt = std::numeric_limits<double>::max();
  PointCloudSelection best;

  while (reader.has_next()) {
    const auto bag_msg = reader.read_next();
    if (bag_msg->topic_name != topic) {
      continue;
    }

    rclcpp::SerializedMessage extracted(*bag_msg->serialized_data);
    sensor_msgs::msg::PointCloud2 cloud;
    serialization.deserialize_message(&extracted, &cloud);

    const double hdr_sec = stamp_to_sec(cloud.header.stamp);
    const double dt = std::abs(hdr_sec - target_unix_sec);
    if (!found || dt < best_dt) {
      found = true;
      best_dt = dt;
      best.cloud = cloud;
      best.header_stamp_sec = hdr_sec;
      best.dt_from_target_sec = dt;
      best.frame_id = cloud.header.frame_id;
    }
  }

  if (!found) {
    throw std::runtime_error("Topic not found in bag: " + topic);
  }
  return best;
}

}  // namespace ndt_direct_measure
