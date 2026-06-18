// Copyright 2026 scripts_for_autoware contributors
// SPDX-License-Identifier: Apache-2.0

#include "ndt_direct_measure/bag_reader.hpp"

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

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
      best.offset_from_nearest = 0;
    }
  }

  if (!found) {
    throw std::runtime_error("Topic not found in bag: " + topic);
  }
  return best;
}

std::vector<PointCloudSelection> find_pointclouds_around_target(
  const std::string & bag_path, const std::string & topic, double target_unix_sec,
  int neighbor_count)
{
  if (neighbor_count <= 0) {
    return {find_nearest_pointcloud(bag_path, topic, target_unix_sec)};
  }

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = resolve_bag_uri(bag_path);
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  struct StampEntry
  {
    double header_stamp_sec;
    size_t topic_seq;
  };

  std::vector<StampEntry> entries;
  entries.reserve(4096);

  {
    rosbag2_cpp::Reader reader;
    try {
      reader.open(storage_options, converter_options);
    } catch (const std::exception & e) {
      throw std::runtime_error("Failed to open bag: " + storage_options.uri + ": " + e.what());
    }

    size_t topic_seq = 0;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
    while (reader.has_next()) {
      const auto bag_msg = reader.read_next();
      if (bag_msg->topic_name != topic) {
        continue;
      }

      rclcpp::SerializedMessage extracted(*bag_msg->serialized_data);
      sensor_msgs::msg::PointCloud2 cloud;
      serialization.deserialize_message(&extracted, &cloud);
      entries.push_back({stamp_to_sec(cloud.header.stamp), topic_seq});
      ++topic_seq;
    }
  }

  if (entries.empty()) {
    throw std::runtime_error("Topic not found in bag: " + topic);
  }

  std::vector<size_t> sorted_indices(entries.size());
  for (size_t i = 0; i < entries.size(); ++i) {
    sorted_indices[i] = i;
  }
  std::sort(sorted_indices.begin(), sorted_indices.end(), [&](size_t a, size_t b) {
    return entries[a].header_stamp_sec < entries[b].header_stamp_sec;
  });

  size_t nearest_sorted_idx = 0;
  double best_dt = std::numeric_limits<double>::max();
  for (size_t i = 0; i < sorted_indices.size(); ++i) {
    const double dt =
      std::abs(entries[sorted_indices[i]].header_stamp_sec - target_unix_sec);
    if (dt < best_dt) {
      best_dt = dt;
      nearest_sorted_idx = i;
    }
  }

  const size_t begin_sorted = (nearest_sorted_idx >= static_cast<size_t>(neighbor_count))
                                ? nearest_sorted_idx - static_cast<size_t>(neighbor_count)
                                : 0;
  const size_t end_sorted = std::min(
    sorted_indices.size(),
    nearest_sorted_idx + static_cast<size_t>(neighbor_count) + 1);

  std::unordered_set<size_t> needed_topic_seq;
  needed_topic_seq.reserve(end_sorted - begin_sorted);
  for (size_t i = begin_sorted; i < end_sorted; ++i) {
    needed_topic_seq.insert(entries[sorted_indices[i]].topic_seq);
  }

  std::vector<PointCloudSelection> selected;
  selected.reserve(end_sorted - begin_sorted);

  rosbag2_cpp::Reader reader;
  try {
    reader.open(storage_options, converter_options);
  } catch (const std::exception & e) {
    throw std::runtime_error("Failed to open bag: " + storage_options.uri + ": " + e.what());
  }

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
  std::unordered_map<size_t, PointCloudSelection> by_topic_seq;
  by_topic_seq.reserve(needed_topic_seq.size());

  size_t topic_seq = 0;
  while (reader.has_next()) {
    const auto bag_msg = reader.read_next();
    if (bag_msg->topic_name != topic) {
      continue;
    }

    if (needed_topic_seq.count(topic_seq) > 0) {
      rclcpp::SerializedMessage extracted(*bag_msg->serialized_data);
      sensor_msgs::msg::PointCloud2 cloud;
      serialization.deserialize_message(&extracted, &cloud);

      PointCloudSelection sel;
      sel.cloud = cloud;
      sel.header_stamp_sec = stamp_to_sec(cloud.header.stamp);
      sel.dt_from_target_sec = std::abs(sel.header_stamp_sec - target_unix_sec);
      sel.frame_id = cloud.header.frame_id;
      by_topic_seq.emplace(topic_seq, std::move(sel));
    }
    ++topic_seq;
  }

  for (size_t i = begin_sorted; i < end_sorted; ++i) {
    const auto topic_seq_i = entries[sorted_indices[i]].topic_seq;
    auto it = by_topic_seq.find(topic_seq_i);
    if (it == by_topic_seq.end()) {
      throw std::runtime_error("Failed to re-read point cloud from bag");
    }
    PointCloudSelection sel = it->second;
    sel.offset_from_nearest =
      static_cast<int>(i) - static_cast<int>(nearest_sorted_idx);
    selected.push_back(std::move(sel));
  }

  return selected;
}

}  // namespace ndt_direct_measure
