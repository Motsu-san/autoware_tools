// Copyright 2026 scripts_for_autoware contributors
// SPDX-License-Identifier: Apache-2.0

#include "ndt_direct_measure/map_loader.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <stdexcept>

namespace ndt_direct_measure
{

namespace fs = std::filesystem;

namespace
{

bool ends_with_pcd(const fs::path & p)
{
  const auto ext = p.extension().string();
  return ext == ".pcd" || ext == ".PCD";
}

struct TileMeta
{
  std::string filename;
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
};

std::vector<TileMeta> parse_metadata_tiles(
  const std::string & metadata_path, double x_res, double y_res)
{
  std::vector<TileMeta> tiles;
  std::ifstream ifs(metadata_path);
  if (!ifs) {
    return tiles;
  }

  std::string line;
  while (std::getline(ifs, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
      line = line.substr(0, hash);
    }
    const auto colon = line.find(':');
    const auto lb = line.find('[', colon == std::string::npos ? 0 : colon);
    const auto comma = line.find(',', lb == std::string::npos ? 0 : lb);
    const auto rb = line.find(']', lb == std::string::npos ? 0 : lb);
    if (
      colon == std::string::npos || lb == std::string::npos || comma == std::string::npos ||
      rb == std::string::npos || colon == 0) {
      continue;
    }

    std::string key = line.substr(0, colon);
    while (!key.empty() && std::isspace(static_cast<unsigned char>(key.back()))) {
      key.pop_back();
    }
    const bool is_pcd =
      key.size() >= 4 && key.compare(key.size() - 4, 4, ".pcd") == 0;
    if (key.find("resolution") != std::string::npos || !is_pcd) {
      continue;
    }

    try {
      TileMeta t;
      t.filename = key;
      t.min_x = std::stod(line.substr(lb + 1, comma - lb - 1));
      t.min_y = std::stod(line.substr(comma + 1, rb - comma - 1));
      t.max_x = t.min_x + x_res;
      t.max_y = t.min_y + y_res;
      tiles.push_back(std::move(t));
    } catch (const std::exception &) {
      continue;
    }
  }
  return tiles;
}

bool tile_intersects_radius(const TileMeta & tile, double cx, double cy, double radius_m)
{
  if (cx + radius_m < tile.min_x || cx - radius_m > tile.max_x) {
    return false;
  }
  if (cy + radius_m < tile.min_y || cy - radius_m > tile.max_y) {
    return false;
  }
  return true;
}

void append_pcd(
  const fs::path & file, pcl::PointCloud<PointType>::Ptr & map, MapLoadInfo & info,
  rclcpp::Logger logger)
{
  sensor_msgs::msg::PointCloud2 partial;
  if (pcl::io::loadPCDFile(file.string(), partial) == -1) {
    RCLCPP_WARN(logger, "PCD load failed: %s", file.c_str());
    return;
  }
  pcl::PointCloud<PointType> cloud;
  pcl::fromROSMsg(partial, cloud);
  *map += cloud;
  info.pcd_file_count++;
  info.point_count += cloud.size();
  info.loaded_cell_ids.push_back(file.filename().string());
}

std::vector<fs::path> collect_pcd_paths_all(const fs::path & map_dir)
{
  std::vector<fs::path> paths;
  for (const auto & entry : fs::recursive_directory_iterator(map_dir)) {
    if (entry.is_regular_file() && ends_with_pcd(entry.path())) {
      paths.push_back(entry.path());
    }
  }
  std::sort(paths.begin(), paths.end());
  return paths;
}

}  // namespace

MapLoadInfo load_map_into_ndt(
  NormalDistributionsTransform & registration, const std::string & map_path,
  const std::string & map_load_mode, double center_x, double center_y, double map_radius_m,
  const std::string & metadata_yaml_path, rclcpp::Logger logger)
{
  MapLoadInfo info;
  info.mode = map_load_mode;

  const fs::path map_dir(map_path);
  if (!fs::exists(map_dir) || !fs::is_directory(map_dir)) {
    throw std::runtime_error("map_path is not a directory: " + map_path);
  }

  pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>);
  std::vector<fs::path> pcd_paths;

  if (map_load_mode == "metadata_radius") {
    double x_res = 20.0;
    double y_res = 20.0;
    try {
      YAML::Node meta = YAML::LoadFile(metadata_yaml_path);
      if (meta["x_resolution"]) {
        x_res = meta["x_resolution"].as<double>();
      }
      if (meta["y_resolution"]) {
        y_res = meta["y_resolution"].as<double>();
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger, "metadata yaml parse failed, using 20m resolution: %s", e.what());
    }

    const auto tiles = parse_metadata_tiles(metadata_yaml_path, x_res, y_res);
    if (tiles.empty()) {
      RCLCPP_WARN(logger, "No tiles parsed from metadata; falling back to all PCDs");
      pcd_paths = collect_pcd_paths_all(map_dir);
    } else {
      for (const auto & tile : tiles) {
        if (!tile_intersects_radius(tile, center_x, center_y, map_radius_m)) {
          continue;
        }
        const fs::path candidate = map_dir / tile.filename;
        if (fs::exists(candidate)) {
          pcd_paths.push_back(candidate);
        } else {
          for (const auto & entry : fs::recursive_directory_iterator(map_dir)) {
            if (entry.is_regular_file() && entry.path().filename() == tile.filename) {
              pcd_paths.push_back(entry.path());
              break;
            }
          }
        }
      }
      std::sort(pcd_paths.begin(), pcd_paths.end());
    }
  } else {
    pcd_paths = collect_pcd_paths_all(map_dir);
  }

  if (pcd_paths.empty()) {
    throw std::runtime_error("No PCD files selected for map load");
  }

  for (const auto & path : pcd_paths) {
    append_pcd(path, map, info, logger);
  }
  if (map->empty()) {
    throw std::runtime_error("Loaded map has zero points");
  }

  registration.setInputTarget(map);
  RCLCPP_INFO(
    logger, "Loaded map: %zu file(s), %zu points (mode=%s)", info.pcd_file_count, info.point_count,
    map_load_mode.c_str());
  return info;
}

}  // namespace ndt_direct_measure
