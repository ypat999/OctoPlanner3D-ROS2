
#include "pcd2octomap_converter.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <sys/stat.h>

#include <pcl/io/pcd_io.h>

namespace pcd2octomap
{

bool Key::operator==(const Key & other) const
{
  return k[0] == other.k[0] && k[1] == other.k[1] && k[2] == other.k[2];
}

std::size_t KeyHash::operator()(const Key & key) const
{
  std::size_t seed = std::hash<unsigned int>{}(key.k[0]);
  seed ^= std::hash<unsigned int>{}(key.k[1]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  seed ^= std::hash<unsigned int>{}(key.k[2]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  return seed;
}

Pcd2OctomapConverter::Pcd2OctomapConverter()
: cloud_(new pcl::PointCloud<pcl::PointXYZ>),
  tree_(std::make_shared<octomap::OcTree>(resolution_))
{
}

void Pcd2OctomapConverter::setInputPcdFile(const std::string & input_pcd)
{
  input_pcd_ = input_pcd;

  // 自动推导 .bt 文件名：与 PCD 同目录、同名、.bt 后缀
  // e.g. /path/to/building.pcd -> /path/to/building.bt
  // 如果用户显式调用了 setOutputBtFile，会被覆盖
  std::size_t dot = input_pcd_.rfind('.');
  std::size_t sep = input_pcd_.rfind('/');
  if (dot != std::string::npos && dot > sep) {
    output_bt_ = input_pcd_.substr(0, dot) + ".bt";
  } else {
    output_bt_ = input_pcd_ + ".bt";
  }
}

void Pcd2OctomapConverter::setOutputBtFile(const std::string & output_bt)
{
  output_bt_ = output_bt;
}

void Pcd2OctomapConverter::setResolution(double resolution)
{
  resolution_ = resolution;
}

void Pcd2OctomapConverter::setMinPointsPerVoxel(int min_points)
{
  min_points_per_voxel_ = min_points;
}

void Pcd2OctomapConverter::setMinClusterVoxels(int min_cluster)
{
  min_cluster_voxels_ = min_cluster;
}

bool Pcd2OctomapConverter::convert()
{
  std::cout << "[OctoPlanner] Converting " << input_pcd_ 
            << "  →  " << output_bt_ << " ..." << std::endl;
  std::cout.flush();
  const auto t_start = std::chrono::steady_clock::now();

  if (tryLoadCached()) {
    const double total_s =
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    std::cout << "[OctoPlanner] Loaded cached " << output_bt_ 
              << " in " << total_s << " s." << std::endl;
    std::cout.flush();
    return true;
  }

  std::cout << "[OctoPlanner] Loading " << input_pcd_ << " ..." << std::endl;
  std::cout.flush();
  if (!loadPointCloud()) {
    return false;
  }

  tree_ = std::make_shared<octomap::OcTree>(resolution_);

  buildVoxelCounts();
  filterByPointCount();
  filterByConnectedClusters();
  fillOcTree();

  if (!saveOctomap()) {
    return false;
  }

  const auto t_end = std::chrono::steady_clock::now();
  const double total_s =
    std::chrono::duration<double>(t_end - t_start).count();
  std::cout << "\n[OctoPlanner] Conversion finished in " << total_s << " s. To visualize, run:\n";
  std::cout << "[OctoPlanner] octovis " << output_bt_ << std::endl;
  std::cout.flush();

  return true;
}

bool Pcd2OctomapConverter::loadPointCloud()
{
  const auto t0 = std::chrono::steady_clock::now();
  cloud_->clear();

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_, *cloud_) == -1) {
    std::cerr << "[OctoPlanner] Couldn't read file " << input_pcd_ << std::endl;
    std::cerr.flush();
    return false;
  }

  const double load_s =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();
  std::cout << "[OctoPlanner] Loaded " << cloud_->size() << " points in "
            << load_s << " s." << std::endl;
  std::cout.flush();
  return true;
}

bool Pcd2OctomapConverter::readPcdHeader(std::size_t & out_point_count) const
{
  std::ifstream f(input_pcd_.c_str());
  if (!f.is_open()) {
    return false;
  }

  std::size_t width = 0;
  std::size_t height = 1;  // 默认 1（非 organized 点云）

  std::string line;
  while (std::getline(f, line) && !line.empty() && line[0] != '#') {
    if (line.substr(0, 6) == "WIDTH ") {
      width = std::stoul(line.substr(6));
    } else if (line.substr(0, 7) == "HEIGHT ") {
      height = std::stoul(line.substr(7));
    } else if (line == "DATA ascii" || line == "DATA binary") {
      break;
    }
  }

  if (width == 0) {
    return false;
  }

  out_point_count = width * height;
  return true;
}

std::string cacheKeyPath(const std::string & bt_path)
{
  return bt_path + ".cache_key";
}

bool Pcd2OctomapConverter::saveCacheKey(std::size_t point_count) const
{
  struct stat pcd_stat;
  if (stat(input_pcd_.c_str(), &pcd_stat) != 0) {
    return false;
  }

  std::ofstream f(cacheKeyPath(output_bt_).c_str());
  if (!f.is_open()) {
    return false;
  }

  f << input_pcd_ << "\n";
  f << pcd_stat.st_size << "\n";
  f << point_count << "\n";
  f << resolution_ << "\n";
  f << min_points_per_voxel_ << "\n";
  f << min_cluster_voxels_ << "\n";
  return true;
}

bool Pcd2OctomapConverter::checkCacheKey() const
{
  std::ifstream f(cacheKeyPath(output_bt_).c_str());
  if (!f.is_open()) {
    return false;
  }

  std::string stored_path;
  std::size_t stored_size = 0;
  std::size_t stored_points = 0;
  double stored_resolution = 0.0;
  int stored_min_points = 0;
  int stored_min_cluster = 0;

  if (!std::getline(f, stored_path))    { return false; }
  if (!(f >> stored_size))              { return false; }
  if (!(f >> stored_points))            { return false; }
  if (!(f >> stored_resolution))        { return false; }
  if (!(f >> stored_min_points))        { return false; }
  if (!(f >> stored_min_cluster))       { return false; }

  // PCD 路径必须一致
  if (stored_path != input_pcd_) {
    std::cout << "[OctoPlanner] Cached PCD path mismatch (" << stored_path
              << " vs " << input_pcd_ << "), rebuilding." << std::endl;
    std::cout.flush();
    return false;
  }

  // 检查当前 PCD 文件大小
  struct stat pcd_stat;
  if (stat(input_pcd_.c_str(), &pcd_stat) != 0) {
    return false;
  }
  if (static_cast<std::size_t>(pcd_stat.st_size) != stored_size) {
    std::cout << "[OctoPlanner] Cached PCD file size changed (" << pcd_stat.st_size
              << " vs " << stored_size << "), rebuilding." << std::endl;
    std::cout.flush();
    return false;
  }

  // 检查当前 PCD 点数
  std::size_t current_points = 0;
  if (!readPcdHeader(current_points)) {
    return false;
  }
  if (current_points != stored_points) {
    std::cout << "[OctoPlanner] Cached PCD point count changed (" << current_points
              << " vs " << stored_points << "), rebuilding." << std::endl;
    std::cout.flush();
    return false;
  }

  // 检查参数是否变化
  if (std::abs(stored_resolution - resolution_) > 1e-6) {
    std::cout << "[OctoPlanner] Resolution changed (" << stored_resolution
              << " vs " << resolution_ << "), rebuilding." << std::endl;
    std::cout.flush();
    return false;
  }
  if (stored_min_points != min_points_per_voxel_) {
    std::cout << "[OctoPlanner] min_points_per_voxel changed (" << stored_min_points
              << " vs " << min_points_per_voxel_ << "), rebuilding." << std::endl;
    std::cout.flush();
    return false;
  }
  if (stored_min_cluster != min_cluster_voxels_) {
    std::cout << "[OctoPlanner] min_cluster_voxels changed (" << stored_min_cluster
              << " vs " << min_cluster_voxels_ << "), rebuilding." << std::endl;
    std::cout.flush();
    return false;
  }

  return true;
}

bool Pcd2OctomapConverter::tryLoadCached()
{
  struct stat bt_stat;

  // 检查 .bt 文件是否存在
  if (stat(output_bt_.c_str(), &bt_stat) != 0) {
    return false;
  }

  // 校验缓存有效性（路径、文件大小、点数）
  if (!checkCacheKey()) {
    return false;
  }

  tree_ = std::make_shared<octomap::OcTree>(resolution_);
  if (!tree_->readBinary(output_bt_)) {
    std::cerr << "[OctoPlanner] Failed to load cached " << output_bt_ << std::endl;
    std::cerr.flush();
    tree_.reset();
    return false;
  }

  std::cout << "[OctoPlanner] Cache valid, loaded " << output_bt_ << " ("
            << bt_stat.st_size << " bytes)." << std::endl;
  std::cout.flush();
  return true;
}

void Pcd2OctomapConverter::buildVoxelCounts()
{
  const auto t0 = std::chrono::steady_clock::now();
  voxel_counts_.clear();

  const size_t total = cloud_->size();
  size_t processed = 0;
  const size_t report_interval = std::max(size_t(1), total / 10);  // 每 10% 报一次

  for (const auto & p : cloud_->points) {
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) {
      ++processed;
      continue;
    }

    octomap::OcTreeKey raw_key;
    if (tree_->coordToKeyChecked(p.x, p.y, p.z, raw_key)) {
      Key key{{raw_key.k[0], raw_key.k[1], raw_key.k[2]}};
      ++voxel_counts_[key];
    }

    ++processed;
    if (processed % report_interval == 0) {
      const int pct = static_cast<int>(processed * 100 / total);
      const double elapsed =
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t0).count();
      std::cout << "\r  Voxelization: " << pct << "%  ("
                << processed << " / " << total << " points, "
                << voxel_counts_.size() << " unique voxels, "
                << elapsed << " s)" << std::flush;
    }
  }

  const double elapsed =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();
  std::cout << "\r[OctoPlanner]   Voxelization: 100%  ("
            << total << " / " << total << " points, "
            << voxel_counts_.size() << " unique voxels, "
            << elapsed << " s)" << std::endl;
  std::cout.flush();
}

void Pcd2OctomapConverter::filterByPointCount()
{
  occupied_keys_.clear();

  for (const auto & entry : voxel_counts_) {
    if (static_cast<int>(entry.second) >= min_points_per_voxel_) {
      occupied_keys_.insert(entry.first);
    }
  }

  std::cout << "[OctoPlanner]   Voxels after point-count filtering: "
            << occupied_keys_.size() << std::endl;
  std::cout.flush();
}

void Pcd2OctomapConverter::filterByConnectedClusters()
{
  if (min_cluster_voxels_ <= 1 || occupied_keys_.empty()) {
    std::cout << "[OctoPlanner]   Voxels after cluster filtering: "
              << occupied_keys_.size() << std::endl;
    std::cout.flush();
    return;
  }

  const auto t0 = std::chrono::steady_clock::now();
  const size_t total_seeds = occupied_keys_.size();

  std::unordered_set<Key, KeyHash> filtered_keys;
  std::unordered_set<Key, KeyHash> visited;
  filtered_keys.reserve(occupied_keys_.size());
  visited.reserve(occupied_keys_.size());

  size_t processed_seeds = 0;
  const size_t report_interval = std::max(size_t(1), total_seeds / 10);

  for (const auto & seed : occupied_keys_) {
    if (visited.count(seed)) {
      continue;
    }

    std::deque<Key> queue;
    std::vector<Key> cluster;

    queue.push_back(seed);
    visited.insert(seed);

    while (!queue.empty()) {
      Key current = queue.front();
      queue.pop_front();
      cluster.push_back(current);

      // 检查 26 邻域，3x3x3 范围
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }

            Key neighbor{{
              static_cast<unsigned int>(current.k[0] + dx),
              static_cast<unsigned int>(current.k[1] + dy),
              static_cast<unsigned int>(current.k[2] + dz)
            }};

            if (occupied_keys_.count(neighbor) && !visited.count(neighbor)) {
              visited.insert(neighbor);
              queue.push_back(neighbor);
            }
          }
        }
      }
    }

    if (static_cast<int>(cluster.size()) >= min_cluster_voxels_) {
      for (const auto & key : cluster) {
        filtered_keys.insert(key);
      }
    }

    ++processed_seeds;
    if (processed_seeds % report_interval == 0) {
      const int pct = static_cast<int>(processed_seeds * 100 / total_seeds);
      const double elapsed =
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t0).count();
      std::cout << "\r[OctoPlanner]   Cluster filtering: " << pct << "%  ("
                << processed_seeds << " / " << total_seeds << " seeds, "
                << filtered_keys.size() << " kept, "
                << elapsed << " s)" << std::flush;
    }
  }

  occupied_keys_ = std::move(filtered_keys);

  const double elapsed =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();
  std::cout << "\r[OctoPlanner]   Cluster filtering: 100%  ("
            << total_seeds << " / " << total_seeds << " seeds, "
            << occupied_keys_.size() << " kept, "
            << elapsed << " s)" << std::endl;
  std::cout.flush();
}

void Pcd2OctomapConverter::fillOcTree()
{
  const auto t0 = std::chrono::steady_clock::now();
  const size_t total = occupied_keys_.size();
  size_t processed = 0;
  const size_t report_interval = std::max(size_t(1), total / 10);

  for (const auto & key : occupied_keys_) {
    octomap::OcTreeKey octo_key;
    octo_key.k[0] = key.k[0];
    octo_key.k[1] = key.k[1];
    octo_key.k[2] = key.k[2];

    tree_->updateNode(tree_->keyToCoord(octo_key), true);

    ++processed;
    if (processed % report_interval == 0) {
      const int pct = static_cast<int>(processed * 100 / total);
      const double elapsed =
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t0).count();
      std::cout << "\r[OctoPlanner]   Filling OctoTree: " << pct << "%  ("
                << processed << " / " << total << " voxels, "
                << elapsed << " s)" << std::flush;
    }
  }

  tree_->updateInnerOccupancy();

  const double elapsed =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();
  std::cout << "\r[OctoPlanner]   Filling OctoTree: 100%  ("
            << total << " / " << total << " voxels, "
            << elapsed << " s)" << std::endl;
  std::cout.flush();
}

bool Pcd2OctomapConverter::saveOctomap() const
{
  const auto t0 = std::chrono::steady_clock::now();

  if (!tree_) {
    std::cerr << "[OctoPlanner] OcTree is null, cannot save." << std::endl;
    std::cerr.flush();
    return false;
  }

  bool ok = tree_->writeBinaryConst(output_bt_);

  const double elapsed =
    std::chrono::duration<double>(
      std::chrono::steady_clock::now() - t0).count();

  if (ok) {
    std::cout << "[OctoPlanner] Saved " << output_bt_ << " in " << elapsed << " s." << std::endl;
    std::cout.flush();
    saveCacheKey(cloud_->size());
    return true;
  }

  std::cerr << "[OctoPlanner] Failed to save " << output_bt_ << std::endl;
  std::cerr.flush();
  return false;
}

bool Pcd2OctomapConverter::isPointFree(const octomap::point3d & p) const
{
  if (!tree_) {
    return false;
  }

  octomap::OcTreeNode * node = tree_->search(p);

  // 未知区域按可通行处理
  if (node == nullptr) {
    return true;
  }

  return !tree_->isNodeOccupied(node);
}

bool Pcd2OctomapConverter::isSpaceFree(
  const octomap::point3d & min_pt,
  const octomap::point3d & max_pt) const
{
  if (!tree_) {
    return false;
  }

  for (octomap::OcTree::leaf_bbx_iterator it = tree_->begin_leafs_bbx(min_pt, max_pt),
       end = tree_->end_leafs_bbx(); it != end; ++it)
  {
    if (tree_->isNodeOccupied(*it)) {
      return false;
    }
  }

  return true;
}

std::shared_ptr<octomap::OcTree> Pcd2OctomapConverter::getOctomap()
{
    return tree_;
}

void Pcd2OctomapConverter::printOccupiedNodes() const
{
  if (!tree_) {
    return;
  }

  int count = 0;

  for (octomap::OcTree::leaf_iterator it = tree_->begin_leafs(),
       end = tree_->end_leafs(); it != end; ++it)
  {
    if (tree_->isNodeOccupied(*it)) {
      octomap::point3d p = it.getCoordinate();
      double size = it.getSize();

      std::cout << "Node [" << count << "]: "
                << "x=" << p.x() << ", y=" << p.y() << ", z=" << p.z()
                << " (Size: " << size << ")" << std::endl;

      ++count;
    }
  }
}

void Pcd2OctomapConverter::printQueryExamples() const
{
  octomap::point3d current_pos(13.1, 4.1, 14);

  if (isPointFree(current_pos)) {
    std::cout << "free" << std::endl;
  } else {
    std::cout << "occupy" << std::endl;
  }

  octomap::point3d min_pt(13.1, 4.1, 10);
  octomap::point3d max_pt(13.1, 4.1, 16);

  if (isSpaceFree(min_pt, max_pt)) {
    std::cout << "free" << std::endl;
  } else {
    std::cout << "occupancy" << std::endl;
  }
}

void Pcd2OctomapConverter::visualizeWithOctovis() const
{
  std::system(("octovis " + output_bt_).c_str());
}

std::shared_ptr<octomap::OcTree> Pcd2OctomapConverter::getTree() const
{
  return tree_;
}

}  // namespace pcd2octomap

// int main()
// {
//   pcd2octomap::Pcd2OctomapConverter converter;

//   if (!converter.convert()) {
//     return -1;
//   }

//   converter.printOccupiedNodes();
//   converter.printQueryExamples();

//   // 如果你已经安装了 octovis，保留下面这行会自动弹窗显示。
//   converter.visualizeWithOctovis();

//   return 0;
// }
