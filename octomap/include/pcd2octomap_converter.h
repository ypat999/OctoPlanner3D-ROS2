/**
 * @file      cd2octomap_converter.h
 * @brief     PCD convert to OctoMap
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-05-31 13:00:01
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC).
 *            All rights reserved.
 */

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcd2octomap
{

// 用于在 STL 容器中存储 OctoMap Key
struct Key
{
  unsigned int k[3];

  bool operator==(const Key & other) const;
};

struct KeyHash
{
  std::size_t operator()(const Key & key) const;
};

class Pcd2OctomapConverter
{
public:
  Pcd2OctomapConverter();

  // 主流程：读取 PCD -> 体素过滤 -> 连通域过滤 -> 生成 OctoMap -> 保存 .bt
  bool convert();

  void setInputPcdFile(const std::string & input_pcd);
  void setOutputBtFile(const std::string & output_bt);
  std::string getOutputBtFile() const { return output_bt_; }
  void setResolution(double resolution);
  void setMinPointsPerVoxel(int min_points);
  void setMinClusterVoxels(int min_cluster);

  // 查询接口
  bool isPointFree(const octomap::point3d & p) const;
  bool isSpaceFree(const octomap::point3d & min_pt, const octomap::point3d & max_pt) const;
  
  std::shared_ptr<octomap::OcTree> getOctomap();



  // 调试 / 可视化
  void printOccupiedNodes() const;
  void printQueryExamples() const;
  void visualizeWithOctovis() const;

  std::shared_ptr<octomap::OcTree> getTree() const;

private:
  bool loadPointCloud();
  bool tryLoadCached();
  bool saveCacheKey(std::size_t point_count) const;
  bool checkCacheKey() const;
  bool readPcdHeader(std::size_t & out_point_count) const;
  void buildVoxelCounts();
  void filterByPointCount();
  void filterByConnectedClusters();
  void fillOcTree();
  bool saveOctomap() const;

private:
  // ================= 配置区域：保持写死，不改成参数 =================
  std::string input_pcd_ = "/home/ztl/slam_data/3d_map/3dmap.pcd";
  std::string output_bt_ = "result_cleaned.bt";

  double resolution_ = 0.2;          // Octomap 分辨率，单位：米
  int min_points_per_voxel_ = 2;     // 每个 voxel 至少多少个点才算占据
  int min_cluster_voxels_ = 2;       // 连通 voxel 数少于该值则视为噪点
  // ===============================================================

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  std::shared_ptr<octomap::OcTree> tree_;

  std::unordered_map<Key, std::size_t, KeyHash> voxel_counts_;
  std::unordered_set<Key, KeyHash> occupied_keys_;
};

}  // namespace pcd2octomap
