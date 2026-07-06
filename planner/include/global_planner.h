/**
 * @file      octo_planner/include/global_planner.h
 * @brief     3D A star Planner
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-05-31 12:00:01 
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC). 
 *            All rights reserved.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <fstream>

#include "octomap/OcTree.h"

namespace global_planner
{

struct GridIndex
{
  int x;
  int y;
  int z;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & k) const
  {
    const std::size_t h1 = std::hash<int>{}(k.x);
    const std::size_t h2 = std::hash<int>{}(k.y);
    const std::size_t h3 = std::hash<int>{}(k.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

struct QueueNode
{
  GridIndex idx;
  double f;
  double g;
};

struct QueueNodeCompare
{
  bool operator()(const QueueNode & a, const QueueNode & b) const
  {
    return a.f > b.f;
  }
};

struct PointPose
{
    double x;
    double y;
    double z;
};

class GlobalPlanner
{
public:
  GlobalPlanner();
  
  ~GlobalPlanner();

  void setOctomap(std::shared_ptr<octomap::OcTree> map);

  /** 设置 OctoMap 并尝试从缓存加载规划层；若缓存不存在则重建并保存 */
  void setOctomapWithCache(
    std::shared_ptr<octomap::OcTree> map,
    const std::string & cache_path);

  /** 设置 OpenMP 并行线程数（0 = 使用默认线程数） */
  void setNumThreads(int n) { num_threads_ = n; }

  void makePlan(const PointPose start,const PointPose goal);

  void getPlannerResults(std::vector<PointPose>& plannerResults);

private:

  void fillBounds(PointPose & min_bound,PointPose & max_bound) const;

  void onGoalPose(const PointPose goal);

  void tryPlan();

  GridIndex worldToGrid(double x, double y, double z) const;

  octomap::point3d gridToWorld(const GridIndex & idx) const;

  bool isInsideMetricBounds(const GridIndex & idx) const;

  bool hasGroundSupport(
    const GridIndex & idx,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells) const;

  bool isOccupiedCell(const GridIndex & idx) const;

  bool hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const;

  bool hasSameLevelNeighborWithOccupiedBelow(const GridIndex & idx) const;

  bool hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const;

  void rebuildPreblockedCells();

//   void onExternalPreblockedMarker(const visualization_msgs::msg::Marker::SharedPtr msg);

  void rebuildPreblockedCostmap();

  double getPreblockedCost(const GridIndex & idx) const;

//   void publishCellSetMarker(
//     const std::unordered_set<GridIndex, GridIndexHash> & cells,
//     const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher,
//     const std::string & ns,
//     float r_color,
//     float g_color,
//     float b_color,
//     float a_color) const;

  void publishPreblockedCellsMarker();

  void publishRiskCostCloud() const;

  void rebuildDerivedLayers();

  /** 将 traversable_cells_ / preblocked_cells_ / preblocked_costmap_ 保存到二进制文件 */
  bool savePlanningCache(const std::string & cache_path) const;
  /** 从二进制文件加载规划层缓存，成功返回 true */
  bool loadPlanningCache(const std::string & cache_path);

  bool isCellTraversable(
    const GridIndex & idx,
    double robot_radius,
    bool require_ground_support,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells) const;

  bool findNearestFreeCell(
    const GridIndex & seed,
    double robot_radius,
    int radius_cells,
    bool require_ground_support,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells,
    GridIndex & out) const;

  std::vector<GridIndex> make26Directions() const;

  std::vector<GridIndex> reconstructPath(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    GridIndex current) const;

  bool startPlan();

  void publishPath(
    const std::vector<GridIndex> & cells,
    const std::string & frame_id);

  double euclidean(const GridIndex & a, const GridIndex & b)
  {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    const double dz = static_cast<double>(a.z - b.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }


private:
 
  std::string source_world_file_;

  double robot_radius_ = 0.25;
  int max_iterations_ = 800000;
  int snap_search_radius_cells_ = 12;
  bool require_ground_support_ = true;
  bool strict_direct_ground_support_ = false;
  int ground_support_xy_radius_cells_ = 1;
  int ground_support_depth_cells_ = 1;
  bool enable_preblocked_costmap_ = true;
  int preblocked_costmap_radius_cells_ = 3;
  double preblocked_costmap_weight_ = 2.5;
  bool lowest_traversable_only_ = false;

  bool map_ready_ = false;
  bool has_start_ = false;
  bool has_goal_ = false;
  bool has_goal_pose_ = false;
  bool planning_in_progress_ = false;

  int num_threads_ = 0;  // OpenMP 线程数，0 = 使用默认值

  std::uint64_t plan_seq_ = 0;
  std::uint64_t last_success_seq_ = 0;
  std::uint64_t last_octomap_hash_ = 0;

  PointPose start_point_;
  PointPose goal_point_;
  PointPose goal_pose_;

  std::vector<PointPose> planner_results_;

  std::shared_ptr<octomap::OcTree> octree_;

  std::unordered_set<GridIndex, GridIndexHash> traversable_cells_;
  std::unordered_set<GridIndex, GridIndexHash> preblocked_cells_;
  std::unordered_set<GridIndex, GridIndexHash> external_preblocked_cells_;
  std::unordered_map<GridIndex, double, GridIndexHash> preblocked_costmap_;
};

}  // namespace global_planner
