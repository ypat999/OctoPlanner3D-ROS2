/**
 * @file      octo_planner/include/global_planner.h
 * @brief     3D A star Planner
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-05-31 12:00:01 
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC). 
 *            All rights reserved.
 */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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

  /** 建立占用缓存：一次遍历八叉树叶节点，后续 isOccupiedCell 变为 O(1) 哈希查询 */
  void buildOccupancyCache();

  /** 设置 OctoMap 并尝试从缓存加载规划层；若缓存不存在则重建并保存 */
  void setOctomapWithCache(
    std::shared_ptr<octomap::OcTree> map,
    const std::string & cache_path);

  /** 设置 OpenMP 并行线程数（0 = 使用默认线程数） */
  void setNumThreads(int n) { num_threads_ = n; }

  /** 设置规划参数（与机器人尺寸和导航行为相关） */
  void setRobotRadius(double radius) { robot_radius_ = radius; }
  void setMaxIterations(int max_iter) { max_iterations_ = max_iter; }
  void setSnapSearchRadiusCells(int radius) { snap_search_radius_cells_ = radius; }
  void setRequireGroundSupport(bool enable) { require_ground_support_ = enable; }
  void setStrictDirectGroundSupport(bool enable) { strict_direct_ground_support_ = enable; }
  void setGroundSupportXYRadiusCells(int radius) { ground_support_xy_radius_cells_ = radius; }
  void setGroundSupportDepthCells(int depth) { ground_support_depth_cells_ = depth; }
  void setEnablePreblockedCostmap(bool enable) { enable_preblocked_costmap_ = enable; }
  void setPreblockedCostmapRadiusCells(int radius) { preblocked_costmap_radius_cells_ = radius; }
  void setPreblockedCostmapWeight(double weight) { preblocked_costmap_weight_ = weight; }
  void setLowestTraversableOnly(bool enable) { lowest_traversable_only_ = enable; }

  // ===== 路径平滑参数 =====
  void setSmoothingEnabled(bool enable) { smoothing_enabled_ = enable; }
  void setSmoothingSimplifyEpsilon(double eps) { smoothing_simplify_epsilon_ = eps; }
  void setSmoothingInterpSpacing(double spacing) { smoothing_interp_spacing_ = spacing; }
  void setSmoothingGradientIterations(int iters) { smoothing_gradient_iters_ = iters; }
  void setSmoothingGradientAlpha(double alpha) { smoothing_gradient_alpha_ = alpha; }
  void setSmoothingCostGradientBeta(double beta) { smoothing_cost_gradient_beta_ = beta; }
  void setSmoothingCostTolerance(double tol) { smoothing_cost_tolerance_ = tol; }
  void setSmoothingMaxStep(double step) { smoothing_max_step_ = step; }
  void setSmoothingZWindowRadius(int r) { smoothing_z_window_radius_ = r; }

  // ===== A* 方向一致性参数 =====
  /** 设置方向变化惩罚权重：进/出方向夹角越大，惩罚越重（0=禁用）
   *  用于减少 A* 在阶跃位置走对角线格子导致的局部曲率过大 */
  void setDirChangeWeight(double w) { dir_change_weight_ = w; }
  /** 设置对角线跨越惩罚：xy 平面对角移动时，若中间轴对齐格子在目标 z 层不可通行，
   *  说明该对角格子是噪声"桥梁"，用于绕过高度变化（0=禁用）
   *  典型场景：斜对角 cell 与本 cell 同高，但正前/左右 cell 低一格 */
  void setDiagonalBridgeWeight(double w) { diagonal_bridge_weight_ = w; }

  /** 对规划结果执行路径平滑 */
  void smoothPath();

  void makePlan(const PointPose start,const PointPose goal);

  void getPlannerResults(std::vector<PointPose>& plannerResults);

  /** 获取可通行层 cost 云数据：每个可通行格子的世界坐标(x,y,z) + cost 值 */
  void getCostCloud(std::vector<std::array<double, 4>> & cloud) const;

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

  void buildTraversableGrid();

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

  // ===== 路径平滑实现 =====
  /** 简化路径：去除共线/近似共线的冗余点（仅当跳过点后的弦仍可通行时才跳过） */
  void simplifyPath(std::vector<PointPose> & path, double epsilon) const;
  /** 碰撞感知的 Catmull-Rom 样条插值：曲线若离开安全走廊则回退到线性插值 */
  void interpolatePath(const std::vector<PointPose> & input, std::vector<PointPose> & output, double spacing) const;
  /** 代价场梯度平滑：拉普拉斯平滑力 + (−∇cost) 排斥力，O(1) 代价门控接受；
   *  迭代中不做碰撞检查；末尾做一次线段安全验证并回退 */
  void gradientDescentSmooth(std::vector<PointPose> & path, int max_iters, double alpha, double beta) const;

  /**
   * @brief z 方向台阶平滑：沿路径做加权滑动平均，消除栅格离散化导致的楼梯状路径
   * @param path 路径点（原地修改）
   * @param window_radius 滑动窗口半径（每侧邻居数）
   */
  void smoothZStairSteps(std::vector<PointPose> & path, int window_radius) const;
  /** 线段碰撞检查：用 3D DDA 遍历线段穿过的所有体素，任一体素不可通行则返回 false */
  bool isSegmentTraversable(const PointPose & a, const PointPose & b) const;
  /** 代价场查询（供平滑优化）：非可通行/越界格返回 1.0，可通行格返回预阻塞代价 */
  double costFieldAt(const GridIndex & idx) const;

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

  // ===== 平滑参数 =====
  bool smoothing_enabled_ = true;
  double smoothing_simplify_epsilon_ = 0.1;      // 简化容忍度（米）
  double smoothing_interp_spacing_ = 0.15;        // 插值点间距（米）
  int smoothing_gradient_iters_ = 50;             // 梯度下降迭代次数
  double smoothing_gradient_alpha_ = 0.3;         // 拉普拉斯平滑步长
  double smoothing_cost_gradient_beta_ = 0.2;     // cost 场梯度排斥步长
  double smoothing_cost_tolerance_ = 0.1;          // 代价门控容差，允许沿等代价线微调
  double smoothing_max_step_ = 0.0;                // 单步位移上限（0=auto=res*0.5）
  int smoothing_z_window_radius_ = 3;              // z 台阶平滑窗口半径

  // ===== A* 方向一致性参数 =====
  double dir_change_weight_ = 1.5;           // 方向变化惩罚权重（0=禁用）
  double diagonal_bridge_weight_ = 5.0;      // 对角线跨越惩罚（0=禁用）

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

  /** 占用缓存：一次构建后 isOccupiedCell 变为 O(1) 哈希查询 */
  std::unordered_set<GridIndex, GridIndexHash> occupancy_cache_;

  std::unordered_set<GridIndex, GridIndexHash> traversable_cells_;
  std::unordered_set<GridIndex, GridIndexHash> preblocked_cells_;
  std::unordered_set<GridIndex, GridIndexHash> external_preblocked_cells_;
  std::unordered_map<GridIndex, double, GridIndexHash> preblocked_costmap_;

  GridIndex grid_min_, grid_max_;
  int64_t grid_dim_x_, grid_dim_y_, grid_dim_z_;
  std::vector<bool> traversable_grid_;
  std::vector<double> preblocked_cost_grid_;

  inline int64_t gridLinear(const GridIndex & idx) const {
    return ((static_cast<int64_t>(idx.x - grid_min_.x) * grid_dim_y_ +
             static_cast<int64_t>(idx.y - grid_min_.y)) * grid_dim_z_ +
            static_cast<int64_t>(idx.z - grid_min_.z));
  }

  inline bool isTraversableGrid(const GridIndex & idx) const {
    if (idx.x < grid_min_.x || idx.x > grid_max_.x ||
        idx.y < grid_min_.y || idx.y > grid_max_.y ||
        idx.z < grid_min_.z || idx.z > grid_max_.z) {
      return false;
    }
    const int64_t lin = gridLinear(idx);
    return lin >= 0 && lin < static_cast<int64_t>(traversable_grid_.size()) && traversable_grid_[lin];
  }

  inline double getPreblockedCostGrid(const GridIndex & idx) const {
    if (idx.x < grid_min_.x || idx.x > grid_max_.x ||
        idx.y < grid_min_.y || idx.y > grid_max_.y ||
        idx.z < grid_min_.z || idx.z > grid_max_.z) {
      return 0.0;
    }
    const int64_t lin = gridLinear(idx);
    return lin >= 0 && lin < static_cast<int64_t>(preblocked_cost_grid_.size()) ? preblocked_cost_grid_[lin] : 0.0;
  }
};

}  // namespace global_planner
