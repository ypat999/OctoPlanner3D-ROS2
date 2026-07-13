/*
 * Nav2 GlobalPlanner plugin – implementation.
 *
 * The plugin ignores the 2D costmap (costmap_ros) and plans on a 3D OctoMap
 * loaded from a PCD file whose path comes from the parent node's parameters.
 *
 * During configure() the OctoMap is built once (expensive).  createPlan()
 * runs the 3D A* search + optional path smoothing on every call.
 */

#include "octo_planner_global_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "pcd2octomap_converter.h"
#include "pluginlib/class_list_macros.hpp"

namespace octo_planner3d
{

void OctoPlannerGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> /*tf*/,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/)
{
  auto node = parent.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "Parent lifecycle node expired");
    return;
  }

  logger_ = node->get_logger();

  // ----  declare parameters on the lifecycle node  ----
  node->declare_parameter(name + ".input_pcd",
    "/home/ztl/slam_data/3d_map/3dmap.pcd");
  node->declare_parameter(name + ".output_bt", "");
  node->declare_parameter(name + ".resolution", 0.2);
  node->declare_parameter(name + ".min_points_per_voxel", 2);
  node->declare_parameter(name + ".min_cluster_voxels", 2);
  node->declare_parameter(name + ".openmp_num_threads", 0);
  node->declare_parameter(name + ".robot_radius", 0.25);
  node->declare_parameter(name + ".max_iterations", 800000);
  node->declare_parameter(name + ".snap_search_radius_cells", 12);
  node->declare_parameter(name + ".require_ground_support", true);
  node->declare_parameter(name + ".strict_direct_ground_support", false);
  node->declare_parameter(name + ".ground_support_xy_radius_cells", 1);
  node->declare_parameter(name + ".ground_support_depth_cells", 1);
  node->declare_parameter(name + ".enable_preblocked_costmap", true);
  node->declare_parameter(name + ".preblocked_costmap_radius_cells", 3);
  node->declare_parameter(name + ".preblocked_costmap_weight", 2.5);
  node->declare_parameter(name + ".lowest_traversable_only", false);
  node->declare_parameter(name + ".start_z", 0.3);
  node->declare_parameter(name + ".goal_z", 0.3);

  // smoothing parameters
  node->declare_parameter(name + ".smoothing_enabled", true);
  node->declare_parameter(name + ".smoothing_simplify_epsilon", 0.1);
  node->declare_parameter(name + ".smoothing_interp_spacing", 0.15);
  node->declare_parameter(name + ".smoothing_gradient_iterations", 50);
  node->declare_parameter(name + ".smoothing_gradient_alpha", 0.3);

  // ---- read parameter values  ----
  const std::string input_pcd    = node->get_parameter(name + ".input_pcd").as_string();
  const std::string output_bt    = node->get_parameter(name + ".output_bt").as_string();
  const double resolution         = node->get_parameter(name + ".resolution").as_double();
  const int min_points_per_voxel  = static_cast<int>(node->get_parameter(name + ".min_points_per_voxel").as_int());
  const int min_cluster_voxels    = static_cast<int>(node->get_parameter(name + ".min_cluster_voxels").as_int());
  const int openmp_num_threads    = static_cast<int>(node->get_parameter(name + ".openmp_num_threads").as_int());
  const double robot_radius       = node->get_parameter(name + ".robot_radius").as_double();
  const int max_iterations        = static_cast<int>(node->get_parameter(name + ".max_iterations").as_int());
  const int snap_search_radius_cells = static_cast<int>(node->get_parameter(name + ".snap_search_radius_cells").as_int());
  const bool require_ground_support  = node->get_parameter(name + ".require_ground_support").as_bool();
  const bool strict_direct_ground_support = node->get_parameter(name + ".strict_direct_ground_support").as_bool();
  const int gs_xy_radius    = static_cast<int>(node->get_parameter(name + ".ground_support_xy_radius_cells").as_int());
  const int gs_depth         = static_cast<int>(node->get_parameter(name + ".ground_support_depth_cells").as_int());
  const bool enable_preblock = node->get_parameter(name + ".enable_preblocked_costmap").as_bool();
  const int preblock_radius  = static_cast<int>(node->get_parameter(name + ".preblocked_costmap_radius_cells").as_int());
  const double preblock_w    = node->get_parameter(name + ".preblocked_costmap_weight").as_double();
  const bool lowest_only     = node->get_parameter(name + ".lowest_traversable_only").as_bool();

  start_z_ = node->get_parameter(name + ".start_z").as_double();
  goal_z_  = node->get_parameter(name + ".goal_z").as_double();

  const bool smoothing_enabled         = node->get_parameter(name + ".smoothing_enabled").as_bool();
  const double smoothing_simplify_eps  = node->get_parameter(name + ".smoothing_simplify_epsilon").as_double();
  const double smoothing_interp_space  = node->get_parameter(name + ".smoothing_interp_spacing").as_double();
  const int smoothing_grad_iters       = static_cast<int>(node->get_parameter(name + ".smoothing_gradient_iterations").as_int());
  const double smoothing_grad_alpha    = node->get_parameter(name + ".smoothing_gradient_alpha").as_double();

  // ---- build OctoMap from PCD  ----
  RCLCPP_INFO(logger_, "Building OctoMap from %s ...", input_pcd.c_str());
  auto converter = std::make_shared<pcd2octomap::Pcd2OctomapConverter>();
  converter->setInputPcdFile(input_pcd);
  if (!output_bt.empty()) {
    converter->setOutputBtFile(output_bt);
  }
  converter->setResolution(resolution);
  converter->setMinPointsPerVoxel(min_points_per_voxel);
  converter->setMinClusterVoxels(min_cluster_voxels);

  if (!converter->convert()) {
    RCLCPP_ERROR(logger_, "Failed to build OctoMap from PCD");
    return;
  }

  octree_ = converter->getOctomap();

  // ---- create the planner engine  ----
  planner_ = std::make_shared<global_planner::GlobalPlanner>();
  planner_->setNumThreads(openmp_num_threads);
  planner_->setRobotRadius(robot_radius);
  planner_->setMaxIterations(max_iterations);
  planner_->setSnapSearchRadiusCells(snap_search_radius_cells);
  planner_->setRequireGroundSupport(require_ground_support);
  planner_->setStrictDirectGroundSupport(strict_direct_ground_support);
  planner_->setGroundSupportXYRadiusCells(gs_xy_radius);
  planner_->setGroundSupportDepthCells(gs_depth);
  planner_->setEnablePreblockedCostmap(enable_preblock);
  planner_->setPreblockedCostmapRadiusCells(preblock_radius);
  planner_->setPreblockedCostmapWeight(preblock_w);
  planner_->setLowestTraversableOnly(lowest_only);

  planner_->setSmoothingEnabled(smoothing_enabled);
  planner_->setSmoothingSimplifyEpsilon(smoothing_simplify_eps);
  planner_->setSmoothingInterpSpacing(smoothing_interp_space);
  planner_->setSmoothingGradientIterations(smoothing_grad_iters);
  planner_->setSmoothingGradientAlpha(smoothing_grad_alpha);

  const std::string cache_path =
    (output_bt.empty() ? input_pcd : output_bt) + "_planning_cache";
  planner_->setOctomapWithCache(octree_, cache_path);

  is_configured_ = true;
  RCLCPP_INFO(logger_, "OctoPlannerGlobalPlanner configured successfully");
}

void OctoPlannerGlobalPlanner::activate()
{
  RCLCPP_INFO(logger_, "OctoPlannerGlobalPlanner activated");
}

void OctoPlannerGlobalPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "OctoPlannerGlobalPlanner deactivated");
}

void OctoPlannerGlobalPlanner::cleanup()
{
  planner_.reset();
  octree_.reset();
  is_configured_ = false;
  RCLCPP_INFO(logger_, "OctoPlannerGlobalPlanner cleaned up");
}

nav_msgs::msg::Path OctoPlannerGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path plan;
  plan.header.frame_id = start.header.frame_id;
  plan.header.stamp = rclcpp::Clock().now();

  if (!is_configured_ || !planner_) {
    RCLCPP_ERROR(logger_, "Planner not configured");
    return plan;
  }

  global_planner::PointPose sp, gp;
  sp.x = start.pose.position.x;
  sp.y = start.pose.position.y;
  sp.z = start.pose.position.z + start_z_;
  gp.x = goal.pose.position.x;
  gp.y = goal.pose.position.y;
  gp.z = goal.pose.position.z + goal_z_;

  RCLCPP_INFO(logger_, "Planning [%.2f,%.2f,%.2f] -> [%.2f,%.2f,%.2f]",
              sp.x, sp.y, sp.z, gp.x, gp.y, gp.z);

  const auto t0 = std::chrono::steady_clock::now();

  planner_->makePlan(sp, gp);
  planner_->smoothPath();

  std::vector<global_planner::PointPose> raw;
  planner_->getPlannerResults(raw);

  const double elapsed =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

  if (raw.empty()) {
    RCLCPP_WARN(logger_, "Planner returned empty path (%.2f s)", elapsed);
    return plan;
  }

  // Build nav_msgs::Path with orientation interpolation
  plan.poses.reserve(raw.size());
  for (size_t i = 0; i < raw.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = plan.header;
    pose.pose.position.x = raw[i].x;
    pose.pose.position.y = raw[i].y;
    pose.pose.position.z = raw[i].z;

    if (i < raw.size() - 1) {
      double yaw = std::atan2(raw[i + 1].y - raw[i].y,
                              raw[i + 1].x - raw[i].x);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation = tf2::toMsg(q);
    } else if (i > 0) {
      pose.pose.orientation = plan.poses[i - 1].pose.orientation;
    } else {
      pose.pose.orientation.w = 1.0;
    }
    plan.poses.push_back(pose);
  }

  RCLCPP_INFO(logger_, "Planned %zu poses in %.2f s", raw.size(), elapsed);
  return plan;
}

}  // namespace octo_planner3d

PLUGINLIB_EXPORT_CLASS(
  octo_planner3d::OctoPlannerGlobalPlanner,
  nav2_core::GlobalPlanner)
