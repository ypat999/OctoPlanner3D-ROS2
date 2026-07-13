/*
 * Nav2 GlobalPlanner plugin wrapping the 3D OctoMap-based planner.
 *
 * This plugin implements nav2_core::GlobalPlanner so that planner_server
 * can load and manage it via pluginlib.  It ignores the 2D costmap and
 * uses its own OctoMap for 3D traversability analysis.
 */

#pragma once

#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "global_planner.h"

namespace octo_planner3d
{

class OctoPlannerGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  OctoPlannerGlobalPlanner() = default;
  ~OctoPlannerGlobalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("OctoPlannerGlobalPlanner")};

  std::shared_ptr<global_planner::GlobalPlanner> planner_;
  std::shared_ptr<octomap::OcTree> octree_;

  // z-fallback: when goal/start z ≈ 0 (typical for RViz 2D Nav Goal),
  // substitute with these configured values.
  double start_z_{0.3};
  double goal_z_{0.3};

  bool is_configured_ = false;

  double start_z_ = 0.3;
  double goal_z_  = 0.3;
};

}  // namespace octo_planner3d
