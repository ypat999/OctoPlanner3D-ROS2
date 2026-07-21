#include "global_planner.h"
#include "pcd2octomap_converter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav2_msgs/action/compute_path_to_pose.hpp>
#include <nav2_msgs/action/compute_path_through_poses.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


// TF2 用于获取机器人位置
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{

constexpr double kZeroZThreshold = 1.0e-6;

struct CachedVoxel
{
  float x, y, z;
  float size;
};

std::string defaultInputPcd()
{
  return "/home/ztl/slam_data/3d_map/3dmap.pcd";
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

global_planner::PointPose toPlannerPoint(
  const geometry_msgs::msg::Pose & pose,
  double fallback_z)
{
  global_planner::PointPose point;
  point.x = pose.position.x;
  point.y = pose.position.y;
  point.z = std::abs(pose.position.z) > kZeroZThreshold ? pose.position.z : fallback_z;
  return point;
}

global_planner::PointPose toPlannerPoint(const geometry_msgs::msg::Point & point_msg)
{
  global_planner::PointPose point;
  point.x = point_msg.x;
  point.y = point_msg.y;
  point.z = point_msg.z;
  return point;
}

}  // namespace

class OctoPlannerRvizNode : public rclcpp::Node
{
public:
  OctoPlannerRvizNode()
  : Node("octo_planner_rviz_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    start_z_ = declare_parameter<double>("start_z", 0.3);
    goal_z_ = declare_parameter<double>("goal_z", 0.3);
    map_alpha_ = declare_parameter<double>("map_alpha", 0.82);
    map_color_mode_ = declare_parameter<std::string>("map_color_mode", "height");
    const std::string clicked_point_topic =
      declare_parameter<std::string>("clicked_point_topic", "clicked_point");
    const std::string input_pcd = declare_parameter<std::string>("input_pcd", defaultInputPcd());
    const std::string output_bt =
      declare_parameter<std::string>("output_bt", "");  // 空 = 由 PCD 文件名自动推导
    const double resolution = declare_parameter<double>("resolution", 0.2);
    const int min_points_per_voxel = declare_parameter<int>("min_points_per_voxel", 2);
    const int min_cluster_voxels = declare_parameter<int>("min_cluster_voxels", 2);
    const double map_publish_period =
      declare_parameter<double>("map_publish_period", 0.0);
    const int openmp_num_threads =
      declare_parameter<int>("openmp_num_threads", 0);

    // nav2 集成参数（自动获取机器人位置作为起点）
    robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_footprint");
    transform_timeout_ = declare_parameter<double>("transform_timeout", 0.5);
    path_orientation_mode_ = declare_parameter<std::string>("path_orientation_mode", "interpolate");
    planned_path_topic_ = declare_parameter<std::string>("planned_path_topic", "plan");

    // 规划参数（与机器人尺寸和导航行为相关）
    const double robot_radius = declare_parameter<double>("robot_radius", 0.25);
    const int max_iterations = declare_parameter<int>("max_iterations", 800000);
    const int snap_search_radius_cells = declare_parameter<int>("snap_search_radius_cells", 12);
    const bool require_ground_support = declare_parameter<bool>("require_ground_support", true);
    const bool strict_direct_ground_support = declare_parameter<bool>("strict_direct_ground_support", false);
    const int ground_support_xy_radius_cells = declare_parameter<int>("ground_support_xy_radius_cells", 1);
    const int ground_support_depth_cells = declare_parameter<int>("ground_support_depth_cells", 1);
    const bool enable_preblocked_costmap = declare_parameter<bool>("enable_preblocked_costmap", true);
    const int preblocked_costmap_radius_cells = declare_parameter<int>("preblocked_costmap_radius_cells", 3);
    const double preblocked_costmap_weight = declare_parameter<double>("preblocked_costmap_weight", 2.5);
    const bool lowest_traversable_only = declare_parameter<bool>("lowest_traversable_only", false);

    // 平滑参数
    const bool smoothing_enabled = declare_parameter<bool>("smoothing_enabled", true);
    const double smoothing_simplify_epsilon =
      declare_parameter<double>("smoothing_simplify_epsilon", 0.1);
    const double smoothing_interp_spacing =
      declare_parameter<double>("smoothing_interp_spacing", 0.15);
    const int smoothing_gradient_iterations =
      declare_parameter<int>("smoothing_gradient_iterations", 50);
    const double smoothing_gradient_alpha =
      declare_parameter<double>("smoothing_gradient_alpha", 0.3);
    const double smoothing_cost_gradient_beta =
      declare_parameter<double>("smoothing_cost_gradient_beta", 0.2);
    const double smoothing_cost_tolerance =
      declare_parameter<double>("smoothing_cost_tolerance", 0.1);
    const double smoothing_max_step =
      declare_parameter<double>("smoothing_max_step", 0.0);
    const int smoothing_z_window_radius =
      declare_parameter<int>("smoothing_z_window_radius", 3);

    // A* 方向一致性参数
    const double dir_change_weight =
      declare_parameter<double>("dir_change_weight", 2.0);
    const double diagonal_bridge_weight =
      declare_parameter<double>("diagonal_bridge_weight", 5.0);

    converter_ = std::make_shared<pcd2octomap::Pcd2OctomapConverter>();
    converter_->setInputPcdFile(input_pcd);
    if (!output_bt.empty()) {
      converter_->setOutputBtFile(output_bt);
    }
    converter_->setResolution(resolution);
    converter_->setMinPointsPerVoxel(min_points_per_voxel);
    converter_->setMinClusterVoxels(min_cluster_voxels);
    planner_ = std::make_shared<global_planner::GlobalPlanner>();
    planner_->setNumThreads(openmp_num_threads);

    // 设置规划参数
    planner_->setRobotRadius(robot_radius);
    planner_->setMaxIterations(max_iterations);
    planner_->setSnapSearchRadiusCells(snap_search_radius_cells);
    planner_->setRequireGroundSupport(require_ground_support);
    planner_->setStrictDirectGroundSupport(strict_direct_ground_support);
    planner_->setGroundSupportXYRadiusCells(ground_support_xy_radius_cells);
    planner_->setGroundSupportDepthCells(ground_support_depth_cells);
    planner_->setEnablePreblockedCostmap(enable_preblocked_costmap);
    planner_->setPreblockedCostmapRadiusCells(preblocked_costmap_radius_cells);
    planner_->setPreblockedCostmapWeight(preblocked_costmap_weight);
    planner_->setLowestTraversableOnly(lowest_traversable_only);

    // 设置平滑参数
    planner_->setSmoothingEnabled(smoothing_enabled);
    planner_->setSmoothingSimplifyEpsilon(smoothing_simplify_epsilon);
    planner_->setSmoothingInterpSpacing(smoothing_interp_spacing);
    planner_->setSmoothingGradientIterations(smoothing_gradient_iterations);
    planner_->setSmoothingGradientAlpha(smoothing_gradient_alpha);
    planner_->setSmoothingCostGradientBeta(smoothing_cost_gradient_beta);
    planner_->setSmoothingCostTolerance(smoothing_cost_tolerance);
    planner_->setSmoothingMaxStep(smoothing_max_step);
    planner_->setSmoothingZWindowRadius(smoothing_z_window_radius);

    // 设置 A* 方向一致性参数
    planner_->setDirChangeWeight(dir_change_weight);
    planner_->setDiagonalBridgeWeight(diagonal_bridge_weight);

    // ===== 参数运行时修改回调（支持 ros2 param set 实时调整） =====
    param_handler_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & p : params) {
          const auto & name = p.get_name();
          if (name == "robot_radius") planner_->setRobotRadius(p.as_double());
          else if (name == "max_iterations") planner_->setMaxIterations(p.as_int());
          else if (name == "snap_search_radius_cells") planner_->setSnapSearchRadiusCells(p.as_int());
          else if (name == "require_ground_support") planner_->setRequireGroundSupport(p.as_bool());
          else if (name == "strict_direct_ground_support") planner_->setStrictDirectGroundSupport(p.as_bool());
          else if (name == "ground_support_xy_radius_cells") planner_->setGroundSupportXYRadiusCells(p.as_int());
          else if (name == "ground_support_depth_cells") planner_->setGroundSupportDepthCells(p.as_int());
          else if (name == "enable_preblocked_costmap") planner_->setEnablePreblockedCostmap(p.as_bool());
          else if (name == "preblocked_costmap_radius_cells") planner_->setPreblockedCostmapRadiusCells(p.as_int());
          else if (name == "preblocked_costmap_weight") planner_->setPreblockedCostmapWeight(p.as_double());
          else if (name == "lowest_traversable_only") planner_->setLowestTraversableOnly(p.as_bool());
          else if (name == "smoothing_enabled") planner_->setSmoothingEnabled(p.as_bool());
          else if (name == "smoothing_simplify_epsilon") planner_->setSmoothingSimplifyEpsilon(p.as_double());
          else if (name == "smoothing_interp_spacing") planner_->setSmoothingInterpSpacing(p.as_double());
          else if (name == "smoothing_gradient_iterations") planner_->setSmoothingGradientIterations(p.as_int());
          else if (name == "smoothing_gradient_alpha") planner_->setSmoothingGradientAlpha(p.as_double());
          else if (name == "smoothing_cost_gradient_beta") planner_->setSmoothingCostGradientBeta(p.as_double());
          else if (name == "smoothing_cost_tolerance") planner_->setSmoothingCostTolerance(p.as_double());
          else if (name == "smoothing_max_step") planner_->setSmoothingMaxStep(p.as_double());
          else if (name == "smoothing_z_window_radius") planner_->setSmoothingZWindowRadius(p.as_int());
          else if (name == "dir_change_weight") planner_->setDirChangeWeight(p.as_double());
          else if (name == "diagonal_bridge_weight") planner_->setDiagonalBridgeWeight(p.as_double());
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    // ===== Action Server: 在 OctoMap 构建前注册，确保 bt_navigator 启动时可见 =====
    using nav2_msgs::action::ComputePathToPose;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ComputePathToPose>;
    action_server_ = rclcpp_action::create_server<ComputePathToPose>(
      this, "compute_path_to_pose",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ComputePathToPose::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](const std::shared_ptr<GoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const std::shared_ptr<GoalHandle> h) { executeComputePathToPose(h); });
    RCLCPP_INFO(get_logger(), "ComputePathToPose action server ready");

    using nav2_msgs::action::ComputePathThroughPoses;
    using PathThroughGoalHandle = rclcpp_action::ServerGoalHandle<ComputePathThroughPoses>;
    action_through_poses_server_ = rclcpp_action::create_server<ComputePathThroughPoses>(
      this, "compute_path_through_poses",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ComputePathThroughPoses::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](const std::shared_ptr<PathThroughGoalHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const std::shared_ptr<PathThroughGoalHandle> h) { executeComputePathThroughPoses(h); });
    RCLCPP_INFO(get_logger(), "ComputePathThroughPoses action server ready");

    RCLCPP_INFO(get_logger(), "Building OctoMap from configured PCD file...");
    if (!converter_->convert()) {
      RCLCPP_ERROR(get_logger(), "Failed to build OctoMap. Node will stay alive for diagnostics.");
      return;
    }

    octree_ = converter_->getOctomap();
    planner_->setOctomapWithCache(octree_, converter_->getOutputBtFile() + "_planning_cache");

    // 预计算地图可视化缓存（遍历八叉树最耗时，只做一次）
    {
      const std::string voxel_cache =
        converter_->getOutputBtFile() + "_voxels";
      loadVoxelCache(voxel_cache);
      if (!cached_voxels_.empty()) {
        RCLCPP_INFO(get_logger(), "Loaded %zu cached voxels from %s",
                    cached_voxels_.size(), voxel_cache.c_str());
      }
      if (cached_voxels_.empty()) {
        preCacheMapData();
        saveVoxelCache(voxel_cache);
        RCLCPP_INFO(get_logger(), "Saved %zu voxels to %s",
                    cached_voxels_.size(), voxel_cache.c_str());
      }
    }

    const auto transient_qos = rclcpp::QoS(1).transient_local().reliable();

    // 地图可视化发布器
    map_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("occupied_map", transient_qos);
    map_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("occupied_map_cloud", transient_qos);
    cost_map_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("cost_map_cloud", transient_qos);

    // 导航路径发布器（用于Nav2执行）
    path_pub_ = create_publisher<nav_msgs::msg::Path>(planned_path_topic_, transient_qos);
    path_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("planned_path_marker", transient_qos);
    nav_start_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("nav_start_marker", transient_qos);
    // 测试目标Marker可视化
    test_path_pub_ = create_publisher<nav_msgs::msg::Path>("test_path", transient_qos);
    test_path_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("test_path_marker", transient_qos);
    test_start_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("test_start_marker", transient_qos);
    test_goal_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("test_goal_marker", transient_qos);

    // 订阅器：导航模式（initialpose 设置起点，通过 compute_path_to_pose action 接收目标）
    start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose",
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onNavStartPose, this, std::placeholders::_1));
    // 订阅器：测试模式（RViz Publish Point）
    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_point_topic,
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onTestClickedPoint, this, std::placeholders::_1));

    // 初始化 TF2（用于自动获取机器人位置）
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_INFO(get_logger(), "TF2 initialized.");

    publishMap();
    if (map_publish_period > 0.0) {
      map_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(map_publish_period)),
        std::bind(&OctoPlannerRvizNode::publishMap, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "Ready. Use RViz2 Publish Point on %s for test path.",
      clicked_point_topic.c_str());
  }

private:
  // ===== 导航模式回调 =====
  void onNavStartPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    nav_start_ = toPlannerPoint(msg->pose.pose, start_z_);
    has_nav_start_ = true;
    publishPoseMarker(nav_start_, "nav_start", 0, makeColor(0.1F, 0.9F, 0.2F, 1.0F), nav_start_marker_pub_);
    RCLCPP_INFO(
      get_logger(),
      "[Nav Mode] Start set to [%.3f, %.3f, %.3f] (planning via ComputePathToPose action)",
      nav_start_.x,
      nav_start_.y,
      nav_start_.z);
    // 不再调用 planNavPath()：导航规划由 ComputePathToPose action server 统一处理
  }

  // ===== 测试模式回调 =====
  void onTestClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (next_clicked_point_is_start_) {
      test_start_ = toPlannerPoint(msg->point);
      has_test_start_ = true;
      has_test_goal_ = false;
      next_clicked_point_is_start_ = false;
      publishPoseMarker(test_start_, "test_start", 0, makeColor(0.3F, 0.5F, 0.9F, 1.0F), test_start_marker_pub_);
      RCLCPP_INFO(
        get_logger(),
        "[Test Mode] Start point set to [%.3f, %.3f, %.3f]. Publish next point as goal.",
        test_start_.x,
        test_start_.y,
        test_start_.z);
      return;
    }

    test_goal_ = toPlannerPoint(msg->point);
    has_test_goal_ = true;
    next_clicked_point_is_start_ = true;
    publishPoseMarker(test_goal_, "test_goal", 0, makeColor(0.9F, 0.5F, 0.3F, 1.0F), test_goal_marker_pub_);
    RCLCPP_INFO(
      get_logger(),
      "[Test Mode] Goal point set to [%.3f, %.3f, %.3f]. Planning test path.",
      test_goal_.x,
      test_goal_.y,
      test_goal_.z);
    planTestPath();
  }

  void preCacheMapData()
  {
    cached_voxels_.clear();
    if (!octree_) {
      return;
    }

    RCLCPP_INFO(get_logger(), "Pre-caching map voxel data...");
    const auto t_start = std::chrono::steady_clock::now();

    const size_t total_leafs = octree_->getNumLeafNodes();
    const size_t report_interval = std::max(size_t(1), total_leafs / 10);
    size_t processed = 0;

    const double res = octree_->getResolution();
    const double eps = res * 1e-6;

    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it)) {
        continue;
      }
      double size = it.getSize();
      if (size <= res * 1.01) {
        // 细叶子：直接存
        CachedVoxel voxel;
        voxel.x = static_cast<float>(it.getX());
        voxel.y = static_cast<float>(it.getY());
        voxel.z = static_cast<float>(it.getZ());
        voxel.size = static_cast<float>(size);
        cached_voxels_.push_back(voxel);
      } else {
        // 粗叶子：展开为细格子存入
        double half = size * 0.5;
        for (double cx = it.getX() - half + res * 0.5; cx <= it.getX() + half - eps; cx += res) {
          for (double cy = it.getY() - half + res * 0.5; cy <= it.getY() + half - eps; cy += res) {
            for (double cz = it.getZ() - half + res * 0.5; cz <= it.getZ() + half - eps; cz += res) {
              CachedVoxel voxel;
              voxel.x = static_cast<float>(cx);
              voxel.y = static_cast<float>(cy);
              voxel.z = static_cast<float>(cz);
              voxel.size = static_cast<float>(res);
              cached_voxels_.push_back(voxel);
            }
          }
        }
      }

      ++processed;
      if (processed % report_interval == 0) {
        const int pct = static_cast<int>(processed * 100 / total_leafs);
        const double elapsed =
          std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t_start).count();
        RCLCPP_INFO(get_logger(),
          "Pre-caching: %d%% (%zu / %zu nodes, %zu occupied voxels, %.1f s)",
          pct, processed, total_leafs, cached_voxels_.size(), elapsed);
      }
    }

    const double total_s =
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    RCLCPP_INFO(get_logger(), "Pre-cached %zu occupied voxels in %.1f s.",
                cached_voxels_.size(), total_s);
  }

  void saveVoxelCache(const std::string & cache_file) const
  {
    if (cached_voxels_.empty()) {
      return;
    }
    std::ofstream f(cache_file, std::ios::binary);
    if (!f.is_open()) {
      return;
    }
    const uint32_t version = 2;
    const double resolution = octree_->getResolution();
    f.write(reinterpret_cast<const char *>(&version), sizeof(version));
    f.write(reinterpret_cast<const char *>(&resolution), sizeof(resolution));
    const uint64_t count = cached_voxels_.size();
    f.write(reinterpret_cast<const char *>(&count), sizeof(count));
    f.write(reinterpret_cast<const char *>(cached_voxels_.data()),
            count * sizeof(CachedVoxel));
  }

  void loadVoxelCache(const std::string & cache_file)
  {
    cached_voxels_.clear();
    std::ifstream f(cache_file, std::ios::binary);
    if (!f.is_open()) {
      return;
    }
    uint32_t version = 0;
    f.read(reinterpret_cast<char *>(&version), sizeof(version));
    if (!f || version == 0) {
      return;
    }
    if (version >= 2) {
      double stored_resolution = 0.0;
      f.read(reinterpret_cast<char *>(&stored_resolution), sizeof(stored_resolution));
      if (!f) return;
      const double current_resolution = octree_->getResolution();
      if (std::abs(stored_resolution - current_resolution) > 1e-6) {
        RCLCPP_WARN(get_logger(), "Voxel cache resolution mismatch (%.4f vs %.4f), discarding.",
                    stored_resolution, current_resolution);
        return;
      }
    }
    uint64_t count = 0;
    f.read(reinterpret_cast<char *>(&count), sizeof(count));
    if (!f || count == 0) {
      return;
    }
    cached_voxels_.resize(count);
    f.read(reinterpret_cast<char *>(cached_voxels_.data()),
           count * sizeof(CachedVoxel));
    if (!f) {
      cached_voxels_.clear();
    }
  }

  // ===== 导航路径规划 =====
  void planNavPath()
  {
    if (!planner_ || !octree_ || !has_nav_start_ || !has_nav_goal_) {
      return;
    }

    RCLCPP_INFO(get_logger(), "[Nav Mode] Planning from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]...",
                nav_start_.x, nav_start_.y, nav_start_.z, nav_goal_.x, nav_goal_.y, nav_goal_.z);
    const auto t_start = std::chrono::steady_clock::now();
    planner_->makePlan(nav_start_, nav_goal_);
    const double plan_elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    planner_->smoothPath();

    std::vector<global_planner::PointPose> path;
    planner_->getPlannerResults(path);
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "[Nav Mode] Planner returned an empty path (%.2f s).", plan_elapsed);
      publishNavPath(path);
      return;
    }

    publishNavPath(path);
    publishCostCloud();
    RCLCPP_INFO(get_logger(), "[Nav Mode] Published nav path with %zu poses (%.2f s).",
                path.size(), plan_elapsed);
  }

  // ===== 测试路径规划 =====
  void planTestPath()
  {
    if (!planner_ || !octree_ || !has_test_start_ || !has_test_goal_) {
      return;
    }

    RCLCPP_INFO(get_logger(), "[Test Mode] Planning from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]...",
                test_start_.x, test_start_.y, test_start_.z, test_goal_.x, test_goal_.y, test_goal_.z);
    const auto t_start = std::chrono::steady_clock::now();
    planner_->makePlan(test_start_, test_goal_);
    planner_->smoothPath();
    const double plan_elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    std::vector<global_planner::PointPose> path;
    planner_->getPlannerResults(path);
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "[Test Mode] Planner returned an empty path (%.2f s).", plan_elapsed);
      publishTestPath(path);
      return;
    }

    publishTestPath(path);
    publishCostCloud();
    RCLCPP_INFO(get_logger(), "[Test Mode] Published test path with %zu poses (%.2f s).", path.size(), plan_elapsed);
  }

  // ===== 从 TF 获取机器人当前位姿 =====
  bool getRobotPose(global_planner::PointPose & pose)
  {
    if (!tf_buffer_) {
      RCLCPP_ERROR(get_logger(), "TF buffer not ready yet");
      return false;
    }
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(frame_id_, robot_base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR(get_logger(), "TF lookup(%s->%s) failed: %s",
                   frame_id_.c_str(), robot_base_frame_.c_str(), e.what());
      return false;
    }
    pose.x = t.transform.translation.x;
    pose.y = t.transform.translation.y;
    pose.z = t.transform.translation.z;
    return true;
  }

  // ===== Nav2 ComputePathToPose action 处理 =====
  void executeComputePathToPose(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::ComputePathToPose>> handle)
  {
    const auto goal = handle->get_goal();

    auto result = std::make_shared<nav2_msgs::action::ComputePathToPose::Result>();
    result->path.header.frame_id = frame_id_;
    result->path.header.stamp = now();

    if (!planner_ || !octree_) {
      RCLCPP_ERROR(get_logger(), "Planner/OctoMap not ready");
      handle->abort(result);
      return;
    }

    // start from TF, goal from action request
    global_planner::PointPose sp;
    if (!getRobotPose(sp)) {
      handle->abort(result);
      return;
    }

    global_planner::PointPose gp;
    gp.x = goal->goal.pose.position.x;
    gp.y = goal->goal.pose.position.y;
    gp.z = goal->goal.pose.position.z + goal_z_;

    RCLCPP_INFO(get_logger(), "[Nav2 Action] Planning [%.2f,%.2f,%.2f] -> [%.2f,%.2f,%.2f]",
                sp.x, sp.y, sp.z, gp.x, gp.y, gp.z);

    const auto t0 = std::chrono::steady_clock::now();
    planner_->makePlan(sp, gp);
    planner_->smoothPath();

    std::vector<global_planner::PointPose> raw;
    planner_->getPlannerResults(raw);
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    if (raw.empty()) {
      RCLCPP_WARN(get_logger(), "[Nav2 Action] Empty path (%.2f s)", elapsed);
      handle->abort(result);
      return;
    }

    // build nav_msgs::Path with orientation interpolation
    result->path.poses.reserve(raw.size());
    for (size_t i = 0; i < raw.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = result->path.header;
      pose.pose.position.x = raw[i].x;
      pose.pose.position.y = raw[i].y;
      pose.pose.position.z = raw[i].z;
      if (i < raw.size() - 1) {
        double yaw = std::atan2(raw[i + 1].y - raw[i].y, raw[i + 1].x - raw[i].x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);
      } else {
        // 最后一个点使用 goal 的目标朝向
        pose.pose.orientation = goal->goal.pose.orientation;
      }
      result->path.poses.push_back(pose);
    }

    // publish to /plan for RViz
    nav_msgs::msg::Path viz_path = result->path;
    path_pub_->publish(viz_path);

    handle->succeed(result);
    RCLCPP_INFO(get_logger(), "[Nav2 Action] Planned %zu poses in %.2f s", raw.size(), elapsed);
  }

  // ===== Nav2 ComputePathThroughPoses action 处理（复用同一 planner）=====
  void executeComputePathThroughPoses(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::ComputePathThroughPoses>> handle)
  {
    const auto goal = handle->get_goal();
    if (goal->goals.empty()) {
      handle->abort(std::make_shared<nav2_msgs::action::ComputePathThroughPoses::Result>());
      return;
    }
    // 使用最后一个途经点作为目标，行为与 ComputePathToPose 一致
    const auto & gpose = goal->goals.back().pose;

    auto result = std::make_shared<nav2_msgs::action::ComputePathThroughPoses::Result>();
    result->path.header.frame_id = frame_id_;
    result->path.header.stamp = now();

    if (!planner_ || !octree_) {
      RCLCPP_ERROR(get_logger(), "Planner/OctoMap not ready");
      handle->abort(result);
      return;
    }

    global_planner::PointPose sp;
    if (!getRobotPose(sp)) {
      handle->abort(result);
      return;
    }

    global_planner::PointPose gp;
    gp.x = gpose.position.x;
    gp.y = gpose.position.y;
    gp.z = gpose.position.z + goal_z_;

    RCLCPP_INFO(get_logger(), "[Nav2 ThroughPoses] Planning [%.2f,%.2f,%.2f] -> [%.2f,%.2f,%.2f]",
                sp.x, sp.y, sp.z, gp.x, gp.y, gp.z);

    const auto t0 = std::chrono::steady_clock::now();
    planner_->makePlan(sp, gp);
    planner_->smoothPath();

    std::vector<global_planner::PointPose> raw;
    planner_->getPlannerResults(raw);
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    if (raw.empty()) {
      RCLCPP_WARN(get_logger(), "[Nav2 ThroughPoses] Empty path (%.2f s)", elapsed);
      handle->abort(result);
      return;
    }

    result->path.poses.reserve(raw.size());
    for (size_t i = 0; i < raw.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = result->path.header;
      pose.pose.position.x = raw[i].x;
      pose.pose.position.y = raw[i].y;
      pose.pose.position.z = raw[i].z;
      if (i < raw.size() - 1) {
        double yaw = std::atan2(raw[i + 1].y - raw[i].y, raw[i + 1].x - raw[i].x);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);
      } else {
        // 最后一个点使用 goal 的目标朝向
        pose.pose.orientation = gpose.orientation;
      }
      result->path.poses.push_back(pose);
    }

    path_pub_->publish(result->path);
    handle->succeed(result);
    RCLCPP_INFO(get_logger(), "[Nav2 ThroughPoses] Planned %zu poses in %.2f s", raw.size(), elapsed);
  }

  void publishMap()
  {
    if (!octree_ || !map_pub_ || cached_voxels_.empty()) {
      return;
    }

    const auto t_start = std::chrono::steady_clock::now();

    double min_x = cached_voxels_[0].x;
    double min_y = cached_voxels_[0].y;
    double min_z = cached_voxels_[0].z;
    double max_x = min_x;
    double max_y = min_y;
    double max_z = min_z;

    for (const auto & v : cached_voxels_) {
      if (v.x < min_x) min_x = v.x;
      if (v.y < min_y) min_y = v.y;
      if (v.z < min_z) min_z = v.z;
      if (v.x > max_x) max_x = v.x;
      if (v.y > max_y) max_y = v.y;
      if (v.z > max_z) max_z = v.z;
    }

    const double z_range = std::max(1.0e-6, max_z - min_z);
    const float alpha = static_cast<float>(std::clamp(map_alpha_, 0.05, 1.0));

    RCLCPP_INFO(get_logger(), "Publishing map (%zu voxels)...", cached_voxels_.size());

    std::unordered_map<float, visualization_msgs::msg::Marker> markers_by_size;
    std::vector<geometry_msgs::msg::Point> cloud_points;
    cloud_points.reserve(cached_voxels_.size());

    for (const auto & v : cached_voxels_) {
      auto marker_it = markers_by_size.find(v.size);
      if (marker_it == markers_by_size.end()) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.ns = "occupied_voxels";
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = v.size;
        marker.scale.y = v.size;
        marker.scale.z = v.size;
        marker.color = makeColor(0.45F, 0.22F, 0.06F, alpha);
        marker_it = markers_by_size.emplace(v.size, std::move(marker)).first;
      }

      auto pt = makePoint(v.x, v.y, v.z);
      marker_it->second.points.push_back(pt);
      cloud_points.push_back(pt);
    }

    {
      visualization_msgs::msg::MarkerArray array;
      int id = 0;
      for (auto & entry : markers_by_size) {
        auto & marker = entry.second;
        marker.header.stamp = now();
        marker.colors.resize(marker.points.size());
        for (size_t i = 0; i < marker.points.size(); ++i) {
          const double t = (marker.points[i].z - min_z) / z_range;
          marker.colors[i] = heightColor(t, alpha);
        }
        marker.id = id++;
        array.markers.push_back(std::move(marker));
      }

      visualization_msgs::msg::Marker cleanup;
      cleanup.header.frame_id = frame_id_;
      cleanup.header.stamp = now();
      cleanup.ns = "occupied_voxels_cleanup";
      cleanup.id = 0;
      cleanup.action = visualization_msgs::msg::Marker::DELETEALL;
      array.markers.insert(array.markers.begin(), cleanup);

      map_pub_->publish(array);
    }

    if (map_cloud_pub_ && !cloud_points.empty()) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      cloud_msg.header.frame_id = frame_id_;
      cloud_msg.header.stamp = now();
      cloud_msg.height = 1;
      cloud_msg.width = static_cast<uint32_t>(cloud_points.size());
      cloud_msg.is_bigendian = false;
      cloud_msg.is_dense = true;

      cloud_msg.fields.resize(4);
      cloud_msg.fields[0].name = "x";
      cloud_msg.fields[0].offset = 0;
      cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg.fields[0].count = 1;
      cloud_msg.fields[1].name = "y";
      cloud_msg.fields[1].offset = 4;
      cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg.fields[1].count = 1;
      cloud_msg.fields[2].name = "z";
      cloud_msg.fields[2].offset = 8;
      cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg.fields[2].count = 1;
      cloud_msg.fields[3].name = "intensity";
      cloud_msg.fields[3].offset = 12;
      cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg.fields[3].count = 1;

      cloud_msg.point_step = 16;
      cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
      cloud_msg.data.resize(cloud_msg.row_step);

      {
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(cloud_msg, "intensity");
        for (const auto & pt : cloud_points) {
          *iter_x = static_cast<float>(pt.x);
          *iter_y = static_cast<float>(pt.y);
          *iter_z = static_cast<float>(pt.z);
          // occupied voxel: intensity=1.0 表示不可通行
          *iter_i = 1.0f;
          ++iter_x; ++iter_y; ++iter_z; ++iter_i;
        }
      }

      map_cloud_pub_->publish(cloud_msg);
    }

    publishCostCloud();

    const double total_s =
      std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();
    RCLCPP_INFO(get_logger(), "Map published in %.2f s", total_s);
  }

  std_msgs::msg::ColorRGBA heightColor(double t, float alpha) const
  {
    if (t < 0.33) {
      const float k = static_cast<float>(t / 0.33);
      return makeColor(0.10F, 0.45F + 0.35F * k, 0.95F - 0.25F * k, alpha);
    }
    if (t < 0.66) {
      const float k = static_cast<float>((t - 0.33) / 0.33);
      return makeColor(0.10F + 0.85F * k, 0.80F + 0.10F * k, 0.70F - 0.55F * k, alpha);
    }
    const float k = static_cast<float>((t - 0.66) / 0.34);
    return makeColor(0.95F, 0.90F - 0.45F * k, 0.15F + 0.05F * k, alpha);
  }

  // ===== Cost 云发布 =====
  void publishCostCloud()
  {
    if (!planner_) {
      RCLCPP_WARN(get_logger(), "publishCostCloud: planner_ is null");
      return;
    }
    if (!cost_map_cloud_pub_) {
      RCLCPP_WARN(get_logger(), "publishCostCloud: cost_map_cloud_pub_ is null");
      return;
    }

    std::vector<global_planner::PointPose> cost_pos;
    std::vector<double> cost_vals;
    planner_->getCostFieldCloud(cost_pos, cost_vals);
    RCLCPP_INFO(get_logger(), "publishCostCloud: got %zu cost points", cost_pos.size());
    if (cost_pos.empty()) return;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = frame_id_;
    cloud_msg.header.stamp = now();
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(cost_pos.size());
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = true;

    cloud_msg.fields.resize(4);
    cloud_msg.fields[0].name = "x";
    cloud_msg.fields[0].offset = 0;
    cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[0].count = 1;
    cloud_msg.fields[1].name = "y";
    cloud_msg.fields[1].offset = 4;
    cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[1].count = 1;
    cloud_msg.fields[2].name = "z";
    cloud_msg.fields[2].offset = 8;
    cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[2].count = 1;
    cloud_msg.fields[3].name = "intensity";
    cloud_msg.fields[3].offset = 12;
    cloud_msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[3].count = 1;

    cloud_msg.point_step = 16;
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
    cloud_msg.data.resize(cloud_msg.row_step);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud_msg, "intensity");
    for (size_t i = 0; i < cost_pos.size(); ++i) {
      *iter_x = static_cast<float>(cost_pos[i].x);
      *iter_y = static_cast<float>(cost_pos[i].y);
      *iter_z = static_cast<float>(cost_pos[i].z);
      *iter_i = static_cast<float>(cost_vals[i]);
      ++iter_x; ++iter_y; ++iter_z; ++iter_i;
    }

    cost_map_cloud_pub_->publish(cloud_msg);
    RCLCPP_INFO(get_logger(), "Published cost cloud with %zu points.", cost_pos.size());
  }

  // ===== 导航路径发布 =====
  void publishNavPath(const std::vector<global_planner::PointPose> & path)
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now();
    msg.poses.reserve(path.size());

    // 计算路径朝向（MPPI 需要朝向信息）
    for (size_t i = 0; i < path.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position = makePoint(path[i].x, path[i].y, path[i].z);

      // 计算朝向
      if (path_orientation_mode_ == "interpolate") {
        // 模式 1: 根据路径方向插值计算所有点的朝向
        if (i < path.size() - 1) {
          double dx = path[i + 1].x - path[i].x;
          double dy = path[i + 1].y - path[i].y;
          double yaw = atan2(dy, dx);
          tf2::Quaternion q;
          q.setRPY(0, 0, yaw);
          pose.pose.orientation = tf2::toMsg(q);
        } else if (i > 0) {
          pose.pose.orientation = msg.poses[i - 1].pose.orientation;
        } else {
          pose.pose.orientation.w = 1.0;
        }
      } else if (path_orientation_mode_ == "from_goal" && i == path.size() - 1) {
        // 模式 2: 最后一个点使用 goal_pose 的朝向
        pose.pose.orientation = nav_goal_orientation_;
        // 其他点根据路径方向插值
        if (i > 0) {
          for (size_t j = 0; j < path.size() - 1; ++j) {
            if (j < path.size() - 1) {
              double dx = path[j + 1].x - path[j].x;
              double dy = path[j + 1].y - path[j].y;
              double yaw = atan2(dy, dx);
              tf2::Quaternion q;
              q.setRPY(0, 0, yaw);
              msg.poses[j].pose.orientation = tf2::toMsg(q);
            }
          }
        }
      } else {
        pose.pose.orientation.w = 1.0;
      }

      msg.poses.push_back(pose);
    }

    path_pub_->publish(msg);  // 发布到planned_path（Nav2执行）
    publishNavPathMarker(path);  // 导航路径可视化
  }

  // ===== 测试路径发布 =====
  void publishTestPath(const std::vector<global_planner::PointPose> & path)
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now();
    msg.poses.reserve(path.size());

    // 测试路径朝向：简单插值（不需要goal_pose朝向）
    for (size_t i = 0; i < path.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position = makePoint(path[i].x, path[i].y, path[i].z);

      if (i < path.size() - 1) {
        double dx = path[i + 1].x - path[i].x;
        double dy = path[i + 1].y - path[i].y;
        double yaw = atan2(dy, dx);
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);
      } else if (i > 0) {
        pose.pose.orientation = msg.poses[i - 1].pose.orientation;
      } else {
        pose.pose.orientation.w = 1.0;
      }

      msg.poses.push_back(pose);
    }

    test_path_pub_->publish(msg);  // 发布到test_path（仅供可视化）
    publishTestPathMarker(path);  // 测试路径可视化（不同颜色）
  }

  // ===== 导航路径Marker可视化 =====
  void publishNavPathMarker(const std::vector<global_planner::PointPose> & path)
  {
    if (!path_marker_pub_) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "planned_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.15;  // 绿色导航路径线条
    marker.color = makeColor(0.1F, 0.8F, 0.2F, 1.0F);  // 绿色（导航路径）

    marker.points.reserve(path.size());
    for (const auto & p : path) {
      marker.points.push_back(makePoint(p.x, p.y, p.z));
    }

    path_marker_pub_->publish(marker);
  }

  // ===== 测试路径Marker可视化 =====
  void publishTestPathMarker(const std::vector<global_planner::PointPose> & path)
  {
    if (!test_path_marker_pub_) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "test_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.12;  // 蓝色测试路径线条
    marker.color = makeColor(0.3F, 0.5F, 0.9F, 1.0F);  // 蓝色（测试路径）

    marker.points.reserve(path.size());
    for (const auto & p : path) {
      marker.points.push_back(makePoint(p.x, p.y, p.z));
    }

    test_path_marker_pub_->publish(marker);
  }

  void publishPoseMarker(
    const global_planner::PointPose & pose,
    const std::string & ns,
    int id,
    const std_msgs::msg::ColorRGBA & color,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher)
  {
    if (!publisher) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = makePoint(pose.x, pose.y, pose.z);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.35;
    marker.scale.y = 0.35;
    marker.scale.z = 0.35;
    marker.color = color;
    publisher->publish(marker);
  }

  std::string frame_id_;
  double start_z_ = 0.3;
  double goal_z_ = 0.3;
  double map_alpha_ = 0.82;
  std::string map_color_mode_ = "height";

  // ===== 双模式状态管理 =====
  // 导航模式状态（用于Nav2集成，initialpose + goal_pose）
  global_planner::PointPose nav_start_;
  global_planner::PointPose nav_goal_;
  bool has_nav_start_ = false;
  bool has_nav_goal_ = false;
  geometry_msgs::msg::Quaternion nav_goal_orientation_;  // 导航目标朝向

  // 测试模式状态（用于RViz Publish Point调试）
  global_planner::PointPose test_start_;
  global_planner::PointPose test_goal_;
  bool has_test_start_ = false;
  bool has_test_goal_ = false;
  bool next_clicked_point_is_start_ = true;  // 测试模式状态切换

  // Nav2集成相关参数（自动获取机器人位置）
  std::string robot_base_frame_ = "base_footprint";
  double transform_timeout_ = 0.5;
  std::string path_orientation_mode_ = "interpolate";
  std::string planned_path_topic_ = "planned_path";

  // OctoMap和规划器
  std::shared_ptr<pcd2octomap::Pcd2OctomapConverter> converter_;
  std::shared_ptr<global_planner::GlobalPlanner> planner_;
  std::shared_ptr<octomap::OcTree> octree_;
  std::vector<CachedVoxel> cached_voxels_;

  // TF2 用于获取机器人位置
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 导航路径发布器（用于Nav2执行）
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nav_start_marker_pub_;

  // Nav2 ComputePathToPose action server
  rclcpp_action::Server<nav2_msgs::action::ComputePathToPose>::SharedPtr action_server_;

  // Nav2 ComputePathThroughPoses action server (满足 bt_navigator 启动依赖)
  rclcpp_action::Server<nav2_msgs::action::ComputePathThroughPoses>::SharedPtr action_through_poses_server_;


  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr test_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr test_path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr test_start_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr test_goal_marker_pub_;

  // 地图可视化发布器
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cost_map_cloud_pub_;

  // 订阅器
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;

  

  rclcpp::TimerBase::SharedPtr map_timer_;

  // 参数运行时修改回调句柄
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handler_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctoPlannerRvizNode>());
  rclcpp::shutdown();
  return 0;
}
