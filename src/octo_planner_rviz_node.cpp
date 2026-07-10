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

    // 导航路径发布器（用于Nav2执行）
    path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", transient_qos);
    path_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("planned_path_marker", transient_qos);
    nav_start_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("nav_start_marker", transient_qos);
    nav_goal_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("nav_goal_marker", transient_qos);

    // 测试路径发布器（仅供可视化，不影响导航）
    test_path_pub_ = create_publisher<nav_msgs::msg::Path>("test_path", transient_qos);
    test_path_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("test_path_marker", transient_qos);
    test_start_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("test_start_marker", transient_qos);
    test_goal_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("test_goal_marker", transient_qos);

    // 订阅器：导航模式（initialpose + goal_pose）
    start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose",
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onNavStartPose, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose",
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onNavGoalPose, this, std::placeholders::_1));

    // 订阅器：测试模式（RViz Publish Point）
    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_point_topic,
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onTestClickedPoint, this, std::placeholders::_1));

    // 初始化 TF2（用于自动获取机器人位置）
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    RCLCPP_INFO(get_logger(), "TF2 initialized. Auto-start from robot position when goal_pose received without initialpose.");

    publishMap();
    if (map_publish_period > 0.0) {
      map_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(map_publish_period)),
        std::bind(&OctoPlannerRvizNode::publishMap, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "Ready. Use RViz2 Publish Point on %s: first click sets start, second click sets goal.",
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
      "[Nav Mode] Start set to [%.3f, %.3f, %.3f]",
      nav_start_.x,
      nav_start_.y,
      nav_start_.z);
    planNavPath();
  }

  void onNavGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // 保存导航目标朝向（用于路径朝向计算）
    nav_goal_orientation_ = msg->pose.orientation;

    nav_goal_ = toPlannerPoint(msg->pose, goal_z_);
    has_nav_goal_ = true;
    publishPoseMarker(nav_goal_, "nav_goal", 0, makeColor(0.95F, 0.25F, 0.15F, 1.0F), nav_goal_marker_pub_);
    RCLCPP_INFO(
      get_logger(),
      "[Nav Mode] Goal set to [%.3f, %.3f, %.3f]",
      nav_goal_.x,
      nav_goal_.y,
      nav_goal_.z);

    // 每次 goal_pose 从 TF 重新获取机器人当前位置作为起点
    // 这样即使机器人移动了，新路径也从当前位置开始规划
    try {
      geometry_msgs::msg::TransformStamped transform;
      transform = tf_buffer_->lookupTransform(
        frame_id_, robot_base_frame_,
        tf2::TimePointZero);

      nav_start_.x = transform.transform.translation.x;
      nav_start_.y = transform.transform.translation.y;
      nav_start_.z = transform.transform.translation.z;
      if (std::abs(nav_start_.z) < kZeroZThreshold) {
        nav_start_.z = start_z_;
      }

      has_nav_start_ = true;
      publishPoseMarker(nav_start_, "nav_start", 0, makeColor(0.1F, 0.9F, 0.2F, 1.0F), nav_start_marker_pub_);
      RCLCPP_INFO(
        get_logger(),
        "[Nav Mode] Auto-start from robot TF: [%.3f, %.3f, %.3f]",
        nav_start_.x,
        nav_start_.y,
        nav_start_.z);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "[Nav Mode] Failed to get robot position from TF: %s", ex.what());
      has_nav_start_ = false;
    }

    planNavPath();
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

    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it)) {
        continue;
      }
      CachedVoxel voxel;
      voxel.x = static_cast<float>(it.getX());
      voxel.y = static_cast<float>(it.getY());
      voxel.z = static_cast<float>(it.getZ());
      voxel.size = static_cast<float>(it.getSize());
      cached_voxels_.push_back(voxel);

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

    std::vector<global_planner::PointPose> path;
    planner_->getPlannerResults(path);
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "[Nav Mode] Planner returned an empty path (%.2f s).", plan_elapsed);
      publishNavPath(path);
      return;
    }

    publishNavPath(path);
    RCLCPP_INFO(get_logger(), "[Nav Mode] Published nav path with %zu poses (%.2f s).", path.size(), plan_elapsed);
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
    RCLCPP_INFO(get_logger(), "[Test Mode] Published test path with %zu poses (%.2f s).", path.size(), plan_elapsed);
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

      cloud_msg.fields.resize(3);
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

      cloud_msg.point_step = 12;
      cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
      cloud_msg.data.resize(cloud_msg.row_step);

      {
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        for (const auto & pt : cloud_points) {
          *iter_x = static_cast<float>(pt.x);
          *iter_y = static_cast<float>(pt.y);
          *iter_z = static_cast<float>(pt.z);
          ++iter_x; ++iter_y; ++iter_z;
        }
      }

      map_cloud_pub_->publish(cloud_msg);
    }

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
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr nav_goal_marker_pub_;

  // 测试路径发布器（仅供可视化，不影响导航）
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr test_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr test_path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr test_start_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr test_goal_marker_pub_;

  // 地图可视化发布器
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;

  // 订阅器
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;

  rclcpp::TimerBase::SharedPtr map_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctoPlannerRvizNode>());
  rclcpp::shutdown();
  return 0;
}
