#include "global_planner.h"
#include "pcd2octomap_converter.h"

#include <algorithm>
#include <chrono>
#include <cmath>
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

namespace
{

constexpr double kZeroZThreshold = 1.0e-6;

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
    const double map_publish_period =
      declare_parameter<double>("map_publish_period", 2.0);

    converter_ = std::make_shared<pcd2octomap::Pcd2OctomapConverter>();
    converter_->setInputPcdFile(input_pcd);
    if (!output_bt.empty()) {
      converter_->setOutputBtFile(output_bt);
    }
    planner_ = std::make_shared<global_planner::GlobalPlanner>();

    RCLCPP_INFO(get_logger(), "Building OctoMap from configured PCD file...");
    if (!converter_->convert()) {
      RCLCPP_ERROR(get_logger(), "Failed to build OctoMap. Node will stay alive for diagnostics.");
      return;
    }

    octree_ = converter_->getOctomap();
    planner_->setOctomap(octree_);

    const auto transient_qos = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("occupied_map", transient_qos);
    map_cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("occupied_map_cloud", transient_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", transient_qos);
    path_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("planned_path_marker", transient_qos);
    start_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("start_marker", transient_qos);
    goal_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("goal_marker", transient_qos);

    start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose",
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onStartPose, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose",
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onGoalPose, this, std::placeholders::_1));
    clicked_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_point_topic,
      rclcpp::QoS(10),
      std::bind(&OctoPlannerRvizNode::onClickedPoint, this, std::placeholders::_1));

    publishMap();
    map_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.1, map_publish_period))),
      std::bind(&OctoPlannerRvizNode::publishMap, this));

    RCLCPP_INFO(
      get_logger(),
      "Ready. Use RViz2 Publish Point on %s: first click sets start, second click sets goal.",
      clicked_point_topic.c_str());
  }

private:
  void onStartPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    start_ = toPlannerPoint(msg->pose.pose, start_z_);
    has_start_ = true;
    publishPoseMarker(start_, "start", 0, makeColor(0.1F, 0.9F, 0.2F, 1.0F), start_marker_pub_);
    RCLCPP_INFO(
      get_logger(),
      "Start set to [%.3f, %.3f, %.3f]",
      start_.x,
      start_.y,
      start_.z);
    planIfReady();
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_ = toPlannerPoint(msg->pose, goal_z_);
    has_goal_ = true;
    publishPoseMarker(goal_, "goal", 0, makeColor(0.95F, 0.25F, 0.15F, 1.0F), goal_marker_pub_);
    RCLCPP_INFO(
      get_logger(),
      "Goal set to [%.3f, %.3f, %.3f]",
      goal_.x,
      goal_.y,
      goal_.z);
    planIfReady();
  }

  void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (next_clicked_point_is_start_) {
      start_ = toPlannerPoint(msg->point);
      has_start_ = true;
      has_goal_ = false;
      next_clicked_point_is_start_ = false;
      publishPoseMarker(start_, "start", 0, makeColor(0.1F, 0.9F, 0.2F, 1.0F), start_marker_pub_);
      RCLCPP_INFO(
        get_logger(),
        "Start point set to [%.3f, %.3f, %.3f]. Publish the next point as goal.",
        start_.x,
        start_.y,
        start_.z);
      return;
    }

    goal_ = toPlannerPoint(msg->point);
    has_goal_ = true;
    next_clicked_point_is_start_ = true;
    publishPoseMarker(goal_, "goal", 0, makeColor(0.95F, 0.25F, 0.15F, 1.0F), goal_marker_pub_);
    RCLCPP_INFO(
      get_logger(),
      "Goal point set to [%.3f, %.3f, %.3f]. Planning with clicked start and goal.",
      goal_.x,
      goal_.y,
      goal_.z);
    planIfReady();
  }

  void planIfReady()
  {
    if (!planner_ || !octree_ || !has_start_ || !has_goal_) {
      return;
    }

    planner_->makePlan(start_, goal_);

    std::vector<global_planner::PointPose> path;
    planner_->getPlannerResults(path);
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "Planner returned an empty path.");
      publishPath(path);
      return;
    }

    publishPath(path);
    RCLCPP_INFO(get_logger(), "Published planned path with %zu poses.", path.size());
  }

  void publishMap()
  {
    if (!octree_ || !map_pub_) {
      return;
    }

    double min_x = 0.0;
    double min_y = 0.0;
    double min_z = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    double max_z = 0.0;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
    const double z_range = std::max(1.0e-6, max_z - min_z);
    const float alpha = static_cast<float>(std::clamp(map_alpha_, 0.05, 1.0));

    // --- MarkerArray（按大小分组，按高度着色） ---
    std::unordered_map<double, visualization_msgs::msg::Marker> markers_by_size;
    // --- PointCloud2 计数 ---
    size_t point_count = 0;

    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it)) {
        continue;
      }
      ++point_count;

      const double size = it.getSize();
      auto marker_it = markers_by_size.find(size);
      if (marker_it == markers_by_size.end()) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.ns = "occupied_voxels";
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        marker.color = makeColor(0.45F, 0.22F, 0.06F, alpha);
        marker_it = markers_by_size.emplace(size, std::move(marker)).first;
      }

      marker_it->second.points.push_back(makePoint(it.getX(), it.getY(), it.getZ()));
    }

    // MarkerArray publish
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

    // --- PointCloud2 publish ---
    if (map_cloud_pub_ && point_count > 0) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      cloud_msg.header.frame_id = frame_id_;
      cloud_msg.header.stamp = now();
      cloud_msg.height = 1;
      cloud_msg.width = static_cast<uint32_t>(point_count);
      cloud_msg.is_bigendian = false;
      cloud_msg.is_dense = true;

      // fields: x, y, z (FLOAT32 each)
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

      // fill data
      {
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
          if (!octree_->isNodeOccupied(*it)) {
            continue;
          }
          *iter_x = static_cast<float>(it.getX());
          *iter_y = static_cast<float>(it.getY());
          *iter_z = static_cast<float>(it.getZ());
          ++iter_x; ++iter_y; ++iter_z;
        }
      }

      map_cloud_pub_->publish(cloud_msg);
    }
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

  void publishPath(const std::vector<global_planner::PointPose> & path)
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now();
    msg.poses.reserve(path.size());

    for (const auto & point : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position = makePoint(point.x, point.y, point.z);
      pose.pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }

    path_pub_->publish(msg);
    publishPathMarker(path);
  }

  void publishPathMarker(const std::vector<global_planner::PointPose> & path)
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
    marker.action = path.empty() ?
      visualization_msgs::msg::Marker::DELETE :
      visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    // 线宽：图里那种比较粗的紫色路径
    marker.scale.x = 0.18;   // 原来是 0.1，可再调成 0.15~0.25

    // 深紫色，接近你图里的颜色
    marker.color = makeColor(0.32F, 0.16F, 0.62F, 1.0F);

    marker.points.reserve(path.size());

    for (const auto & point : path) {
      marker.points.push_back(makePoint(point.x, point.y, point.z));
    }

    path_marker_pub_->publish(marker);
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
  bool has_start_ = false;
  bool has_goal_ = false;
  bool next_clicked_point_is_start_ = true;

  global_planner::PointPose start_;
  global_planner::PointPose goal_;
  std::shared_ptr<pcd2octomap::Pcd2OctomapConverter> converter_;
  std::shared_ptr<global_planner::GlobalPlanner> planner_;
  std::shared_ptr<octomap::OcTree> octree_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr start_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_marker_pub_;
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
