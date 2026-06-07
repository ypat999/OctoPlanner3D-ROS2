# OctoPlanner3D-ROS2

OctoPlanner3D-ROS2 是基于纯 C++ 版本 [OctoPlanner3D](https://github.com/JackJu-HIT/OctoPlanner3D) 封装的 ROS2 可视化测试版本，主要用于验证 3D 全局路径规划算法在不同场景下的规划效果、性能表现和能力边界。

## 项目说明

本项目在原有纯 C++ 版本 OctoPlanner3D 的基础上增加了 ROS2 外壳，用于更方便地进行地图加载、路径规划和 RViz 可视化测试。

需要说明的是：

- 核心规划算法仍然保持纯 C++ 实现；
- ROS2 部分主要负责节点封装、参数管理和可视化发布；
- 地图可视化没有使用 `octomap_msgs` 消息类型，而是通过 `visualization_msgs::msg::MarkerArray` 发布体素地图；
- 项目主要用于测试 OctoPlanner3D 算法在三维场景、跨楼层场景和复杂障碍环境下的规划能力。

## 依赖环境

- ROS2
- PCL
- OctoMap
- Eigen
- CMake / colcon

## 编译方法

```bash
colcon build
````

编译完成后，加载环境变量：

```bash
source install/setup.bash
```

## 运行方法

```bash
ros2 run octo_planner3d octo_planner_rviz_node
```

运行后可在 RViz2 中查看：

* OctoMap 体素地图
* 起点与终点
* 三维规划路径
* 路径搜索结果

## 项目特点

* 基于 PCD 点云构建 OctoMap 地图
* 支持三维空间路径搜索
* 支持跨楼层路径规划测试
* 支持 RViz2 可视化显示
* 核心算法与 ROS2 框架解耦
* 便于测试算法效果和能力边界

## 原始项目

纯 C++ 版本仓库：

[https://github.com/JackJu-HIT/OctoPlanner3D](https://github.com/JackJu-HIT/OctoPlanner3D)

## 更多内容

更多机器人路径规划、运动控制和自动驾驶相关内容，欢迎关注微信公众号：

**机器人规划与控制研究所**

````

