# OctoPlanner3D-ROS2 - Nav2集成说明

## **集成方式**

OctoPlanner3D-ROS2已集成到`LIO-SAM_MID360_ROS2_PKG`工作空间，支持：

1. **符号链接方式**：通过符号链接连接到主仓库，便于同步更新
2. **global_config适配**：自动根据主机名加载对应配置
3. **namespace支持**：支持多机器人命名空间

## **配置系统**

### **1. global_config配置**

OctoPlanner配置已添加到`global_config/global_config/__init__.py`：

```python
# 不同主机配置示例
'ywj-B250-D3A': {
    # OctoPlanner配置
    'OCTOPLANNER_BASE_CODE_PATH': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/OctoPlanner3D-ROS2/',
    'OCTOPLANNER_PCD_FILE_PATH': '/home/ywj/slam_data/pcd/octomap.pcd',
    'OCTOPLANNER_PARAMS_FILE': '/home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/OctoPlanner3D-ROS2/config/params.yaml',
}
```

**配置项说明**：
- `OCTOPLANNER_BASE_CODE_PATH`：OctoPlanner代码路径
- `OCTOPLANNER_PCD_FILE_PATH`：PCD地图文件路径（不同主机可配置不同地图）
- `OCTOPLANNER_PARAMS_FILE`：参数文件路径

### **2. Launch文件使用**

#### **标准启动方式**（自动从global_config读取）

```bash
# 启动OctoPlanner（自动适配当前主机）
ros2 launch octo_planner3d octo_planner3d_nav2.launch.py
```

#### **自定义参数启动**

```bash
# 覆盖默认参数
ros2 launch octo_planner3d octo_planner3d_nav2.launch.py \
    params_file:=/path/to/custom_params.yaml \
    namespace:=robot1 \
    use_sim_time:=true
```

### **3. Namespace支持**

支持多机器人场景：

```bash
# 机器人1
ros2 launch octo_planner3d octo_planner3d_nav2.launch.py namespace:=robot1

# 机器人2
ros2 launch octo_planner3d octo_planner3d_nav2.launch.py namespace:=robot2
```

**话题映射**：
- 无namespace：`/planned_path`、`/goal_pose`
- namespace=`robot1`：`/robot1/planned_path`、`/robot1/goal_pose`

## **与Nav2集成**

### **1. 修改后的nav2_launch_3d.py**

已在`lio_nav2_unified_3d.launch.py`中集成：

```python
# OctoPlanner启动（自动namespace）
octoplanner_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(octoplanner_dir, "launch", "octo_planner3d_nav2.launch.py")
    ),
    launch_arguments={
        "params_file": octoplanner_params,
        "namespace": namespace,
        "use_sim_time": use_sim_time,
    }.items(),
)
```

### **2. 启动完整3D导航系统**

```bash
# 启动LIO + Nav2 + OctoPlanner（自动适配主机）
ros2 launch nav2_dog_slam lio_nav2_unified_3d.launch.py
```

## **不同主机配置指南**

### **添加新主机配置**

编辑`global_config/global_config/__init__.py`：

```python
config_by_machine = {
    'YOUR_HOSTNAME': {
        # ...其他配置
        
        # OctoPlanner配置
        'OCTOPLANNER_BASE_CODE_PATH': '/path/to/OctoPlanner3D-ROS2/',
        'OCTOPLANNER_PCD_FILE_PATH': '/path/to/your_octomap.pcd',
        'OCTOPLANNER_PARAMS_FILE': '/path/to/OctoPlanner3D-ROS2/config/params.yaml',
    },
}
```

### **获取当前主机名**

```bash
hostname  # 或 platform.node() in Python
```

## **参数文件说明**

### **params.yaml关键参数**

```yaml
/**:
  ros__parameters:
    # PCD文件路径（实际由global_config决定）
    input_pcd: "/home/ywj/slam_data/pcd/octomap.pcd"
    
    # 规划参数
    resolution: 0.25
    robot_radius: 0.5
    
    # nav2集成参数
    robot_base_frame: "base_footprint"
    map_frame: "map"
    path_orientation_mode: "interpolate"
```

**注意**：params.yaml中的路径是默认值，实际运行时由global_config覆盖。

## **编译和测试**

### **编译**

```bash
cd /home/ywj/git/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2
colcon build --packages-select octo_planner3d nav2_dog_slam global_config
source install/setup.bash
```

### **测试**

```bash
# 1. 测试OctoPlanner单独运行
ros2 launch octo_planner3d octo_planner3d_nav2.launch.py

# 2. 发送测试目标
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: 5.0, z: 0.3}}}"

# 3. 观察路径
ros2 topic echo /planned_path

# 4. 测试完整系统
ros2 launch nav2_dog_slam lio_nav2_unified_3d.launch.py
```

## **故障排查**

### **常见问题**

1. **global_config导入失败**
   ```bash
   # 确保global_config已编译
   colcon build --packages-select global_config
   source install/setup.bash
   ```

2. **PCD文件路径错误**
   ```bash
   # 检查global_config配置
   python3 -c "from global_config.global_config import OCTOPLANNER_PCD_FILE_PATH; print(OCTOPLANNER_PCD_FILE_PATH)"
   ```

3. **namespace话题不匹配**
   ```bash
   # 检查话题名称
   ros2 topic list | grep octoplanner
   ```

## **架构总结**

```
global_config（根据主机名自动选择）
  ↓ 导出配置参数
octo_planner3d_nav2.launch.py（支持namespace）
  ↓ 加载params.yaml
OctoPlanner节点（三维规划）
  ↓ 发布planned_path
octoplanner_nav2_adapter（Action适配）
  ↓ ComputePathToPose Action
Nav2 bt_navigator（行为树）
  ↓ 执行路径
```

## **维护说明**

### **更新OctoPlanner代码**

由于使用符号链接，更新主仓库即可：

```bash
cd /home/ywj/git/3d_dog_navi_ros2/src/OctoPlanner3D-ROS2
git pull origin main
```

### **修改配置**

修改`global_config/global_config/__init__.py`，重启launch即可生效。

## **参考文档**

- [OctoPlanner3D-ROS2主仓库](链接)
- [global_config使用说明](链接)
- [Nav2集成架构](链接)