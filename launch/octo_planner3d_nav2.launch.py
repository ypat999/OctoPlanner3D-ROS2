#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node

def generate_launch_description():
    """生成 OctoPlanner3D 的 ROS2 launch 描述（支持namespace和global_config）。"""

    # 尝试从global_config导入参数
    try:
        from global_config.global_config import (
            OCTOPLANNER_BASE_CODE_PATH,
            OCTOPLANNER_PCD_FILE_PATH,
            OCTOPLANNER_PARAMS_FILE,
            DEFAULT_NAMESPACE,
            DEFAULT_USE_SIM_TIME
        )
        default_params_file = OCTOPLANNER_PARAMS_FILE
        default_namespace = DEFAULT_NAMESPACE
        default_use_sim_time = DEFAULT_USE_SIM_TIME
    except ImportError:
        # 如果global_config不存在，使用默认值
        pkg_dir = get_package_share_directory("octo_planner3d")
        default_params_file = os.path.join(pkg_dir, "config", "params.yaml")
        default_namespace = ''
        default_use_sim_time = False

    # --- Launch 级别的可覆盖参数 ------------------------------------------
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="YAML 参数文件路径（默认从global_config读取）",
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=default_namespace,
        description="节点命名空间（默认从global_config读取）",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value=str(default_use_sim_time).lower(),
        description="是否使用仿真时间（默认从global_config读取）",
    )

    # --- 节点配置（带namespace） -------------------------------------------
    node_group = GroupAction([
        PushRosNamespace(LaunchConfiguration("namespace")),
        Node(
            package="octo_planner3d",
            executable="octo_planner_rviz_node",
            name="octo_planner_rviz_node",
            output="screen",
            parameters=[
                LaunchConfiguration("params_file"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
        ),
    ])

    return LaunchDescription([
        params_file_arg,
        namespace_arg,
        use_sim_time_arg,
        node_group
    ])