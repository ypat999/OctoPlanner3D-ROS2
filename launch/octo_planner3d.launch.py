import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 octo_planner3d 的 ROS2 launch 描述。"""

    pkg_dir = get_package_share_directory("octo_planner3d")

    # --- Launch 级别的可覆盖参数 ------------------------------------------
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_dir, "config", "params.yaml"),
        description="YAML 参数文件路径",
    )

    # --- 节点 -------------------------------------------------------------
    node = Node(
        package="octo_planner3d",
        executable="octo_planner_rviz_node",
        name="octo_planner_rviz_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([params_file_arg, node])
