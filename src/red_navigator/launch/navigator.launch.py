from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_red_navigator = get_package_share_directory("red_navigator")
    drone_id = LaunchConfiguration("drone_id")

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(pkg_red_navigator) / "launch" / "nav2.launch.py")),
        launch_arguments={
            "drone_id": drone_id,
        }.items(),
    )

    navigator_node = Node(
        package="red_navigator",
        executable="navigator_node",
        name="navigator_node",
        namespace=drone_id,
        output="screen",
        parameters=[
            {
                "hover_height": ParameterValue(
                    LaunchConfiguration("hover_height"), value_type=float
                ),
                "use_sim_time": True,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("drone_id", description="Drone namespace"),
            DeclareLaunchArgument("hover_height", description="Target hover height"),
            nav2_launch,
            navigator_node,
        ]
    )
