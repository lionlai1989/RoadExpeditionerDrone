from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("drone_id", description="Drone namespace"),
            DeclareLaunchArgument("hover_height", description="Target hover height"),
            Node(
                package="red_navigator",
                executable="navigator_node",
                name="navigator_node",
                namespace=LaunchConfiguration("drone_id"),
                output="screen",
                parameters=[
                    {
                        "hover_height": ParameterValue(
                            LaunchConfiguration("hover_height"), value_type=float
                        ),
                        "use_sim_time": True,
                    }
                ],
            ),
        ]
    )
