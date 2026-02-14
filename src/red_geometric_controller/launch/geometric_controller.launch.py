from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument("drone_id", description="Drone namespace")

    drone_id = LaunchConfiguration("drone_id")

    geometric_controller_node = Node(
        package="red_geometric_controller",
        executable="geometric_controller_node",
        name="geometric_controller_node",
        namespace=drone_id,
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription(
        [
            drone_id_arg,
            geometric_controller_node,
        ]
    )
