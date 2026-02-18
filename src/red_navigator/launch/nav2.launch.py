from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument("drone_id", description="Drone namespace")

    drone_id = LaunchConfiguration("drone_id")

    params_file = "./src/red_navigator/config/nav2_params.yaml"

    param_rewrites = {
        "robot_base_frame": PythonExpression(["'", drone_id, "' + '/base_link'"]),
        "odom_frame": PythonExpression(["'", drone_id, "' + '/odom'"]),
        "global_frame": PythonExpression(["'", drone_id, "' + '/map'"]),
        "map_topic": PythonExpression(["'/' + '", drone_id, "' + '/map'"]),
    }
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=drone_id,
        param_rewrites=param_rewrites,
        convert_types=True,
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        namespace=drone_id,
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        namespace=drone_id,
        output="screen",
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        [
            drone_id_arg,
            planner_server,
            lifecycle_manager,
        ]
    )
