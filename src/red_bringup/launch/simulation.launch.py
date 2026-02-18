import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

DRONES_CONFIG = {
    "drone_1": {  # namespace for the drone
        "model_name": "x500_depth",
        "initial_pose": "0.0,0.0,0.0",  # spawn position in meters in world frame (x, y, z)
        "hover_height": "1.0",  # meters
    },
}

# First per-drone launch starts after core services are up.
# Large worlds can take longer to load before /world/.../create is available.
DRONE_LAUNCH_START_TIME = 8.0
# Global cursor to serialize per-drone launches.
DRONE_LAUNCH_CURSOR = DRONE_LAUNCH_START_TIME
# Define individual launch durations for each action
SPAWN_LAUNCH_DURATION = 5.0
VIO_ESTIMATOR_LAUNCH_DURATION = 5.0
PERCEPTION_LAUNCH_DURATION = 3.0
NAVIGATOR_LAUNCH_DURATION = 3.0
GEOMETRIC_CONTROLLER_LAUNCH_DURATION = 2.0
RVIZ_LAUNCH_DURATION = 2.0


def parse_initial_pose(pose_str):
    parts = [p.strip() for p in pose_str.split(",")]
    if len(parts) != 3:
        raise ValueError(f"initial_pose must be 'x,y,z', got '{pose_str}'")
    return [float(part) for part in parts]


def build_spawn_request(model_sdf_path, model_name, pose_str):
    x, y, z = parse_initial_pose(pose_str)
    return (
        f'sdf_filename: "{model_sdf_path}" '
        f'name: "{model_name}" '
        "allow_renaming: false "
        f"pose: {{ position: {{ x: {x}, y: {y}, z: {z} }} }}"
    )


def create_drone_launch(
    drone_id,
    drone_config,
    models_dir,
    pkg_red_bringup,
    pkg_red_geometric_controller,
    pkg_red_navigator,
    pkg_red_perception,
    pkg_red_vio_estimator,
):
    global DRONE_LAUNCH_CURSOR
    static_tf_launch = create_static_tf(drone_id)

    model_sdf = Path(models_dir) / drone_config["model_name"] / "model.sdf"
    if not model_sdf.exists():
        raise FileNotFoundError(f"Model SDF not found: {model_sdf}")
    spawn_request = build_spawn_request(model_sdf, drone_id, drone_config["initial_pose"])
    spawn_drone = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            PythonExpression(['"/world/" + "', LaunchConfiguration("world_name"), '" + "/create"']),
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "15000",
            "--req",
            spawn_request,
        ],
        output="screen",
    )

    vio_estimator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg_red_vio_estimator) / "launch" / "vio_estimator.launch.py")
        ),
        launch_arguments={
            "drone_id": drone_id,
        }.items(),
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg_red_perception) / "launch" / "perception.launch.py")
        ),
        launch_arguments={
            "drone_id": drone_id,
            "world_name": LaunchConfiguration("world_name"),
        }.items(),
    )

    navigator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg_red_navigator) / "launch" / "navigator.launch.py")
        ),
        launch_arguments={
            "drone_id": drone_id,
            "hover_height": drone_config["hover_height"],
        }.items(),
    )

    geometric_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg_red_geometric_controller) / "launch" / "geometric_controller.launch.py")
        ),
        launch_arguments={
            "drone_id": drone_id,
        }.items(),
    )

    # In "default.rviz", I hardcode `Fixed Frame: drone_1/map`. Ignore it for now.
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=drone_id,
        arguments=[
            "-d",
            str(Path(pkg_red_bringup) / "rviz" / "default.rviz"),
            "--ros-args",
            "--log-level",
            "warn",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    launch_actions = []

    # Start per-drone launches after Gazebo is up.
    # Use a global cursor so each drone starts after the previous one.
    current_time = DRONE_LAUNCH_CURSOR

    launch_actions.append(TimerAction(period=current_time, actions=[spawn_drone]))
    current_time += SPAWN_LAUNCH_DURATION

    launch_actions.append(TimerAction(period=current_time, actions=static_tf_launch))

    launch_actions.append(TimerAction(period=current_time, actions=[vio_estimator_launch]))
    current_time += VIO_ESTIMATOR_LAUNCH_DURATION

    launch_actions.append(TimerAction(period=current_time, actions=[perception_launch]))
    current_time += PERCEPTION_LAUNCH_DURATION

    launch_actions.append(TimerAction(period=current_time, actions=[navigator_launch]))
    current_time += NAVIGATOR_LAUNCH_DURATION

    launch_actions.append(TimerAction(period=current_time, actions=[geometric_controller_launch]))
    current_time += GEOMETRIC_CONTROLLER_LAUNCH_DURATION

    launch_actions.append(TimerAction(period=current_time, actions=[rviz2_node]))
    current_time += RVIZ_LAUNCH_DURATION

    # Advance global cursor so next drone launches later.
    DRONE_LAUNCH_CURSOR = current_time

    return launch_actions


def create_cleanup_actions():
    # terminate all gz sim instances on the machine
    return [
        ExecuteProcess(
            cmd=["bash", "-lc", "pkill -TERM -f '(^|.*/)?gz sim' || true"],
            output="screen",
        ),
    ]


def generate_launch_description():
    # Get the directories of the ROS packages
    pkg_red_bringup = get_package_share_directory("red_bringup")
    pkg_red_geometric_controller = get_package_share_directory("red_geometric_controller")
    pkg_red_navigator = get_package_share_directory("red_navigator")
    pkg_red_gz_plugin = get_package_share_directory("red_gz_plugin")
    pkg_red_perception = get_package_share_directory("red_perception")
    pkg_red_vio_estimator = get_package_share_directory("red_vio_estimator")

    pkg_red_gz_plugin_prefix = get_package_prefix("red_gz_plugin")

    project_models_dir = Path(pkg_red_gz_plugin) / "models"
    project_worlds_dir = Path(pkg_red_bringup) / "worlds"
    home_gazebo_models_dir = Path.home() / ".gazebo" / "models"

    # Named arguments that can be passed to the launch file from the command line
    world_path_arg = DeclareLaunchArgument(
        "world_path",
        description="Path to the Gazebo world file to launch",
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        description="Gazebo world name `<world name='...'>` defined in the world SDF.",
    )

    # Gazebo environment variables.
    gz_resource_paths = [
        str(project_models_dir),
        ":",
        str(project_worlds_dir),
        ":",
        str(home_gazebo_models_dir),
    ]
    gz_system_plugin_path = str(Path(pkg_red_gz_plugin_prefix) / "lib")
    existing_system_plugin_path = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
    if existing_system_plugin_path:
        gz_system_plugin_path = existing_system_plugin_path + os.pathsep + gz_system_plugin_path

    gz_additional_env = {
        "GZ_SIM_RESOURCE_PATH": gz_resource_paths,
        "GZ_SIM_SYSTEM_PLUGIN_PATH": gz_system_plugin_path,
    }

    # Always run Gazebo with GUI enabled.
    gz_sim = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "--verbose=1",
            "-r",
            LaunchConfiguration("world_path"),
        ],
        output="screen",
        additional_env=gz_additional_env,
    )

    ros_gz_bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_clock",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    drone_launches = []
    for drone_id, drone_config in DRONES_CONFIG.items():
        drone_launches.extend(
            create_drone_launch(
                drone_id,
                drone_config,
                project_models_dir,
                pkg_red_bringup,
                pkg_red_geometric_controller,
                pkg_red_navigator,
                pkg_red_perception,
                pkg_red_vio_estimator,
            )
        )

    return LaunchDescription(
        [
            # Launch arguments
            world_path_arg,
            world_name_arg,
            # Pre-cleanup
            *create_cleanup_actions(),
            # (t=3.0) Start Gazebo. Assume pre-cleanup has been done.
            TimerAction(
                period=3.0,
                actions=[gz_sim],
            ),
            # (t=6.0) Start ros_gz_bridge_clock.
            TimerAction(
                period=6.0,
                actions=[ros_gz_bridge_clock],
            ),
            *drone_launches,
            RegisterEventHandler(OnShutdown(on_shutdown=create_cleanup_actions())),
        ]
    )


def create_static_tf(drone_id):
    # Read src/red_gz_plugin/models/x500_depth/model.sdf
    # base_link -> camera_link (.12 .03 .484)
    static_tf_base_link_to_camera_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_link_to_camera_link",
        namespace=drone_id,
        output="screen",
        arguments=[
            "--x",
            "0.12",
            "--y",
            "0.03",
            "--z",
            "0.484",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            f"{drone_id}/base_link",
            "--child-frame-id",
            f"{drone_id}/camera_link",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # 1. Translation Calculation (Base Link -> Sensor):
    #    - Base -> Camera Mount (OakD-Lite Body):
    #      From x500_depth/model.sdf: x=0.12, y=0.03, z=0.484
    #    - Camera Mount -> Sensor (IMX214/StereoOV7251):
    #      From OakD-Lite/model.sdf: x=0.01233, y=-0.03, z=0.01878
    #    - Total Translation:
    #      x = 0.12 + 0.01233 = 0.13233
    #      y = 0.03 + (-0.03) = 0.00
    #      z = 0.484 + 0.01878 = 0.50278
    # 2. Rotation Calculation (Gazebo Body Frame -> ROS Optical Frame):
    #    - Gazebo Camera (SDF): Uses standard robotics convention where X is Forward, Y is Left,
    #      Z is Up. The sensor in SDF has <pose>... 0 0 0</pose>, so it aligns with the drone body
    #      (Base Link).
    #    - ROS Camera (Optical): Expects Z is Forward (Depth), X is Right, Y is Down.
    #    - Transformation Required (Body -> Optical):
    #      Goal: Align Body X (Fwd) -> Optical Z (Fwd)
    #            Align Body -Y (Right) -> Optical X (Right)
    #            Align Body -Z (Down) -> Optical Y (Down)
    #    - Euler Angles (Static Transform Publisher uses Yaw, Pitch, Roll sequence):
    #      1. Yaw -90 deg (-1.5707 rad) around Z:
    #         Points New X to Right (Old -Y), New Y to Forward (Old X).
    #      2. Roll -90 deg (-1.5707 rad) around New X:
    #         Points New Y to Down (Old -Z), New Z to Forward (Old X).
    #      Result: X=Right, Y=Down, Z=Forward.
    return [
        static_tf_base_link_to_camera_link,
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_StereoOV7251",
            namespace=drone_id,
            output="screen",
            arguments=[
                "--x",
                "0.13233",
                "--y",
                "0.00",
                "--z",
                "0.50278",
                "--yaw",
                "-1.5707",
                "--pitch",
                "0",
                "--roll",
                "-1.5707",
                "--frame-id",
                f"{drone_id}/base_link",
                "--child-frame-id",
                f"{drone_id}/camera_link/StereoOV7251",
            ],
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_base_link_to_imx214",
            namespace=drone_id,
            output="screen",
            arguments=[
                "--x",
                "0.13233",
                "--y",
                "0.00",
                "--z",
                "0.50278",
                "--yaw",
                "-1.5707",
                "--pitch",
                "0",
                "--roll",
                "-1.5707",
                "--frame-id",
                f"{drone_id}/base_link",
                "--child-frame-id",
                f"{drone_id}/camera_link/IMX214",
            ],
            parameters=[{"use_sim_time": True}],
        ),
    ]
