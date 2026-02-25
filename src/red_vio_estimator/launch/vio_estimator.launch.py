"""
Launch VIO for a namespaced drone.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument("drone_id", description="Drone namespace")
    vio_source_arg = DeclareLaunchArgument(
        "vio_source", default_value="groundtruth", description="groundtruth or openvins"
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name", description="Gazebo world name `<world name='...'>` defined in the world SDF."
    )

    drone_id = LaunchConfiguration("drone_id")
    vio_source = LaunchConfiguration("vio_source")
    world_name = LaunchConfiguration("world_name")

    use_openvins = IfCondition(PythonExpression(["'", vio_source, "' == 'openvins'"]))

    bridge_groundtruth_odometry = PythonExpression(
        [
            "'/model/' + '",
            drone_id,
            "' + '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'",
        ]
    )
    remap_groundtruth_odometry = PythonExpression(
        [
            "'/model/' + '",
            drone_id,
            "' + '/odometry:=groundtruth_odometry'",
        ]
    )
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="odometry_groundtruth_bridge",
        namespace=drone_id,
        parameters=[{"use_sim_time": True}],
        arguments=[
            bridge_groundtruth_odometry,
            "--ros-args",
            "-r",
            remap_groundtruth_odometry,
        ],
        output="screen",
    )

    bridge_imu = PythonExpression(
        [
            "'/world/' + '",
            world_name,
            "' + '/model/' + '",
            drone_id,
            "' + '/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'",
        ]
    )
    bridge_rgb_image = PythonExpression(
        [
            "'/world/' + '",
            world_name,
            "' + '/model/' + '",
            drone_id,
            "' + '/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image'",
        ]
    )
    bridge_rgb_camera_info = PythonExpression(
        [
            "'/world/' + '",
            world_name,
            "' + '/model/' + '",
            drone_id,
            "' + '/link/camera_link/sensor/IMX214/camera_info"
            "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'",
        ]
    )
    remap_imu = PythonExpression(
        [
            "'/world/' + '",
            world_name,
            "' + '/model/' + '",
            drone_id,
            "' + '/link/base_link/sensor/imu_sensor/imu:=imu'",
        ]
    )
    remap_rgb_image = PythonExpression(
        [
            "'/world/' + '",
            world_name,
            "' + '/model/' + '",
            drone_id,
            "' + '/link/camera_link/sensor/IMX214/image:=rgb/image'",
        ]
    )
    remap_rgb_camera_info = PythonExpression(
        [
            "'/world/' + '",
            world_name,
            "' + '/model/' + '",
            drone_id,
            "' + '/link/camera_link/sensor/IMX214/camera_info:=rgb/camera_info'",
        ]
    )
    # Keep IMU and RGB in separate bridge processes: IMU is high-rate and
    # timing-critical for VIO, while images are lower-rate but CPU-heavy.
    # Separation avoids image-load contention (callbacks/serialization), reducing
    # IMU latency/jitter and improving estimator stability under load.
    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="imu_bridge",
        namespace=drone_id,
        parameters=[{"use_sim_time": True}],
        arguments=[
            bridge_imu,
            "--ros-args",
            "-r",
            remap_imu,
        ],
        output="screen",
    )
    rgb_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="rgb_bridge",
        namespace=drone_id,
        parameters=[{"use_sim_time": True}],
        arguments=[
            bridge_rgb_image,
            bridge_rgb_camera_info,
            "--ros-args",
            "-r",
            remap_rgb_image,
            "-r",
            remap_rgb_camera_info,
        ],
        output="screen",
    )

    # TF frame IDs are not auto-namespaced by node namespace.
    odom_frame_id = PythonExpression(["'", drone_id, "' + '/odom'"])
    base_frame_id = PythonExpression(["'", drone_id, "' + '/base_link'"])

    odom_adapter = Node(
        package="red_vio_estimator",
        executable="odom_adapter",
        name="odom_adapter",
        namespace=drone_id,
        parameters=[
            {
                "use_sim_time": True,
                "odom_frame_id": odom_frame_id,
                "base_frame_id": base_frame_id,
                "vio_source": vio_source,
            }
        ],
        output="screen",
    )

    openvins_config_path = PathJoinSubstitution(
        [
            FindPackageShare("red_vio_estimator"),
            "config",
            "openvins",
            "estimator_config.yaml",
        ]
    )
    openvins_node = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        name="openvins",
        namespace=drone_id,
        condition=use_openvins,
        parameters=[
            {
                "use_sim_time": True,
                "verbosity": "WARNING",
                "config_path": openvins_config_path,
                "use_stereo": False,
                "max_cameras": 1,
                "publish_global_to_imu_tf": False,
                "publish_calibration_tf": False,
            }
        ],
        remappings=[
            ("/imu0", "imu"),
            ("/cam0/image_raw", "rgb/image"),  # OpenVINS converts rgb to mono8 internally.
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            drone_id_arg,
            vio_source_arg,
            world_name_arg,
            ros_gz_bridge,
            imu_bridge,
            rgb_bridge,
            odom_adapter,
            openvins_node,
        ]
    )
