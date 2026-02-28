from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument("drone_id", description="Drone namespace")
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        description="Gazebo world name `<world name='...'>` defined in the world SDF.",
    )

    drone_id = LaunchConfiguration("drone_id")
    world_name = LaunchConfiguration("world_name")

    bridge_depth_image = PythonExpression(
        [
            "'/world/' + '",
            world_name,
            "' + '/model/' + '",
            drone_id,
            "' + '/link/camera_link/sensor/StereoOV7251/depth_image"
            "@sensor_msgs/msg/Image[gz.msgs.Image'",
        ]
    )
    bridge_depth_camera_info = PythonExpression(
        [
            "'/world/' + '",
            world_name,
            "' + '/model/' + '",
            drone_id,
            "' + '/link/camera_link/sensor/StereoOV7251/camera_info"
            "@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'",
        ]
    )

    remap_depth_image = PythonExpression(
        [
            "'/world/' + '",
            world_name,
            "' + '/model/' + '",
            drone_id,
            "' + '/link/camera_link/sensor/StereoOV7251/depth_image:=depth/image'",
        ]
    )
    remap_depth_camera_info = PythonExpression(
        [
            "'/world/' + '",
            world_name,
            "' + '/model/' + '",
            drone_id,
            "' + '/link/camera_link/sensor/StereoOV7251/camera_info:=depth/camera_info'",
        ]
    )
    depth_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="depth_bridge",
        namespace=drone_id,
        parameters=[{"use_sim_time": True}],
        arguments=[
            bridge_depth_image,
            bridge_depth_camera_info,
            "--ros-args",
            "-r",
            remap_depth_image,
            "-r",
            remap_depth_camera_info,
        ],
        output="screen",
    )

    # Gazebo publishes IMU data with frame_id "<drone_id>/base_link/imu_sensor".
    # RTAB-Map's body frame is "<drone_id>/base_link", so it needs TF between
    # these two frames to transform accel/gyro into the base frame.
    # In this model, IMU is colocated with base_link, so the transform is identity.
    static_tf_base_link_to_imu_sensor = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_link_to_imu_sensor",
        namespace=drone_id,
        output="screen",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            PythonExpression(["'", drone_id, "' + '/base_link'"]),
            "--child-frame-id",
            PythonExpression(["'", drone_id, "' + '/base_link/imu_sensor'"]),
        ],
        parameters=[{"use_sim_time": True}],
    )

    # RTAB-Map SLAM
    # https://wiki.ros.org/rtabmap_slam
    # https://github.com/introlab/rtabmap/blob/humble-devel/corelib/include/rtabmap/core/Parameters.h
    # Input: RGB, Depth, Odom
    # Output: Map
    rtabmap_db_path = PythonExpression(["'/tmp/rtabmap_' + '", drone_id, "' + '.db'"])
    rtabmap_params_file = PathJoinSubstitution(
        [FindPackageShare("red_perception"), "config", "rtabmap_params.yaml"]
    )
    rtabmap_launch = Node(
        # On ROS 2 Humble, `rtabmap_ros` is often a metapackage with no libexec.
        # The `rtabmap` node executable is provided by `rtabmap_slam`.
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace=drone_id,
        parameters=[
            rtabmap_params_file,
            {
                "frame_id": PythonExpression(["'", drone_id, "' + '/base_link'"]),
                "odom_frame_id": PythonExpression(["'", drone_id, "' + '/odom'"]),
                "map_frame_id": PythonExpression(["'", drone_id, "' + '/map'"]),
                "publish_tf": True,
                "subscribe_depth": True,
                "subscribe_imu": True,
                "wait_imu_to_init": True,
                "subscribe_scan": False,
                "approx_sync": True,
                "use_sim_time": True,
                "sync_queue_size": 3,  # Small to favor the latest RGB-D/odom data.
                "wait_for_transform": 0.5,
                "database_path": rtabmap_db_path,
            }
        ],
        remappings=[
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
            ("odom", "odometry"),
        ],
        arguments=["-d", "--ros-args", "--log-level", "warn"],  # debug, info, warn, error, fatal
        output="screen",
    )

    return LaunchDescription(
        [
            drone_id_arg,
            world_name_arg,
            depth_bridge,
            static_tf_base_link_to_imu_sensor,
            rtabmap_launch,
        ]
    )
