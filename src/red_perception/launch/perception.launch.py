from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sensor_bridge",
        namespace=drone_id,
        parameters=[{"use_sim_time": True}],
        arguments=[
            bridge_depth_image,
            bridge_depth_camera_info,
            bridge_rgb_image,
            bridge_rgb_camera_info,
            "--ros-args",
            "-r",
            remap_depth_image,
            "-r",
            remap_depth_camera_info,
            "-r",
            remap_rgb_image,
            "-r",
            remap_rgb_camera_info,
        ],
        output="screen",
    )

    # RTAB-Map SLAM
    # https://wiki.ros.org/rtabmap_slam
    # https://github.com/introlab/rtabmap/blob/humble-devel/corelib/include/rtabmap/core/Parameters.h
    # Input: RGB, Depth, Odom
    # Output: Map
    rtabmap_db_path = PythonExpression(["'/tmp/rtabmap_' + '", drone_id, "' + '.db'"])
    rtabmap_launch = Node(
        # On ROS 2 Humble, `rtabmap_ros` is often a metapackage with no libexec.
        # The `rtabmap` node executable is provided by `rtabmap_slam`.
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace=drone_id,
        parameters=[
            {
                "frame_id": PythonExpression(["'", drone_id, "' + '/base_link'"]),
                "odom_frame_id": PythonExpression(["'", drone_id, "' + '/odom'"]),
                "map_frame_id": PythonExpression(["'", drone_id, "' + '/map'"]),
                "publish_tf": True,
                "subscribe_depth": True,
                "subscribe_scan": False,
                "approx_sync": True,
                "use_sim_time": True,
                "sync_queue_size": 10,
                "wait_for_transform": 0.5,
                "database_path": rtabmap_db_path,
                # RTAB-Map parameters
                "Rtabmap/LoopThr": "0.2",
                "Rtabmap/LoopRatio": "0.2",
                "RGBD/AngularUpdate": "0.01",
                "RGBD/LinearUpdate": "0.01",
                "RGBD/OptimizeFromGraphEnd": "false",
                "RGBD/LoopCovLimited": "false",
                "RGBD/OptimizeMaxError": "2.0",
                "Vis/CorNNDR": "0.7",
                "Vis/MinInliers": "30",
                "Vis/MinInliersDistribution": "0.0",
                "Vis/MaxFeatures": "1000",
                "Kp/NndrRatio": "0.7",
                "Kp/BadSignRatio": "0.5",
                "Kp/MaxFeatures": "600",
                "Grid/MaxGroundHeight": "0.2",  # Points < 0.2m are Ground (Free)
                "Grid/MaxObstacleHeight": "2.0",  # Points > 2.0m (and < 3m walls) are Obstacles
                "Grid/RangeMax": "5.0",  # Limit grid map range
                "Grid/NoiseFilteringMinNeighbors": "5",  # Reduce noise
                "Grid/RayTracing": "true",  # Ray tracing to clear free space
                "Grid/FromDepth": "true",  # Create 2D grid from depth camera
                "Mem/DepthCompressionFormat": ".png",
                "Mem/SaveDepth16Format": "false",  # float32 depth
                "Mem/BadSignaturesIgnored": "false",
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
            ros_gz_bridge,
            rtabmap_launch,
        ]
    )
