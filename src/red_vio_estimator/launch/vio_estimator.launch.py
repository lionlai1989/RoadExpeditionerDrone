"""
It bridges the ground truth odometry from Gazebo sim and publishes the
corresponding TF (odom -> base_link) so SLAM/RViz can resolve transforms.

In the future, it will perform visual inertial odometry. but for now, it bridges ground truth
odometry from gazebo sim.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument("drone_id", description="Drone namespace")
    vio_source_arg = DeclareLaunchArgument("vio_source", description="groundtruth or openvins")

    drone_id = LaunchConfiguration("drone_id")

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

    # TODO: what is the frequency of the odometry topic?
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
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            drone_id_arg,
            vio_source_arg,
            ros_gz_bridge,
            odom_adapter,
        ]
    )
