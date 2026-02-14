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

    drone_id = LaunchConfiguration("drone_id")

    bridge_odometry = PythonExpression(
        [
            "'/model/' + '",
            drone_id,
            "' + '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'",
        ]
    )

    remap_odometry = PythonExpression(
        [
            "'/model/' + '",
            drone_id,
            "' + '/odometry:=odometry'",
        ]
    )

    # TODO: what is the frequency of the odometry topic?
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="odometry_ground_truth_bridge",
        namespace=drone_id,
        parameters=[{"use_sim_time": True}],
        arguments=[
            bridge_odometry,
            "--ros-args",
            "-r",
            remap_odometry,
        ],
        output="screen",
    )

    # TF frame IDs are not auto-namespaced by node namespace.
    odom_frame_id = PythonExpression(["'", drone_id, "' + '/odom'"])
    base_frame_id = PythonExpression(["'", drone_id, "' + '/base_link'"])

    # Broadcast the odom -> base_link transform from the bridged Odometry topic.
    odom_tf_broadcaster = Node(
        package="red_vio_estimator",
        executable="odom_tf_broadcaster",
        name="odom_tf_broadcaster",
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
            ros_gz_bridge,
            odom_tf_broadcaster,
        ]
    )
