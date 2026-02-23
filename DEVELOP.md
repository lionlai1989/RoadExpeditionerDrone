# Container-based Development

- Visualize nodes, topics, services
```
rqt_graph
```

- List all ROS topics:
```
~/RoadExpeditionerDrone$ ros2 topic list
/clicked_point
/clock
/diagnostics
/drone_1/apriltag/detections
/drone_1/aruco/detections
/drone_1/aruco_opencv/detections
/drone_1/bond
/drone_1/cloud_ground
/drone_1/cloud_map
/drone_1/cloud_obstacles
/drone_1/command/motor_speed
/drone_1/depth/camera_info
/drone_1/depth/image
/drone_1/desired_odometry
/drone_1/global_costmap/costmap
/drone_1/global_costmap/costmap_raw
/drone_1/global_costmap/costmap_updates
/drone_1/global_costmap/footprint
/drone_1/global_costmap/global_costmap/transition_event
/drone_1/global_costmap/published_footprint
/drone_1/global_path
/drone_1/global_path_nodes
/drone_1/global_pose
/drone_1/goal
/drone_1/goal_node
/drone_1/goal_out
/drone_1/goal_reached
/drone_1/gps/fix
/drone_1/grid_prob_map
/drone_1/groundtruth_odometry
/drone_1/imu
/drone_1/info
/drone_1/initialpose
/drone_1/labels
/drone_1/landmark_detection
/drone_1/landmark_detections
/drone_1/landmarks
/drone_1/local_grid_empty
/drone_1/local_grid_ground
/drone_1/local_grid_obstacle
/drone_1/local_path
/drone_1/local_path_nodes
/drone_1/localization_pose
/drone_1/loop_depth
/drone_1/loop_depth/compressed
/drone_1/loop_depth/compressedDepth
/drone_1/loop_depth/theora
/drone_1/loop_depth_colored
/drone_1/loop_depth_colored/compressed
/drone_1/loop_depth_colored/compressedDepth
/drone_1/loop_depth_colored/theora
/drone_1/loop_extrinsic
/drone_1/loop_feats
/drone_1/loop_intrinsics
/drone_1/loop_pose
/drone_1/map
/drone_1/mapData
/drone_1/mapGraph
/drone_1/mapOdomCache
/drone_1/mapPath
/drone_1/map_updates
/drone_1/octomap_binary
/drone_1/octomap_empty_space
/drone_1/octomap_full
/drone_1/octomap_global_frontier_space
/drone_1/octomap_grid
/drone_1/octomap_ground
/drone_1/octomap_obstacles
/drone_1/octomap_occupied_space
/drone_1/odometry
/drone_1/odomimu
/drone_1/pathgt
/drone_1/pathimu
/drone_1/plan
/drone_1/planner_server/transition_event
/drone_1/points_aruco
/drone_1/points_msckf
/drone_1/points_sim
/drone_1/points_slam
/drone_1/posegt
/drone_1/poseimu
/drone_1/rgb/camera_info
/drone_1/rgb/image
/drone_1/rtabmap/republish_node_data
/drone_1/tag_detections
/drone_1/trackhist
/drone_1/trackhist/compressed
/drone_1/trackhist/compressedDepth
/drone_1/trackhist/theora
/drone_1/unsmoothed_plan
/drone_1/user_data_async
/initialpose
/move_base_simple/goal
/parameter_events
/rosout
/tf
/tf_static
```

- List all ROS nodes:
```
~/RoadExpeditionerDrone$ ros2 node list
/drone_1/flight_interface
/drone_1/geometric_controller_node
/drone_1/global_costmap/global_costmap
/drone_1/imu_bridge
/drone_1/lifecycle_manager_navigation
/drone_1/navigator_node
/drone_1/odom_adapter
/drone_1/odometry_groundtruth_bridge
/drone_1/openvins
/drone_1/planner_server
/drone_1/rgb_bridge
/drone_1/rtabmap
/drone_1/rviz2
/drone_1/sensor_bridge
/drone_1/static_tf_base_link_to_StereoOV7251
/drone_1/static_tf_base_link_to_camera_link
/drone_1/static_tf_base_link_to_imu_sensor
/drone_1/static_tf_base_link_to_imx214
/drone_1/transform_listener_impl_57c2a061ec40
/drone_1/transform_listener_impl_596512edca60
/drone_1/transform_listener_impl_60869eb3d6c0
/drone_1/transform_listener_impl_62a14f2c1210
/ros_gz_bridge_clock
```

- List all ROS transforms:
```
~/RoadExpeditionerDrone$ ros2 run tf2_tools view_frames
[INFO] [1771816989.069583032] [view_frames]: Listening to tf data for 5.0 seconds...
[INFO] [1771816994.071858850] [view_frames]: Generating graph in frames.pdf file...
[INFO] [1771816994.074868930] [view_frames]: Result:tf2_msgs.srv.FrameGraph_Response(frame_yaml="
    drone_1/odom: \n  parent: 'drone_1/map'\n  broadcaster: 'default_authority'\n  rate: 32.865\n  most_recent_transform: 77.305000\n  oldest_transform: 74.384000\n  buffer_length: 2.921\n
    drone_1/camera_link/StereoOV7251: \n  parent: 'drone_1/base_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\n
    drone_1/base_link: \n  parent: 'drone_1/odom'\n  broadcaster: 'default_authority'\n  rate: 250.342\n  most_recent_transform: 77.216000\n  oldest_transform: 74.288000\n  buffer_length: 2.928\n
    drone_1/camera_link/IMX214: \n  parent: 'drone_1/base_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\n
    drone_1/base_link/imu_sensor: \n  parent: 'drone_1/base_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\n
    drone_1/camera_link: \n  parent: 'drone_1/base_link'\n  broadcaster: 'default_authority'\n  rate: 10000.000\n  most_recent_transform: 0.000000\n  oldest_transform: 0.000000\n  buffer_length: 0.000\n
")

```

- Get frequency of a ROS topic:
```
~/RoadExpeditionerDrone$ ros2 topic hz /drone_1/odometry
average rate: 217.910
        min: 0.001s max: 0.020s std dev: 0.00220s window: 219
average rate: 226.386
        min: 0.001s max: 0.020s std dev: 0.00173s window: 454
average rate: 229.374
        min: 0.001s max: 0.025s std dev: 0.00166s window: 690
```

- List all Gazebo topics:
```
~/RoadExpeditionerDrone$ gz topic -l
/clock
/drone_1/command/motor_speed
/gazebo/resource_paths
/gui/camera/pose
/gui/currently_tracked
/gui/track
/model/drone_1/odometry
/model/drone_1/odometry_with_covariance
/model/drone_1/pose
/sensors/marker
/stats
/world/single_layer_maze_world/clock
/world/single_layer_maze_world/dynamic_pose/info
/world/single_layer_maze_world/model/drone_1/link/base_link/sensor/imu_sensor/imu
/world/single_layer_maze_world/model/drone_1/link/base_link/sensor/navsat_sensor/navsat
/world/single_layer_maze_world/model/drone_1/link/camera_link/sensor/IMX214/camera_info
/world/single_layer_maze_world/model/drone_1/link/camera_link/sensor/IMX214/image
/world/single_layer_maze_world/model/drone_1/link/camera_link/sensor/StereoOV7251/camera_info
/world/single_layer_maze_world/model/drone_1/link/camera_link/sensor/StereoOV7251/depth_image
/world/single_layer_maze_world/model/drone_1/link/camera_link/sensor/StereoOV7251/depth_image/points
/world/single_layer_maze_world/pose/info
/world/single_layer_maze_world/scene/deletion
/world/single_layer_maze_world/scene/info
/world/single_layer_maze_world/state
/world/single_layer_maze_world/stats
/world/single_layer_maze_world/light_config
/world/single_layer_maze_world/material_color
```
