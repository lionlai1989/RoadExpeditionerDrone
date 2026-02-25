# Container-based Development

## Coordinate Frames

Following standard ROS conventions (REP-105), the operational TF chain is
`map -> odom -> base_link -> sensor frames`.

- **Gazebo `world`**: The absolute global frame used by the simulator for model poses and ground
  truth. It is not part of the ROS TF chain unless explicitly published.

- **OpenVINS `global`**: An estimator-internal reference frame, independent of Gazebo `world`.
  During initialization, it is gravity-aligned (global +Z opposite gravity), while yaw and position
  gauge freedoms are set by initialization constraints.

- **`odom`**: A local, continuous frame used for control and short-horizon tracking. In this
  project, `odom_adapter` publishes `odom -> base_link`, initializes `odom` to identity at startup,
  bootstraps from ground truth, and then (in OpenVINS mode) hands off to OpenVINS with a frozen
  alignment transform to preserve continuity.

- **`map`**: A globally consistent SLAM frame produced by RTAB-Map. RTAB-Map publishes `map ->
  odom`; this transform may jump during loop closure to correct long-term drift.

- **`base_link`**: The drone body frame (REP-103: x forward, y left, z up). Sensor frames (IMU,
  cameras) are static children of `base_link` directly or via intermediate frames (e.g.,
  `camera_link`).

## Useful Commands

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
/drone_1/visual_path
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
/drone_1/depth_bridge
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
/drone_1/static_tf_base_link_to_StereoOV7251
/drone_1/static_tf_base_link_to_camera_link
/drone_1/static_tf_base_link_to_imu_sensor
/drone_1/static_tf_base_link_to_imx214
/drone_1/transform_listener_impl_55e163ea3220
/drone_1/transform_listener_impl_56a3497839a0
/drone_1/transform_listener_impl_58208091ee00
/drone_1/transform_listener_impl_650ded189030
/ros_gz_bridge_clock
```

- List all ROS transforms:
```
# save to frames_*.gv
~/RoadExpeditionerDrone$ ros2 run tf2_tools view_frames
```

- Get frequency of a ROS topic:
```
~/RoadExpeditionerDrone$ ros2 topic hz /drone_1/odometry
average rate: 162.661
        min: 0.002s max: 0.174s std dev: 0.01471s window: 164
average rate: 196.888
        min: 0.001s max: 0.174s std dev: 0.00957s window: 396
average rate: 199.207
        min: 0.001s max: 0.174s std dev: 0.00841s window: 600
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

- Check a topic info
```
~/RoadExpeditionerDrone$ ros2 topic info /drone_1/groundtruth_odometry
Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 1

~/RoadExpeditionerDrone$ ros2 topic info /drone_1/odomimu
Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 1
```

- Echo a topic
```
~/RoadExpeditionerDrone$ ros2 topic echo /drone_1/groundtruth_odometry --once
header:
  stamp:
    sec: 26
    nanosec: 474000000
  frame_id: drone_1/odom
child_frame_id: base_link
pose:
  pose:
    position:
      x: 0.1717624200014549
      y: 0.14759456979226276
      z: 1.004849135096039
    orientation:
      x: 0.0004421677389328969
      y: 6.664824350618467e-05
      z: 0.9354640663051916
      w: 0.3534215340036009
  covariance:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
twist:
  twist:
    linear:
      x: 0.005936595370283374
      y: -0.011820595552447837
      z: -0.0023790858835462704
    angular:
      x: 0.001200939294909607
      y: -0.00015051510292427134
      z: 0.29768048411510095
  covariance:
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
---
~/RoadExpeditionerDrone$ ros2 topic echo /drone_1/odomimu --once
header:
  stamp:
    sec: 48
    nanosec: 236000000
  frame_id: global
child_frame_id: imu
pose:
  pose:
    position:
      x: 524.0059590795693
      y: -541.8659050516916
      z: 2162.3691493474594
    orientation:
      x: 0.011868988924405377
      y: -0.12840298366496253
      z: -0.7841384711993249
      w: 0.6070409037890555
  covariance:
  - 125.86681079318883
  - 121.12481361786037
  - -0.10303018309036126
  - 0.03038978631846393
  - 0.05113505057067531
  - 0.2247365431072873
  - 121.12481361785991
  - 117.1587417458563
  - 0.022560989359674703
  - 0.029520597865216647
  - 0.04920014339809374
  - 0.21673994769454033
  - -0.10303018309035923
  - 0.022560989359670207
  - 0.028395545024802297
  - 2.9537003361377188e-05
  - -4.229477996344877e-05
  - -8.736810491777887e-05
  - 0.030389786318464077
  - 0.029520597865216595
  - 2.953700336137925e-05
  - 7.694322930646216e-06
  - 1.2258688431100008e-05
  - 5.448035152536218e-05
  - 0.051135050570675245
  - 0.04920014339809386
  - -4.229477996344797e-05
  - 1.225868843109973e-05
  - 2.1090461771056878e-05
  - 9.163272426984228e-05
  - 0.22473654310728747
  - 0.21673994769454036
  - -8.736810491776139e-05
  - 5.4480351525361866e-05
  - 9.163272426984202e-05
  - 0.00040211217278125007
twist:
  twist:
    linear:
      x: 86.37960064937997
      y: 134.4579131110566
      z: 215.30644600072247
    angular:
      x: 0.06602473296943537
      y: 0.05597980272247064
      z: -0.3010309613216407
  covariance:
  - 2.9755371683157197
  - -1.9263867887540203
  - 0.00794957761217231
  - 0.0
  - 0.0
  - 0.0
  - -1.9263867887540265
  - 1.253001513906738
  - -0.008114578778154679
  - 0.0
  - 0.0
  - 0.0
  - 0.007949577612173667
  - -0.00811457877815541
  - 0.001547766084588071
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 7.617600000004222e-07
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 7.617600000004222e-07
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 7.617600000004222e-07
---
```
