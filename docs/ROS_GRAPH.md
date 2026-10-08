# ROS 2 Graph and Topic Design

All nodes run with `use_sim_time:=true` (Gazebo `/clock`). Frames follow REP-103 (map = ENU, body = FLU, optical = z-forward).

## Topics

| Topic | Type | Publisher | Subscribers | Class |
|---|---|---|---|---|
| `/clock` | rosgraph_msgs/Clock | gz_bridge | all | infra |
| `/camera/image_raw`, `/camera/camera_info` | Image, CameraInfo | gz_bridge | camera_processor | sensor |
| `/down_camera/image_raw`, `/down_camera/camera_info` | Image, CameraInfo | gz_bridge | camera_processor | sensor |
| `/lidar/points` | PointCloud2 (lidar_frame) | gz_bridge | lidar_processor | sensor |
| `/down_lidar/points` | PointCloud2 (down_lidar_frame) | gz_bridge | lidar_processor | sensor |
| `/perception/camera/image`, `/perception/camera/camera_info` | Image, CameraInfo | camera_processor | fusion_node, dashboard_bridge | autonomy |
| `/perception/down_camera/image`, `.../camera_info` | Image, CameraInfo | camera_processor | detector_node | autonomy |
| `/perception/lidar/points` | PointCloud2 | lidar_processor | fusion_node, mapping (fallback), avoidance | autonomy |
| `/perception/down_lidar/points` | PointCloud2 | lidar_processor | validator_node | autonomy |
| `/fused/points` | PointCloud2 x,y,z,rgb,in_fov | fusion_node | mapping_node, dashboard_bridge | autonomy |
| `/fusion/status` | String (JSON) | fusion_node | evaluation, dashboard | status |
| `/tracking/measurement` | ps05_interfaces/TrackingMeasurement | ground_tracker | localization_node | autonomy (permitted localisation) |
| `/fmu/in/vehicle_visual_odometry` | px4_msgs/VehicleOdometry | localization_node | PX4 EKF2 | autonomy |
| `/fmu/out/vehicle_odometry` | px4_msgs/VehicleOdometry | PX4 | localization_node | autonomy |
| `/vehicle/odom` | nav_msgs/Odometry | localization_node | avoidance, validator, mission_manager, evaluation | autonomy |
| `/tf`, `/tf_static` | TF | localization_node, static_frames | all | autonomy |
| `/occupancy_map` | PointCloud2 (occupied voxels, RGB) | mapping_node | RViz, dashboard | autonomy |
| `/occupancy_map/grid` | ps05_interfaces/VoxelGrid | mapping_node | planner_node, avoidance_node | autonomy |
| `/planner/plan_path` | ps05_interfaces/PlanPath (service) | planner_node | mission_manager | autonomy |
| `/planned_path` | nav_msgs/Path | planner_node | RViz, dashboard | autonomy |
| `/mission/corridor` | MarkerArray | planner_node | RViz | autonomy |
| `/planner/status`, `/planner/inflated` | String, PointCloud2 | planner_node | evaluation, dashboard, RViz | status |
| `/local_trajectory` | nav_msgs/Path | mission_manager | avoidance_node, scenario_events, RViz | autonomy |
| `/avoidance/status` | String (JSON) | avoidance_node | mission_manager | autonomy |
| `/landing/target_pose` | PoseStamped (map) | detector_node | validator, mission_manager | autonomy |
| `/landing/detection` | String (JSON) | detector_node | evaluation, dashboard | status |
| `/landing/safety_status` | String (JSON) | validator_node | mission_manager | autonomy |
| `/sensors/health` | String (JSON) | processors, localization | mission_manager (watchdogs) | autonomy |
| `/px4_interface/setpoint`, `/px4_interface/command` | String (JSON / text) | mission_manager | px4_interface | autonomy |
| `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, `/fmu/in/vehicle_command` | px4_msgs | px4_interface | PX4 | autonomy |
| `/fmu/out/vehicle_status(_v1)`, `/fmu/out/vehicle_land_detected` | px4_msgs | PX4 | px4_interface | autonomy |
| `/px4_interface/state` | String (JSON) | px4_interface | mission_manager | autonomy |
| `/mission/status`, `/mission/events` | String (JSON) | mission_manager | evaluation, dashboard, processors (fault-clock triggers) | status |
| `/sim/truth/odom` | nav_msgs/Odometry | gz_bridge | **evaluation_node, ground_tracker (sensor model), scenario_events, dashboard_bridge only** | **EVALUATION** |
| `/scenario/events` | String (JSON) | scenario_events | evaluation, dashboard | test harness |
| `/evaluation/metrics` | String (JSON) | evaluation_node | dashboard | evaluation |

## TF tree

```
world ─ map ─ odom ─ base_link ─┬─ camera_link ─ camera_optical_frame
                                ├─ lidar_link ─ lidar_frame
                                ├─ down_camera_link ─ down_camera_optical_frame
                                └─ down_lidar_link ─ down_lidar_frame
```
`odom → base_link` comes from EKF2 (via localization_node). Everything else is static from `configs/sensors.yaml`.
