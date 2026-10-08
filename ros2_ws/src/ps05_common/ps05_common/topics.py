"""Canonical topic names and the AUTONOMY vs EVALUATION data classification.

The GPS / ground-truth separation audit (tests/regression/test_gps_audit.py and
scripts/gps_audit.py) uses these lists:

* Autonomy nodes must never subscribe to any topic in FORBIDDEN_FOR_AUTONOMY.
* Only nodes listed in EVALUATION_NODES (and the simulated tracking-station
  sensor model) may subscribe to simulator ground truth.
"""

# --- Simulator ground truth: EVALUATION ONLY --------------------------------
TRUTH_ODOM = '/sim/truth/odom'
TRUTH_PREFIX = '/sim/truth'

# --- GPS-related PX4 topics: must not be consumed by autonomy -----------------
GPS_TOPICS = (
    '/fmu/out/vehicle_gps_position',
    '/fmu/out/sensor_gps',
    '/fmu/out/vehicle_global_position',
    '/fmu/out/estimator_gps_status',
    '/fmu/out/gps_inject_data',
)

FORBIDDEN_FOR_AUTONOMY = (TRUTH_ODOM,) + GPS_TOPICS
FORBIDDEN_PREFIXES_FOR_AUTONOMY = (TRUTH_PREFIX, '/world/', '/model/')

# Nodes that ARE allowed to read ground truth.
EVALUATION_NODES = ('evaluation_node', 'scenario_events', 'dashboard_bridge')
# The tracking-station *sensor model* reads truth to synthesise noisy (r, theta, phi).
SENSOR_MODEL_NODES = ('ground_tracker',)

AUTONOMY_NODES = (
    'camera_processor', 'lidar_processor', 'fusion_node', 'localization_node',
    'mapping_node', 'planner_node', 'avoidance_node', 'detector_node',
    'validator_node', 'mission_manager', 'px4_interface', 'static_frames',
)

# --- Autonomy data topics ----------------------------------------------------
TRACKING = '/tracking/measurement'
VEHICLE_ODOM = '/vehicle/odom'
FUSED_POINTS = '/fused/points'
OCCUPANCY_MAP = '/occupancy_map'
OCCUPANCY_GRID = '/occupancy_map/grid'
PLANNED_PATH = '/planned_path'
LOCAL_TRAJECTORY = '/local_trajectory'
ACTUAL_PATH = '/vehicle/actual_path'
CORRIDOR_MARKERS = '/mission/corridor'
LANDING_TARGET = '/landing/target_pose'
LANDING_SAFETY = '/landing/safety_status'
LANDING_DETECTION = '/landing/detection'
MISSION_STATUS = '/mission/status'
MISSION_EVENTS = '/mission/events'
PLANNER_STATUS = '/planner/status'
AVOIDANCE_STATUS = '/avoidance/status'
FUSION_STATUS = '/fusion/status'
MAP_STATUS = '/mapping/status'
SENSOR_HEALTH = '/sensors/health'
PX4_STATE = '/px4_interface/state'
PX4_SETPOINT = '/px4_interface/setpoint'
PX4_COMMAND = '/px4_interface/command'
EVAL_METRICS = '/evaluation/metrics'
SCENARIO_EVENTS = '/scenario/events'
