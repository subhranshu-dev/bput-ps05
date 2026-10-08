"""PX4 offboard interface (Phase 1).

The single bridge between the ROS autonomy stack and PX4 over uXRCE-DDS.
* input  /px4_interface/setpoint (JSON: mode position|velocity, pos, vel, yaw; map ENU)
* input  /px4_interface/command  (String: offboard | arm | disarm | land | hold)
* output /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint, /fmu/in/vehicle_command
* output /px4_interface/state (JSON: connected, armed, nav_state, offboard, landed, ...)

Safety: if the autonomy stops sending setpoints for > setpoint_timeout_s, this
node stops the offboard heartbeat, so PX4's own offboard-loss failsafe takes
over (PX4 remains the flight controller). ENU<->NED happens only via
ps05_common.frames.
"""
import json
import math

import numpy as np
import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLandDetected, VehicleStatus
from rclpy.node import Node
from std_msgs.msg import String

from ps05_common import topics
from ps05_common.frames import enu_to_ned, yaw_enu_to_ned
from ps05_common.ros_utils import PX4_QOS, RELIABLE_QOS, json_msg, now_sec

NAV_OFFBOARD = 14
ARMED = 2
NAN = float('nan')


class Px4Interface(Node):
    def __init__(self):
        super().__init__('px4_interface')
        self.declare_parameter('setpoint_timeout_s', 0.5)
        self.declare_parameter('rate_hz', 20.0)
        self.timeout = self.get_parameter('setpoint_timeout_s').value
        self.pub_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)
        self.pub_state = self.create_publisher(String, topics.PX4_STATE, RELIABLE_QOS)
        # PX4 v1.16+ may version some topics (e.g. vehicle_status_v1): subscribe to both spellings
        for name in ('/fmu/out/vehicle_status', '/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_status_v2'):
            self.create_subscription(VehicleStatus, name, self.on_status, PX4_QOS)
        self.create_subscription(VehicleLandDetected, '/fmu/out/vehicle_land_detected', self.on_land, PX4_QOS)
        self.create_subscription(String, topics.PX4_SETPOINT, self.on_setpoint, RELIABLE_QOS)
        self.create_subscription(String, topics.PX4_COMMAND, self.on_command, RELIABLE_QOS)
        self.status = None
        self.status_t = -1e9
        self.landed = True
        self.sp = None
        self.sp_t = -1e9
        self.cmd_log = []
        self.heartbeat_active = False
        self.create_timer(1.0 / self.get_parameter('rate_hz').value, self.tick)
        self.create_timer(0.1, self.publish_state)

    # ------------------------------------------------------------- callbacks
    def on_status(self, msg):
        self.status = msg
        self.status_t = now_sec(self)

    def on_land(self, msg):
        self.landed = bool(msg.landed)

    def on_setpoint(self, msg):
        try:
            sp = json.loads(msg.data)
            if sp.get('mode') not in ('position', 'velocity'):
                raise ValueError('mode must be position or velocity')
            for k in ('pos', 'vel'):
                if k in sp and sp[k] is not None and (len(sp[k]) != 3 or not all(map(math.isfinite, sp[k]))):
                    raise ValueError(f'invalid {k}')
            self.sp = sp
            self.sp_t = now_sec(self)
        except (ValueError, TypeError, json.JSONDecodeError) as e:
            self.get_logger().error(f'rejected setpoint: {e}')

    def on_command(self, msg):
        c = msg.data.strip().lower()
        t = now_sec(self)
        if c == 'arm':
            self.vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        elif c == 'disarm':
            self.vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        elif c == 'offboard':
            self.vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif c == 'land':
            self.vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        elif c == 'hold':
            self.vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
        else:
            self.get_logger().error(f'unknown command {c!r}')
            return
        self.cmd_log.append({'t': t, 'cmd': c})
        self.get_logger().info(f'command: {c}')

    # ------------------------------------------------------------- output
    def vehicle_command(self, command, p1=0.0, p2=0.0, p3=0.0):
        m = VehicleCommand()
        m.command = command
        m.param1, m.param2, m.param3 = float(p1), float(p2), float(p3)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(m)

    def tick(self):
        t = now_sec(self)
        fresh = self.sp is not None and (t - self.sp_t) <= self.timeout
        if not fresh:
            if self.heartbeat_active:
                self.get_logger().warn('setpoint stream stale -> stopping offboard heartbeat (PX4 failsafe)')
            self.heartbeat_active = False
            return
        self.heartbeat_active = True
        ts = int(self.get_clock().now().nanoseconds / 1000)
        mode = OffboardControlMode()
        mode.timestamp = ts
        vel_mode = self.sp['mode'] == 'velocity'
        mode.position = not vel_mode
        mode.velocity = vel_mode
        self.pub_mode.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = ts
        if vel_mode:
            sp.position = [NAN, NAN, NAN]
            sp.velocity = enu_to_ned(self.sp['vel']).astype(np.float32).tolist()
        else:
            sp.position = enu_to_ned(self.sp['pos']).astype(np.float32).tolist()
            v = self.sp.get('vel')
            sp.velocity = enu_to_ned(v).astype(np.float32).tolist() if v is not None else [NAN, NAN, NAN]
        sp.acceleration = [NAN, NAN, NAN]
        sp.yaw = float(yaw_enu_to_ned(self.sp.get('yaw', 0.0))) if self.sp.get('yaw') is not None else NAN
        sp.yawspeed = NAN
        self.pub_sp.publish(sp)

    def publish_state(self):
        t = now_sec(self)
        st = self.status
        connected = st is not None and (t - self.status_t) < 1.5
        self.pub_state.publish(json_msg({
            'stamp': t,
            'connected': connected,
            'armed': bool(st and st.arming_state == ARMED),
            'nav_state': int(st.nav_state) if st else None,
            'offboard': bool(st and st.nav_state == NAV_OFFBOARD),
            'landed': self.landed,
            'preflight_ok': bool(st and getattr(st, 'pre_flight_checks_pass', False)),
            'failsafe': bool(st and getattr(st, 'failsafe', False)),
            'heartbeat_active': self.heartbeat_active,
            'setpoint_mode': self.sp['mode'] if self.sp else None,
            'last_commands': self.cmd_log[-5:],
        }))


def main():
    rclpy.init()
    node = Px4Interface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
