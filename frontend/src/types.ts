// Shapes of the telemetry produced by dashboard_bridge (all values are real ROS data or null).
export type Vec3 = [number, number, number]

export interface MissionStats {
  replans: number; obstacle_detections: number; replan_latencies_ms: (number | null)[]; plan_latencies_ms: number[]
  target_losses: number; reacquire_times_s: number[]; aborts: number; corridor_breaches: number; sensor_failures: number
  max_spatial_error: number; mean_spatial_error: number | null; path_lengths_m: number[]
  state_history: { t: number; from: string; to: string; reason: string }[]
  mission_start_t: number | null; mission_end_t: number | null; outcome: string | null
}
export interface Mission {
  stamp: number; state: string; failure: boolean; outcome: string | null; spatial_error: number; e_max: number; e_warn: number
  reference: Vec3 | null; pose: Vec3 | null; yaw: number | null; velocity: Vec3 | null; goal: Vec3 | null
  trajectory_progress: number | null; sensors: Record<string, boolean>
  target: { pos: Vec3; stamp: number } | null
  landing_safety: { safe: boolean; surface_ok: boolean; reasons: string[]; tilt_deg: number | null; plane_rms: number | null
    height_above_ground: number | null; alignment_error: number | null; obstruction_points: number } | null
  pid: { x: { p: number; i: number; d: number; u: number; saturated: boolean }; y: { p: number; i: number; d: number; u: number; saturated: boolean } }
  stats: MissionStats
}
export interface Health { sensor: string; ok: boolean; stamp: number; rate_hz?: number; expected_hz?: number; age_s?: number | null; [k: string]: unknown }
export interface Snapshot {
  mission?: Mission; px4?: Record<string, any>; planner?: Record<string, any>; fusion?: Record<string, any>
  mapping?: Record<string, any>; avoidance?: Record<string, any>; detection?: Record<string, any>
  landing_safety?: Record<string, any>; metrics?: Record<string, any>; sensors?: Record<string, Health>
  events?: { source: string; t?: number; stamp?: number; level?: string; state?: string; text?: string; event?: string; note?: string; object?: string }[]
  paths?: { planned?: Vec3[]; local?: Vec3[]; actual?: Vec3[] }
  truth_position?: Vec3 | null; ros_time?: number; gps?: string
}
export interface Clouds { fused?: { xyz: number[]; rgb: number[]; in_fov: number[] }; map?: { xyz: number[]; rgb: number[]; total: number } }
export interface Obstacle { name: string; class: string; type: 'box' | 'cylinder' | 'sphere'; pos: Vec3; size?: Vec3; radius?: number; length?: number; color: Vec3 }
export interface Scene {
  world: { name: string; obstacles: Obstacle[]; dynamic: Obstacle[]; landing_pad: { center: [number, number]; size: [number, number]; tag_plate: number }; start: Vec3; tracking_station: Vec3 }
  mission: { mission_space: { x: [number, number]; y: [number, number]; z: [number, number] }; corridor: { centerline: Vec3[]; max_deviation: number; safety_margin: number }; landing_zone: { center: [number, number] } }
}
