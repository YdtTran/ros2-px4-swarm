"""
PX4SwarmNode — điều khiển bầy đàn 3 UAV qua XRCE-DDS (px4_msgs).

Commands (ARM, OFFBOARD, velocity) → VehicleCommand / TrajectorySetpoint → PX4
State + position                   → VehicleStatus / VehicleLocalPosition ← PX4

Frame convention:
  PX4 / XRCE-DDS : NED  (x=North, y=East,  z=Down)
  Algorithms      : ENU  (x=East,  y=North, z=Up)
  Conversion ENU→NED: vx_ned=vy_enu, vy_ned=vx_enu, vz_ned=-vz_enu
"""

import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

from uav_swarm_demo.algorithms.formation import LeaderFollowerFormation, UAVState
from uav_swarm_demo.algorithms.collision_avoidance import CollisionAvoidance


NUM_UAVS      = 3
TARGET_ALT    = 5.0    # m (ENU Up)
TAKEOFF_SPEED = 1.5    # m/s
ALT_TOLERANCE = 0.3    # m
MAX_SPEED     = 2.0    # m/s
GRID_RES      = 1.0    # m/cell
CTRL_RATE     = 20.0   # Hz

# ── Mission A → B ─────────────────────────────────────────────────────────────
# UAVs spawn at A = (0, 0) ENU and fly in V-formation to B = (10, 0) ENU.
# UAV0 = leader (0,0), UAV1 = follower (2,0), UAV2 = follower (4,0).
# All three fly +10 m East and settle into V-shape behind the leader.
POINT_A_ENU = (0.0, 0.0)    # spawn centroid (East, North) m
POINT_B_ENU = (10.0, 0.0)   # goal            (East, North) m  — 10 m East

# QoS cho px4_msgs (BEST_EFFORT + VOLATILE)
_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class FlightState(Enum):
    PREFLIGHT = auto()
    ARMING    = auto()
    TAKEOFF   = auto()
    FLYING    = auto()
    ARRIVED   = auto()


class UAVData:
    def __init__(self, uav_id: int, init_x_enu: float, init_y_enu: float):
        self.id               = uav_id
        self.flight_state     = FlightState.PREFLIGHT
        self.armed            = False
        self.nav_state        = 0
        self.x                = init_x_enu   # ENU East  (m)
        self.y                = init_y_enu   # ENU North (m)
        self.z                = 0.0          # ENU Up    (m)
        self.offboard_count      = 0
        self.arm_retry_count     = 0
        self.preflight_pass      = False
        self.preflight_stable    = 0   # consecutive ticks with preflight_pass=True
        self.preflight_wait      = 0   # ticks waiting for stable preflight


class PX4SwarmNode(Node):

    def __init__(self):
        super().__init__('px4_swarm_node')

        # PX4_GZ_MODEL_POSE: "i*2,0,0" → Gazebo x=0,2,4 (East) → ENU x=0,2,4
        spawn_xy = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]
        self._uavs = [UAVData(i, *pos) for i, pos in enumerate(spawn_xy)]

        self._pos_subs  = []
        self._stat_subs = []
        self._ocm_pubs  = []
        self._tsp_pubs  = []
        self._vcmd_pubs = []

        for i in range(NUM_UAVS):
            # Instance 0 → bare /fmu/..., instance N → /px4_N/fmu/...
            ns = '' if i == 0 else f'/px4_{i}'
            self._pos_subs.append(self.create_subscription(
                VehicleLocalPosition,
                f'{ns}/fmu/out/vehicle_local_position_v1',
                lambda msg, idx=i: self._on_position(msg, idx),
                _QOS,
            ))
            self._stat_subs.append(self.create_subscription(
                VehicleStatus,
                f'{ns}/fmu/out/vehicle_status_v4',
                lambda msg, idx=i: self._on_status(msg, idx),
                _QOS,
            ))
            self._ocm_pubs.append(self.create_publisher(
                OffboardControlMode, f'{ns}/fmu/in/offboard_control_mode', _QOS,
            ))
            self._tsp_pubs.append(self.create_publisher(
                TrajectorySetpoint, f'{ns}/fmu/in/trajectory_setpoint', _QOS,
            ))
            self._vcmd_pubs.append(self.create_publisher(
                VehicleCommand, f'{ns}/fmu/in/vehicle_command', _QOS,
            ))

        # No obstacles — fly direct A → B (single waypoint in metric coords)
        self._path   = [(int(round(POINT_B_ENU[0] / GRID_RES)),
                         int(round(POINT_B_ENU[1] / GRID_RES)))]
        self._wp_idx = 0

        # V-formation: followers 2 m behind leader, ±1.5 m to the side
        self._formation = LeaderFollowerFormation(
            follower_offsets=[(-2.0, -1.5), (-2.0, 1.5)],
            max_speed=MAX_SPEED,
            k_p=1.2,
        )
        self._ca = CollisionAvoidance(
            time_step=1.0 / CTRL_RATE,
            neighbor_dist=6.0,
            max_neighbors=10,
            time_horizon=2.0,
            time_horizon_obst=2.0,
            radius=0.8,
            max_speed=MAX_SPEED,
        )
        self._agent_ids = [
            self._ca.add_agent(uav.x, uav.y) for uav in self._uavs
        ]
        self._fstates = [
            UAVState(uav_id=i, x=uav.x, y=uav.y)
            for i, uav in enumerate(self._uavs)
        ]

        self._goal_reached = False
        self._ts = 0  # microsecond timestamp, refreshed once per control tick

        self._timer = self.create_timer(1.0 / CTRL_RATE, self._control_loop)
        self.get_logger().info(
            f'PX4SwarmNode started — A{POINT_A_ENU} → B{POINT_B_ENU}, direct path'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_position(self, msg: VehicleLocalPosition, idx: int):
        uav = self._uavs[idx]
        # NED → ENU: x_enu=y_ned, y_enu=x_ned, z_enu=-z_ned
        uav.x = msg.y
        uav.y = msg.x
        uav.z = -msg.z

    def _on_status(self, msg: VehicleStatus, idx: int):
        uav = self._uavs[idx]
        uav.armed        = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        uav.nav_state    = msg.nav_state
        uav.preflight_pass = getattr(msg, 'pre_flight_checks_pass', False)
        if uav.preflight_pass:
            uav.preflight_stable = min(uav.preflight_stable + 1, 10)
        else:
            uav.preflight_stable = 0

    # ── Publishers ───────────────────────────────────────────────────────────

    def _pub_offboard_mode(self, idx: int):
        msg = OffboardControlMode()
        msg.timestamp    = self._ts
        msg.position     = True   # Z position hold (altitude)
        msg.velocity     = True   # XY velocity control
        msg.acceleration = False
        self._ocm_pubs[idx].publish(msg)

    def _pub_velocity(self, idx: int, vx_enu: float, vy_enu: float, vz_enu: float = 0.0):
        msg = TrajectorySetpoint()
        msg.timestamp    = self._ts
        # XY: velocity control (ENU → NED); Z: position hold at TARGET_ALT.
        # Mixed position+velocity mode prevents altitude drift from EKF noise.
        msg.velocity     = [vy_enu, vx_enu, float('nan')]
        # During takeoff we command upward velocity — use velocity-Z not position-Z.
        # In all other states, hold TARGET_ALT via position setpoint.
        if vz_enu != 0.0:
            msg.position = [float('nan'), float('nan'), float('nan')]
            msg.velocity[2] = -vz_enu  # NED: down = negative of ENU up
        else:
            msg.position = [float('nan'), float('nan'), -TARGET_ALT]  # NED z = -altitude
        msg.acceleration = [float('nan')] * 3
        msg.yaw          = float('nan')
        self._tsp_pubs[idx].publish(msg)

    def _pub_vehicle_command(self, idx: int, command: int,
                             p1: float = 0.0, p2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp        = self._ts
        msg.command          = command
        msg.param1           = p1
        msg.param2           = p2
        msg.target_system    = idx + 1
        msg.target_component = 1
        msg.source_system    = 255
        msg.source_component = 0
        msg.from_external    = True
        self._vcmd_pubs[idx].publish(msg)

    def _arm(self, idx: int):
        # p2=21196 forces arm, bypassing preflight health checks (SITL)
        self._pub_vehicle_command(
            idx, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0, p2=21196.0,
        )

    def _set_offboard_mode(self, idx: int):
        self._pub_vehicle_command(
            idx, VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            p1=1.0, p2=6.0,  # custom_mode=6 → OFFBOARD
        )

    # ── Control loop ─────────────────────────────────────────────────────────

    def _control_loop(self):
        self._ts = int(time.time() * 1e6)  # one timestamp for the whole tick
        for i, uav in enumerate(self._uavs):
            self._pub_offboard_mode(i)
            uav.offboard_count += 1

        if self._goal_reached:
            for i in range(NUM_UAVS):
                self._pub_velocity(i, 0.0, 0.0)
            return

        self._run_state_machine()

    def _run_state_machine(self):
        all_flying = True

        for i, uav in enumerate(self._uavs):
            if uav.flight_state == FlightState.PREFLIGHT:
                self._pub_velocity(i, 0.0, 0.0)
                if uav.offboard_count > 10:
                    uav.preflight_wait += 1
                    # Require 3 consecutive preflight_pass ticks (~150ms) before arming.
                    # Fall back to force-arm after 30s if EKF2 never fully settles.
                    ready = uav.preflight_stable >= 3
                    timeout = uav.preflight_wait >= 600  # 30s at 20Hz
                    if ready or timeout:
                        self._set_offboard_mode(i)
                        self._arm(i)
                        uav.flight_state = FlightState.ARMING
                        reason = 'preflight OK' if ready else 'timeout→force-arm'
                        self.get_logger().info(f'UAV {i}: ARMING ({reason})...')
                all_flying = False

            elif uav.flight_state == FlightState.ARMING:
                self._pub_velocity(i, 0.0, 0.0)
                uav.arm_retry_count += 1
                if uav.arm_retry_count % 80 == 0:
                    self.get_logger().info(
                        f'UAV {i}: retry (armed={uav.armed}, nav_state={uav.nav_state})'
                    )
                    self._set_offboard_mode(i)
                    if not uav.armed:
                        self._arm(i)
                if uav.armed and uav.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    uav.flight_state = FlightState.TAKEOFF
                    self.get_logger().info(f'UAV {i}: TAKEOFF → {TARGET_ALT}m')
                all_flying = False

            elif uav.flight_state == FlightState.TAKEOFF:
                if not uav.armed:
                    uav.flight_state = FlightState.ARMING
                    uav.arm_retry_count = 0
                    self.get_logger().info(f'UAV {i}: disarmed, retry ARMING')
                    self._pub_velocity(i, 0.0, 0.0)
                    all_flying = False
                    continue
                err_z = TARGET_ALT - uav.z
                if err_z > ALT_TOLERANCE:
                    self._pub_velocity(i, 0.0, 0.0, TAKEOFF_SPEED)
                else:
                    uav.flight_state = FlightState.FLYING
                    self.get_logger().info(f'UAV {i}: FLYING')
                all_flying = False

            elif uav.flight_state == FlightState.ARRIVED:
                self._pub_velocity(i, 0.0, 0.0)

        if not all_flying:
            # UAVs already in FLYING state hover while waiting for others to catch up
            for i, uav in enumerate(self._uavs):
                if uav.flight_state == FlightState.FLYING:
                    self._pub_velocity(i, 0.0, 0.0)
            return

        self._run_formation_and_orca()

    def _run_formation_and_orca(self):
        for uav, fs in zip(self._uavs, self._fstates):
            fs.x, fs.y = uav.x, uav.y

        leader_fs    = self._fstates[0]
        followers_fs = self._fstates[1:]

        (lvx, lvy), self._wp_idx = self._formation.compute_leader_velocity(
            leader_fs, self._path, self._wp_idx, GRID_RES
        )
        leader_fs.vx, leader_fs.vy = lvx, lvy

        for fs, (fvx, fvy) in zip(
            followers_fs,
            self._formation.compute_follower_velocities(leader_fs, followers_fs),
        ):
            fs.vx, fs.vy = fvx, fvy

        for fs, aid in zip(self._fstates, self._agent_ids):
            self._ca.update_agent_position(aid, fs.x, fs.y)
            self._ca.set_preferred_velocity(aid, fs.vx, fs.vy)
        self._ca.step()

        for i, aid in enumerate(self._agent_ids):
            vx, vy = self._ca.get_velocity(aid)
            self._fstates[i].vx = vx
            self._fstates[i].vy = vy
            self._pub_velocity(i, vx, vy, 0.0)

        if self._wp_idx >= len(self._path):
            leader = self._uavs[0]
            if (leader.x - POINT_B_ENU[0]) ** 2 + (leader.y - POINT_B_ENU[1]) ** 2 < 1.0:
                for uav in self._uavs:
                    uav.flight_state = FlightState.ARRIVED
                self._goal_reached = True
                self.get_logger().info(
                    f'✅ Swarm đã đến điểm B {POINT_B_ENU} — dừng bay!'
                )


def main(args=None):
    rclpy.init(args=args)
    node = PX4SwarmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
