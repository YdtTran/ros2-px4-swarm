"""
PX4SwarmNode — điều khiển bầy đàn 3 UAV.

Commands (ARM, OFFBOARD, velocity) → pymavlink direct UDP → PX4
State + position                   → MAVROS subscriptions (nhận-only)
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode

from pymavlink import mavutil

from uav_swarm_demo.algorithms.planner import GlobalPlanner
from uav_swarm_demo.algorithms.formation import LeaderFollowerFormation, UAVState
from uav_swarm_demo.algorithms.collision_avoidance import CollisionAvoidance


NUM_UAVS      = 3
USE_WALL      = False  # True = có bức tường ngang giữa map
TARGET_ALT    = 5.0    # m (ENU Up)
TAKEOFF_SPEED = 1.5    # m/s
ALT_TOLERANCE = 0.3    # m
MAX_SPEED     = 2.0    # m/s
GRID_RES      = 2.0    # m/cell
CTRL_RATE     = 20.0   # Hz

_VEL_ONLY_MASK = 0b0000110111000111


class State_(Enum):
    PREFLIGHT = auto()
    ARMING    = auto()
    TAKEOFF   = auto()
    FLYING    = auto()
    ARRIVED   = auto()


def _build_map(rows: int = 15, cols: int = 15) -> list[list[int]]:
    grid = [[1] * cols for _ in range(rows)]
    if USE_WALL:
        wall_row = rows // 2
        gap = cols // 2
        for c in range(cols):
            if c != gap:
                grid[wall_row][c] = 0
    return grid


class UAVData:
    def __init__(self, uav_id: int, init_x: float, init_y: float):
        self.id        = uav_id
        self.state     = State_.PREFLIGHT
        self.connected = False
        self.armed     = False
        self.mode      = ''
        self.x         = init_x   # ENU East  (m)
        self.y         = init_y   # ENU North (m)
        self.z         = 0.0      # ENU Up    (m)
        self.setpoint_count  = 0
        self.arm_retry_count = 0


class PX4SwarmNode(Node):

    def __init__(self):
        super().__init__('px4_swarm_node')

        # MAVROS state topic: RELIABLE + VOLATILE (MAVROS2 publishes VOLATILE)
        qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # Sensor data: BEST_EFFORT
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Spawn positions (ENU): PX4_GZ_MODEL_POSE="i*2,0,0,0,0,0" → East axis
        spawn_positions = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0)]
        self._uavs = [UAVData(i, *pos) for i, pos in enumerate(spawn_positions)]

        # ── MAVROS subscriptions (read-only) ──────────────────────────────
        self._state_subs = []
        self._pose_subs  = []

        for i in range(NUM_UAVS):
            ns = f'/uav{i}'
            self._state_subs.append(self.create_subscription(
                State,
                f'{ns}/state',
                lambda msg, idx=i: self._on_state(msg, idx),
                qos_state,
            ))
            self._pose_subs.append(self.create_subscription(
                PoseStamped,
                f'{ns}/local_position/pose',
                lambda msg, idx=i: self._on_pose(msg, idx),
                qos_sensor,
            ))

        # ── MAVROS: set_mode service (dùng để switch OFFBOARD) ──────────
        self._set_mode_clients = [
            self.create_client(SetMode, f'/uav{i}/set_mode')
            for i in range(NUM_UAVS)
        ]

        # ── pymavlink: kết nối UDP trực tiếp tới từng PX4 instance ───────
        # PX4 instance i lắng nghe trên port 14580+i
        self._mav = []
        for i in range(NUM_UAVS):
            port = 14580 + i
            conn = mavutil.mavlink_connection(
                f'udpout:localhost:{port}',
                source_system=255,
                source_component=190,
            )
            self._mav.append(conn)
            self.get_logger().info(f'UAV {i}: pymavlink → localhost:{port}')

        # ── Algorithms ───────────────────────────────────────────────────
        self._map     = _build_map()
        self._planner = GlobalPlanner(self._map)
        self._path    = None
        self._wp_idx  = 0

        self._formation = LeaderFollowerFormation(
            follower_offsets=[(-3.0, -2.0), (-3.0, 2.0)],
            max_speed=MAX_SPEED,
            k_p=1.2,
        )
        self._ca = CollisionAvoidance(
            time_step=1.0 / CTRL_RATE,
            neighbor_dist=8.0,
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

        self._goal_grid    = (12, 12)
        self._goal_reached = False

        self._timer = self.create_timer(1.0 / CTRL_RATE, self._control_loop)
        self.get_logger().info('PX4SwarmNode started (pymavlink direct).')

    # ── MAVROS callbacks ─────────────────────────────────────────────────

    def _on_state(self, msg: State, idx: int):
        self._uavs[idx].connected = msg.connected
        self._uavs[idx].armed     = msg.armed
        self._uavs[idx].mode      = msg.mode

    def _on_pose(self, msg: PoseStamped, idx: int):
        self._uavs[idx].x = msg.pose.position.x
        self._uavs[idx].y = msg.pose.position.y
        self._uavs[idx].z = msg.pose.position.z

    # ── pymavlink commands ───────────────────────────────────────────────

    def _send_vel(self, idx: int, vx_enu: float, vy_enu: float, vz_enu: float = 0.0):
        """Gửi velocity setpoint tới PX4 qua MAVLink (ENU → NED)."""
        # ENU→NED: x_ned=y_enu, y_ned=x_enu, z_ned=-z_enu
        self._mav[idx].mav.set_position_target_local_ned_send(
            0,            # time_boot_ms
            idx + 1,      # target_system
            1,            # target_component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            _VEL_ONLY_MASK,
            0, 0, 0,                          # x, y, z (ignored)
            vy_enu, vx_enu, -vz_enu,          # vx_ned, vy_ned, vz_ned
            0, 0, 0,                          # ax, ay, az (ignored)
            0, 0,                             # yaw, yaw_rate (ignored)
        )

    def _arm(self, idx: int):
        self._mav[idx].mav.command_long_send(
            idx + 1, 1,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0, 21196.0, 0, 0, 0, 0, 0,  # param1=1 ARM, param2=21196 force
        )

    def _set_offboard(self, idx: int):
        client = self._set_mode_clients[idx]
        if client.service_is_ready():
            req = SetMode.Request()
            req.base_mode = 0
            req.custom_mode = 'OFFBOARD'
            client.call_async(req)

    # ── Control loop ─────────────────────────────────────────────────────

    def _control_loop(self):
        if self._goal_reached:
            for i in range(NUM_UAVS):
                self._send_vel(i, 0.0, 0.0)
            return

        # Stream zero setpoints via pymavlink (PX4 cần >2Hz cho OFFBOARD keepalive)
        for i, uav in enumerate(self._uavs):
            self._send_vel(i, 0.0, 0.0)
            uav.setpoint_count += 1

        self._run_state_machine()

    def _run_state_machine(self):
        all_flying = True

        for i, uav in enumerate(self._uavs):
            if uav.state == State_.PREFLIGHT:
                if uav.connected and uav.setpoint_count > 50:
                    self._set_offboard(i)
                    self._arm(i)
                    uav.state = State_.ARMING
                    self.get_logger().info(f'UAV {i}: ARMING...')
                elif not uav.connected and uav.setpoint_count % 40 == 1:
                    self.get_logger().info(f'UAV {i}: chờ FCU kết nối...')
                all_flying = False

            elif uav.state == State_.ARMING:
                uav.arm_retry_count += 1
                if uav.arm_retry_count % 40 == 0:
                    self.get_logger().info(
                        f'UAV {i}: retry '
                        f'(connected={uav.connected}, armed={uav.armed}, mode={uav.mode!r})'
                    )
                    self._set_offboard(i)
                    if not uav.armed:
                        self._arm(i)
                if uav.armed and uav.mode == 'OFFBOARD':
                    uav.state = State_.TAKEOFF
                    self.get_logger().info(f'UAV {i}: TAKEOFF → {TARGET_ALT}m')
                all_flying = False

            elif uav.state == State_.TAKEOFF:
                if not uav.armed:
                    # PX4 auto-disarmed (COM_DISARM_PRFLT timeout) → retry
                    uav.state = State_.ARMING
                    uav.arm_retry_count = 0
                    self.get_logger().info(f'UAV {i}: disarmed during TAKEOFF, retry ARMING')
                    all_flying = False
                    continue
                err_z = TARGET_ALT - uav.z
                if err_z > ALT_TOLERANCE:
                    self._send_vel(i, 0.0, 0.0, TAKEOFF_SPEED)
                else:
                    uav.state = State_.FLYING
                    self.get_logger().info(f'UAV {i}: FLYING')
                all_flying = False

            elif uav.state == State_.ARRIVED:
                self._send_vel(i, 0.0, 0.0)

        if not all_flying:
            return

        self._run_formation_and_orca()

    def _run_formation_and_orca(self):
        if self._path is None:
            leader = self._uavs[0]
            sx = int(round(leader.x / GRID_RES))
            sy = int(round(leader.y / GRID_RES))
            self._path = self._planner.plan(start=(sx, sy), goal=self._goal_grid)
            if self._path is None:
                self.get_logger().error('A*: không tìm được đường!')
                return
            self.get_logger().info(f'A* path: {len(self._path)} waypoints')

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
            self._send_vel(i, vx, vy, 0.0)

        if self._wp_idx >= len(self._path):
            leader = self._uavs[0]
            gx_m = self._goal_grid[0] * GRID_RES
            gy_m = self._goal_grid[1] * GRID_RES
            if math.sqrt((leader.x - gx_m) ** 2 + (leader.y - gy_m) ** 2) < 1.0:
                for uav in self._uavs:
                    uav.state = State_.ARRIVED
                self._goal_reached = True
                self.get_logger().info('✅ Swarm đã đến đích!')


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
