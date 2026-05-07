"""
PX4SwarmNode — điều khiển bầy đàn 3 UAV qua PX4 offboard mode.

Pipeline:
  1. ARM + OFFBOARD MODE cho cả 3 UAV
  2. TAKEOFF đến độ cao mục tiêu
  3. FORMATION FLIGHT:
       A* → waypoints cho Leader
       Leader-Follower → preferred velocity cho Follower
       ORCA → collision-free velocity cho tất cả
       → TrajectorySetpoint (velocity) → PX4

Frame convention:
  - PX4 / XRCE-DDS: NED (North-East-Down)
  - Algorithms:      ENU (East-North-Up, x=East, y=North)
  - Conversion ENU→NED: x_ned=y_enu, y_ned=x_enu, z_ned=-z_enu
"""

import math
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

from uav_swarm_demo.algorithms.planner import GlobalPlanner
from uav_swarm_demo.algorithms.formation import LeaderFollowerFormation, UAVState
from uav_swarm_demo.algorithms.collision_avoidance import CollisionAvoidance


# ── Tham số mô phỏng ────────────────────────────────────────────────────────
NUM_UAVS       = 3
TARGET_ALT     = 5.0      # m (ENU, dương = lên trên)
TAKEOFF_SPEED  = 1.5      # m/s
ALT_TOLERANCE  = 0.3      # m
MAX_SPEED      = 2.0      # m/s
GRID_RES       = 2.0      # m/ô lưới
CTRL_RATE      = 20.0     # Hz (phải > 2 Hz cho offboard)


class State(Enum):
    PREFLIGHT  = auto()   # Chờ kết nối PX4
    ARMING     = auto()   # Đang gửi lệnh arm
    TAKEOFF    = auto()   # Bay lên độ cao mục tiêu
    FLYING     = auto()   # Bay đội hình
    ARRIVED    = auto()   # Đã đến đích


def _build_map(rows: int = 15, cols: int = 15) -> list[list[int]]:
    """Bản đồ 15×15 ô, mỗi ô = GRID_RES mét. 1=free, 0=obstacle."""
    grid = [[1] * cols for _ in range(rows)]
    # Tường ngang giữa bản đồ, có lỗ hở
    wall_row = rows // 2
    gap      = cols // 2
    for c in range(cols):
        if c != gap:
            grid[wall_row][c] = 0
    return grid


# ── Chuyển đổi frame ─────────────────────────────────────────────────────────

def enu_to_ned_vel(vx_e: float, vy_e: float) -> tuple[float, float, float]:
    """ENU velocity (x=East, y=North) → NED (x=North, y=East, z=Down)."""
    return vy_e, vx_e, 0.0


def ned_to_enu_pos(x_n: float, y_n: float) -> tuple[float, float]:
    """NED position → ENU (x=East, y=North)."""
    return y_n, x_n


# ── Dữ liệu trạng thái mỗi UAV ───────────────────────────────────────────────

class UAVData:
    def __init__(self, uav_id: int, init_x_enu: float, init_y_enu: float):
        self.id          = uav_id
        self.state       = State.PREFLIGHT
        self.armed       = False
        self.nav_state   = 0
        # Vị trí ENU (m)
        self.x_enu       = init_x_enu
        self.y_enu       = init_y_enu
        self.z_enu       = 0.0
        # Vị trí NED từ PX4
        self.x_ned       = 0.0
        self.y_ned       = 0.0
        self.z_ned       = 0.0
        self.offboard_counter = 0


# ── Node chính ────────────────────────────────────────────────────────────────

class PX4SwarmNode(Node):

    def __init__(self):
        super().__init__('px4_swarm_node')

        # QoS profile cho px4_msgs (best-effort, volatile)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Vị trí spawn ban đầu của 3 UAV (x=0,2,4 m theo NED North → ENU East)
        # run_swarm.sh spawn tại x=0, x=2, x=4 (NED frame)
        # → ENU: y=0,2,4 (North), x=0 (East)
        spawn_positions = [(0.0, 0.0), (0.0, 2.0), (0.0, 4.0)]  # (x_enu, y_enu)
        self._uavs = [UAVData(i, *pos) for i, pos in enumerate(spawn_positions)]

        # ── Subscribers & Publishers cho mỗi UAV ────────────────────────
        self._pos_subs   = []
        self._stat_subs  = []
        self._ocm_pubs   = []   # OffboardControlMode
        self._tsp_pubs   = []   # TrajectorySetpoint
        self._vcmd_pubs  = []   # VehicleCommand

        for i in range(NUM_UAVS):
            ns = f'/px4_{i}'
            self._pos_subs.append(self.create_subscription(
                VehicleLocalPosition,
                f'{ns}/fmu/out/vehicle_local_position',
                lambda msg, idx=i: self._on_position(msg, idx),
                qos,
            ))
            self._stat_subs.append(self.create_subscription(
                VehicleStatus,
                f'{ns}/fmu/out/vehicle_status',
                lambda msg, idx=i: self._on_status(msg, idx),
                qos,
            ))
            self._ocm_pubs.append(self.create_publisher(
                OffboardControlMode,
                f'{ns}/fmu/in/offboard_control_mode',
                qos,
            ))
            self._tsp_pubs.append(self.create_publisher(
                TrajectorySetpoint,
                f'{ns}/fmu/in/trajectory_setpoint',
                qos,
            ))
            self._vcmd_pubs.append(self.create_publisher(
                VehicleCommand,
                f'{ns}/fmu/in/vehicle_command',
                qos,
            ))

        # ── Thuật toán ───────────────────────────────────────────────────
        self._map      = _build_map()
        self._planner  = GlobalPlanner(self._map)
        self._path     = None      # A* path (grid cells)
        self._wp_idx   = 0

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
        # Agent IDs trong ORCA khớp với UAV index
        self._agent_ids = [
            self._ca.add_agent(uav.x_enu, uav.y_enu)
            for uav in self._uavs
        ]

        # Formation UAVState (dùng cho formation controller)
        self._fstates = [
            UAVState(uav_id=i, x=uav.x_enu, y=uav.y_enu)
            for i, uav in enumerate(self._uavs)
        ]

        # Goal (grid) — UAV bay từ (0,0) → (12,12)
        self._goal_grid = (12, 12)
        self._goal_reached = False

        # ── Timer điều khiển ─────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / CTRL_RATE, self._control_loop)
        self.get_logger().info('PX4SwarmNode started — chờ kết nối UAV...')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_position(self, msg: VehicleLocalPosition, idx: int):
        uav = self._uavs[idx]
        uav.x_ned = msg.x
        uav.y_ned = msg.y
        uav.z_ned = msg.z
        # Chuyển về ENU để dùng trong thuật toán
        uav.x_enu, uav.y_enu = ned_to_enu_pos(msg.x, msg.y)
        uav.z_enu = -msg.z   # Down → Up

    def _on_status(self, msg: VehicleStatus, idx: int):
        uav = self._uavs[idx]
        uav.armed     = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        uav.nav_state = msg.nav_state

    # ── Publishers helper ─────────────────────────────────────────────────────

    def _pub_offboard_mode(self, idx: int):
        msg = OffboardControlMode()
        msg.timestamp  = self._px4_timestamp()
        msg.position   = False
        msg.velocity   = True
        msg.acceleration = False
        self._ocm_pubs[idx].publish(msg)

    def _pub_velocity(self, idx: int, vx_enu: float, vy_enu: float, vz_enu: float = 0.0):
        vx_ned, vy_ned, vz_ned = enu_to_ned_vel(vx_enu, vy_enu)
        vz_ned = vz_enu * -1.0   # ENU up → NED down (negative)
        msg = TrajectorySetpoint()
        msg.timestamp    = self._px4_timestamp()
        msg.velocity     = [vx_ned, vy_ned, vz_ned]
        msg.position     = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.yaw          = float('nan')
        self._tsp_pubs[idx].publish(msg)

    def _pub_vehicle_command(self, idx: int, command: int, p1: float = 0.0, p2: float = 0.0):
        msg = VehicleCommand()
        msg.timestamp        = self._px4_timestamp()
        msg.command          = command
        msg.param1           = p1
        msg.param2           = p2
        msg.target_system    = idx + 1   # PX4 system ID = instance + 1
        msg.target_component = 1
        msg.source_system    = 255
        msg.source_component = 0
        msg.from_external    = True
        self._vcmd_pubs[idx].publish(msg)

    def _arm(self, idx: int):
        self._pub_vehicle_command(
            idx,
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            p1=1.0,
        )

    def _set_offboard_mode(self, idx: int):
        self._pub_vehicle_command(
            idx,
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            p1=1.0,   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            p2=6.0,   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        )

    @staticmethod
    def _px4_timestamp() -> int:
        import time
        return int(time.time() * 1e6)

    # ── Vòng điều khiển chính ─────────────────────────────────────────────────

    def _control_loop(self):
        if self._goal_reached:
            # Hover tại chỗ
            for i in range(NUM_UAVS):
                self._pub_offboard_mode(i)
                self._pub_velocity(i, 0.0, 0.0)
            return

        for i, uav in enumerate(self._uavs):
            self._pub_offboard_mode(i)
            uav.offboard_counter += 1

        self._run_state_machine()

    def _run_state_machine(self):
        all_flying = True

        for i, uav in enumerate(self._uavs):
            if uav.state == State.PREFLIGHT:
                # Phải publish offboard mode trước khi switch (>10 lần)
                if uav.offboard_counter > 10:
                    self._set_offboard_mode(i)
                    self._arm(i)
                    uav.state = State.ARMING
                    self.get_logger().info(f'UAV {i}: ARMING...')
                self._pub_velocity(i, 0.0, 0.0, 0.0)
                all_flying = False

            elif uav.state == State.ARMING:
                self._pub_velocity(i, 0.0, 0.0, 0.0)
                if uav.armed:
                    uav.state = State.TAKEOFF
                    self.get_logger().info(f'UAV {i}: TAKEOFF → {TARGET_ALT}m')
                all_flying = False

            elif uav.state == State.TAKEOFF:
                err_z = TARGET_ALT - uav.z_enu
                if err_z > ALT_TOLERANCE:
                    self._pub_velocity(i, 0.0, 0.0, TAKEOFF_SPEED)
                else:
                    uav.state = State.FLYING
                    self.get_logger().info(f'UAV {i}: FLYING — vào đội hình')
                all_flying = False

            elif uav.state == State.FLYING:
                pass   # Xử lý phía dưới

            elif uav.state == State.ARRIVED:
                self._pub_velocity(i, 0.0, 0.0)

        if not all_flying:
            return

        # ── Tất cả UAV đang FLYING → chạy thuật toán ────────────────────
        self._run_formation_and_orca()

    def _run_formation_and_orca(self):
        # Lần đầu: tính A* path
        if self._path is None:
            # Vị trí leader → grid cell
            leader = self._uavs[0]
            sx = int(leader.x_enu / GRID_RES)
            sy = int(leader.y_enu / GRID_RES)
            gx, gy = self._goal_grid
            self._path = self._planner.plan(start=(sx, sy), goal=(gx, gy))
            if self._path is None:
                self.get_logger().error('A*: không tìm được đường!')
                return
            self.get_logger().info(f'A* path: {len(self._path)} waypoints')

        # Sync vị trí ENU từ UAV data vào formation states
        for i, (uav, fs) in enumerate(zip(self._uavs, self._fstates)):
            fs.x, fs.y = uav.x_enu, uav.y_enu

        leader_fs = self._fstates[0]
        followers_fs = self._fstates[1:]

        # Formation: preferred velocities
        (lvx, lvy), self._wp_idx = self._formation.compute_leader_velocity(
            leader_fs, self._path, self._wp_idx, GRID_RES
        )
        leader_fs.vx, leader_fs.vy = lvx, lvy

        f_vels = self._formation.compute_follower_velocities(leader_fs, followers_fs)
        for fs, (fvx, fvy) in zip(followers_fs, f_vels):
            fs.vx, fs.vy = fvx, fvy

        # ORCA: set preferred velocity + step
        for fs, aid in zip(self._fstates, self._agent_ids):
            self._ca.set_preferred_velocity(aid, fs.vx, fs.vy)
        self._ca.step()

        # Publish velocity commands
        for i, (uav, aid) in enumerate(zip(self._uavs, self._agent_ids)):
            vx_enu, vy_enu = self._ca.get_velocity(aid)
            self._pub_velocity(i, vx_enu, vy_enu, 0.0)

        # Goal check
        if self._wp_idx >= len(self._path):
            leader = self._uavs[0]
            gx_m = self._goal_grid[0] * GRID_RES
            gy_m = self._goal_grid[1] * GRID_RES
            if math.sqrt((leader.x_enu - gx_m) ** 2 + (leader.y_enu - gy_m) ** 2) < 1.0:
                for uav in self._uavs:
                    uav.state = State.ARRIVED
                self._goal_reached = True
                self.get_logger().info('✅ Swarm đã đến đích!')


# ── Entry point ───────────────────────────────────────────────────────────────

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
