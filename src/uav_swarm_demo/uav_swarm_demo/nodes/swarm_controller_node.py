"""
SwarmControllerNode — central controller for a 3-UAV Leader-Follower swarm.

Pipeline (runs every timer tick):
  1. Formation   → compute preferred velocity for each UAV
  2. ORCA        → compute collision-free velocity for each UAV
  3. Integrate   → update UAV positions inside the simulator
  4. Publish     → PoseArray + MarkerArray (RViz) + nav_msgs/Path (A* path)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseArray, Pose, Point, Vector3
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from uav_swarm_demo.algorithms.planner import GlobalPlanner
from uav_swarm_demo.algorithms.formation import LeaderFollowerFormation, UAVState
from uav_swarm_demo.algorithms.collision_avoidance import CollisionAvoidance


# ---------------------------------------------------------------------------
# Default obstacle map (1 = free, 0 = obstacle)  20×20 grid
# Modify or replace this with dynamic map loading as needed.
# ---------------------------------------------------------------------------
def _build_default_map(rows: int = 20, cols: int = 20) -> list[list[int]]:
    grid = [[1] * cols for _ in range(rows)]
    # Horizontal wall with a gap
    wall_row = rows // 2
    gap = cols // 2
    for c in range(cols):
        if c != gap:
            grid[wall_row][c] = 0
    return grid


class SwarmControllerNode(Node):

    def __init__(self):
        super().__init__('swarm_controller')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('start_x',         0)
        self.declare_parameter('start_y',         0)
        self.declare_parameter('goal_x',          19)
        self.declare_parameter('goal_y',          19)
        self.declare_parameter('grid_resolution', 1.0)   # m / cell
        self.declare_parameter('altitude',        5.0)   # fixed Z (m)
        self.declare_parameter('max_speed',       2.0)   # m/s
        self.declare_parameter('update_rate',     10.0)  # Hz
        self.declare_parameter('frame_id',        'map')

        p = self.get_parameters([
            'start_x', 'start_y', 'goal_x', 'goal_y',
            'grid_resolution', 'altitude', 'max_speed',
            'update_rate', 'frame_id',
        ])
        self._sx        = p[0].value
        self._sy        = p[1].value
        self._gx        = p[2].value
        self._gy        = p[3].value
        self._res       = p[4].value        # grid_resolution
        self._alt       = p[5].value        # altitude
        self._max_spd   = p[6].value
        self._rate      = p[7].value
        self._frame     = p[8].value

        # ── Global planner (A*) ───────────────────────────────────────────
        self._map = _build_default_map()
        self._planner = GlobalPlanner(self._map)
        self._path = self._planner.plan(
            start=(self._sx, self._sy),
            goal=(self._gx, self._gy),
        )
        if self._path is None:
            self.get_logger().error('A*: no path found! Check start/goal/map.')
            return
        self.get_logger().info(
            f'A* path found: {len(self._path)} waypoints  '
            f'({self._sx},{self._sy}) → ({self._gx},{self._gy})'
        )
        self._wp_idx = 0   # current waypoint index for the leader

        # ── Formation controller ──────────────────────────────────────────
        self._formation = LeaderFollowerFormation(
            follower_offsets=[(-2.0, -2.0), (-2.0, 2.0)],
            max_speed=self._max_spd,
            k_p=1.5,
        )

        # Internal UAV states (used for formation logic)
        sx_m = self._sx * self._res
        sy_m = self._sy * self._res
        self._uavs = [
            UAVState(uav_id=0, x=sx_m,       y=sy_m),        # leader
            UAVState(uav_id=1, x=sx_m - 2.0, y=sy_m - 2.0),  # follower 1
            UAVState(uav_id=2, x=sx_m - 2.0, y=sy_m + 2.0),  # follower 2
        ]

        # ── ORCA collision avoidance (pyrvo) ──────────────────────────────
        self._ca = CollisionAvoidance(
            time_step=1.0 / self._rate,
            neighbor_dist=6.0,
            max_neighbors=10,
            time_horizon=2.0,
            time_horizon_obst=2.0,
            radius=0.6,
            max_speed=self._max_spd,
        )
        self._agent_ids = [
            self._ca.add_agent(uav.x, uav.y) for uav in self._uavs
        ]

        # ── Publishers ────────────────────────────────────────────────────
        self._pub_poses   = self.create_publisher(PoseArray,   '/uav_swarm/poses',   10)
        self._pub_markers = self.create_publisher(MarkerArray, '/uav_swarm/markers', 10)
        self._pub_path    = self.create_publisher(Path,        '/uav_swarm/path',    10)

        # ── Timer ─────────────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / self._rate, self._control_loop)

        self._goal_reached = False
        self.get_logger().info('SwarmControllerNode started.')

    # -----------------------------------------------------------------------
    # Main control loop
    # -----------------------------------------------------------------------

    def _control_loop(self):
        if self._goal_reached or self._path is None:
            return

        leader = self._uavs[0]
        followers = self._uavs[1:]

        # ── 1. Formation: preferred velocities ────────────────────────────
        (lvx, lvy), self._wp_idx = self._formation.compute_leader_velocity(
            leader, self._path, self._wp_idx, self._res
        )
        leader.vx, leader.vy = lvx, lvy

        f_vels = self._formation.compute_follower_velocities(leader, followers)
        for f, (fvx, fvy) in zip(followers, f_vels):
            f.vx, f.vy = fvx, fvy

        # ── 2. ORCA: set preferred velocities ─────────────────────────────
        for uav, aid in zip(self._uavs, self._agent_ids):
            self._ca.set_preferred_velocity(aid, uav.vx, uav.vy)

        self._ca.step()

        # ── 3. Sync positions back to internal state ───────────────────────
        for uav, aid in zip(self._uavs, self._agent_ids):
            uav.x, uav.y = self._ca.get_position(aid)
            uav.vx, uav.vy = self._ca.get_velocity(aid)

        # ── 4. Publish ────────────────────────────────────────────────────
        now = self.get_clock().now().to_msg()
        self._publish_poses(now)
        self._publish_markers(now)
        self._publish_path(now)

        # ── 5. Goal check ─────────────────────────────────────────────────
        if self._wp_idx >= len(self._path):
            gx_m = self._gx * self._res
            gy_m = self._gy * self._res
            d = math.sqrt((leader.x - gx_m) ** 2 + (leader.y - gy_m) ** 2)
            if d < 0.5:
                self._goal_reached = True
                self.get_logger().info('Goal reached! Swarm stopped.')

    # -----------------------------------------------------------------------
    # Publishers
    # -----------------------------------------------------------------------

    def _header(self, stamp) -> Header:
        h = Header()
        h.stamp = stamp
        h.frame_id = self._frame
        return h

    def _publish_poses(self, stamp):
        msg = PoseArray()
        msg.header = self._header(stamp)
        for uav in self._uavs:
            p = Pose()
            p.position.x = uav.x
            p.position.y = uav.y
            p.position.z = self._alt
            p.orientation.w = 1.0
            msg.poses.append(p)
        self._pub_poses.publish(msg)

    def _publish_markers(self, stamp):
        markers = MarkerArray()
        colors = [
            (1.0, 0.2, 0.2),   # UAV 0 leader  — red
            (0.2, 0.8, 0.2),   # UAV 1 follower — green
            (0.2, 0.4, 1.0),   # UAV 2 follower — blue
        ]
        for i, (uav, (r, g, b)) in enumerate(zip(self._uavs, colors)):
            m = Marker()
            m.header = self._header(stamp)
            m.ns = 'uavs'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = uav.x
            m.pose.position.y = uav.y
            m.pose.position.z = self._alt
            m.pose.orientation.w = 1.0
            m.scale = Vector3(x=1.0, y=1.0, z=0.4)
            m.color = ColorRGBA(r=r, g=g, b=b, a=0.9)
            markers.markers.append(m)

            # Velocity arrow
            arrow = Marker()
            arrow.header = self._header(stamp)
            arrow.ns = 'velocities'
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(x=uav.x, y=uav.y, z=self._alt),
                Point(x=uav.x + uav.vx * 0.5,
                      y=uav.y + uav.vy * 0.5,
                      z=self._alt),
            ]
            arrow.scale = Vector3(x=0.1, y=0.2, z=0.2)
            arrow.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)
            markers.markers.append(arrow)

        self._pub_markers.publish(markers)

    def _publish_path(self, stamp):
        msg = Path()
        msg.header = self._header(stamp)
        for (wx, wy) in self._path:
            ps = PoseStamped()
            ps.header = self._header(stamp)
            ps.pose.position.x = wx * self._res
            ps.pose.position.y = wy * self._res
            ps.pose.position.z = self._alt
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._pub_path.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = SwarmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
