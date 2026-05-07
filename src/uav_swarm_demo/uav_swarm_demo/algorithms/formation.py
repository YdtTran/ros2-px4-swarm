import math
from typing import List, Tuple, Optional


class UAVState:
    def __init__(self, uav_id: int, x: float, y: float, vx: float = 0.0, vy: float = 0.0):
        self.id = uav_id
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy

    @property
    def position(self) -> Tuple[float, float]:
        return (self.x, self.y)


class LeaderFollowerFormation:
    """
    Leader-Follower formation control for 3 UAVs.

    Formation layout (default: V-shape):
           [Leader]
          /         \\
    [Follower1]  [Follower2]

    Followers maintain a fixed offset relative to the Leader's
    position and heading direction.
    """

    def __init__(
        self,
        follower_offsets: Optional[List[Tuple[float, float]]] = None,
        max_speed: float = 2.0,
        k_p: float = 1.0,
    ):
        """
        Args:
            follower_offsets: list of (dx, dy) offsets in leader's local frame
                              for each follower. Default: V-shape.
            max_speed: maximum speed for followers (m/s)
            k_p: proportional gain for position error
        """
        if follower_offsets is None:
            # V-shape: followers behind and to each side
            follower_offsets = [(-2.0, -2.0), (-2.0, 2.0)]

        self.offsets = follower_offsets
        self.max_speed = max_speed
        self.k_p = k_p

    def _rotate_offset(
        self, offset: Tuple[float, float], heading: float
    ) -> Tuple[float, float]:
        """Rotate a local-frame offset by the leader's heading angle."""
        dx, dy = offset
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        rx = dx * cos_h - dy * sin_h
        ry = dx * sin_h + dy * cos_h
        return (rx, ry)

    def _clamp_velocity(self, vx: float, vy: float) -> Tuple[float, float]:
        speed = math.sqrt(vx ** 2 + vy ** 2)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            return (vx * scale, vy * scale)
        return (vx, vy)

    def get_leader_heading(self, leader: UAVState) -> float:
        """Heading angle from leader's velocity vector."""
        if abs(leader.vx) < 1e-6 and abs(leader.vy) < 1e-6:
            return 0.0
        return math.atan2(leader.vy, leader.vx)

    def compute_follower_targets(
        self, leader: UAVState
    ) -> List[Tuple[float, float]]:
        """
        Compute target positions for all followers based on leader state.

        Returns:
            List of (target_x, target_y) for each follower.
        """
        heading = self.get_leader_heading(leader)
        targets = []
        for offset in self.offsets:
            rot_offset = self._rotate_offset(offset, heading)
            tx = leader.x + rot_offset[0]
            ty = leader.y + rot_offset[1]
            targets.append((tx, ty))
        return targets

    def compute_follower_velocities(
        self,
        leader: UAVState,
        followers: List[UAVState],
    ) -> List[Tuple[float, float]]:
        """
        Compute desired velocities for each follower to reach their target.

        Uses proportional controller:  v = k_p * (target - current)

        Args:
            leader:    current leader state
            followers: list of follower UAVStates (same order as offsets)

        Returns:
            List of (vx, vy) desired velocity for each follower.
        """
        targets = self.compute_follower_targets(leader)
        velocities = []

        for follower, (tx, ty) in zip(followers, targets):
            ex = tx - follower.x
            ey = ty - follower.y
            vx = self.k_p * ex
            vy = self.k_p * ey
            vx, vy = self._clamp_velocity(vx, vy)
            velocities.append((vx, vy))

        return velocities

    def compute_leader_velocity(
        self,
        leader: UAVState,
        waypoints: List[Tuple[int, int]],
        current_wp_index: int,
        grid_resolution: float = 1.0,
    ) -> Tuple[Tuple[float, float], int]:
        """
        Move leader toward the next waypoint along the A* path.

        Args:
            leader:           current leader state
            waypoints:        A* path as list of (grid_x, grid_y)
            current_wp_index: index of the waypoint leader is heading toward
            grid_resolution:  meters per grid cell

        Returns:
            (vx, vy), updated_wp_index
        """
        if current_wp_index >= len(waypoints):
            return (0.0, 0.0), current_wp_index

        wx, wy = waypoints[current_wp_index]
        target_x = wx * grid_resolution
        target_y = wy * grid_resolution

        ex = target_x - leader.x
        ey = target_y - leader.y
        dist = math.sqrt(ex ** 2 + ey ** 2)

        # Advance to next waypoint when close enough
        if dist < 0.3 * grid_resolution:
            current_wp_index += 1
            if current_wp_index >= len(waypoints):
                return (0.0, 0.0), current_wp_index
            wx, wy = waypoints[current_wp_index]
            target_x = wx * grid_resolution
            target_y = wy * grid_resolution
            ex = target_x - leader.x
            ey = target_y - leader.y

        vx = self.k_p * ex
        vy = self.k_p * ey
        vx, vy = self._clamp_velocity(vx, vy)
        return (vx, vy), current_wp_index
