"""
Multi-agent collision avoidance using the `pyrvo` library (ORCA / RVO2).
"""

from pyrvo import RVOSimulator
from typing import List, Tuple


class CollisionAvoidance:
    """
    Wrapper around pyrvo.RVOSimulator for n-agent ORCA collision avoidance.

    Usage:
        ca = CollisionAvoidance(time_step=0.1)
        ids = [ca.add_agent(x, y) for x, y in start_positions]
        ...
        ca.set_preferred_velocity(agent_id, vx, vy)
        ca.step()
        x, y = ca.get_position(agent_id)
        vx, vy = ca.get_velocity(agent_id)
    """

    def __init__(
        self,
        time_step: float = 0.1,
        neighbor_dist: float = 5.0,
        max_neighbors: int = 10,
        time_horizon: float = 2.0,
        time_horizon_obst: float = 2.0,
        radius: float = 0.5,
        max_speed: float = 2.0,
    ):
        """
        Args:
            time_step:          simulation dt (seconds)
            neighbor_dist:      radius to search for neighbours (m)
            max_neighbors:      max neighbours considered per agent
            time_horizon:       lookahead time for agent-agent avoidance (s)
            time_horizon_obst:  lookahead time for agent-obstacle avoidance (s)
            radius:             collision radius per agent (m)
            max_speed:          maximum speed per agent (m/s)
        """
        self._sim = RVOSimulator(
            time_step,
            neighbor_dist,
            max_neighbors,
            time_horizon,
            time_horizon_obst,
            radius,
            max_speed,
        )
        self._radius = radius
        self._max_speed = max_speed

    # ------------------------------------------------------------------
    # Agent management
    # ------------------------------------------------------------------

    def add_agent(
        self,
        x: float,
        y: float,
        radius: float | None = None,
        max_speed: float | None = None,
    ) -> int:
        """
        Add an agent at position (x, y).

        Returns:
            agent id (int) assigned by the simulator.
        """
        pos = (x, y)
        if radius is None and max_speed is None:
            return self._sim.add_agent(pos)
        return self._sim.add_agent(
            pos,
            radius or self._radius,
            10,                        # max_neighbors
            5.0,                       # neighbor_dist
            2.0,                       # time_horizon
            2.0,                       # time_horizon_obst
            max_speed or self._max_speed,
        )

    def add_obstacle(self, vertices: List[Tuple[float, float]]) -> int:
        """
        Add a static polygon obstacle.

        Args:
            vertices: list of (x, y) corner points (counter-clockwise).

        Returns:
            obstacle id.
        """
        verts = [(x, y) for x, y in vertices]
        obs_id = self._sim.add_obstacle(verts)
        self._sim.process_obstacles()
        return obs_id

    # ------------------------------------------------------------------
    # Per-step control
    # ------------------------------------------------------------------

    def set_preferred_velocity(
        self, agent_id: int, vx: float, vy: float
    ) -> None:
        """Set the preferred (desired) velocity for an agent."""
        self._sim.set_agent_pref_velocity(agent_id, (vx, vy))

    def step(self) -> None:
        """Advance the simulation by one time step."""
        self._sim.do_step()

    # ------------------------------------------------------------------
    # State queries
    # ------------------------------------------------------------------

    def get_position(self, agent_id: int) -> Tuple[float, float]:
        p = self._sim.get_agent_position(agent_id)
        return (p.x, p.y)

    def get_velocity(self, agent_id: int) -> Tuple[float, float]:
        v = self._sim.get_agent_velocity(agent_id)
        return (v.x, v.y)

    def num_agents(self) -> int:
        return self._sim.get_num_agents()
