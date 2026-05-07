"""
Global path planner using the `pathfinding` library (A* on a 2-D grid).
"""

from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from typing import List, Tuple, Optional


class GlobalPlanner:
    """
    Thin wrapper around pathfinding.AStarFinder.

    Grid convention:  matrix[row][col]
        - 1  : free cell (pathfinding library uses 1 for walkable)
        - 0  : obstacle
    """

    def __init__(self, matrix: List[List[int]]):
        """
        Args:
            matrix: 2-D list where 1 = free, 0 = obstacle.
                     Rows = Y axis, columns = X axis.
        """
        self._matrix = matrix
        self._finder = AStarFinder()

    def plan(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> Optional[List[Tuple[int, int]]]:
        """
        Find shortest path on the grid.

        Args:
            start: (col, row) i.e. (x, y) in grid coordinates
            goal:  (col, row) i.e. (x, y) in grid coordinates

        Returns:
            List of (x, y) waypoints, or None if no path exists.
        """
        grid = Grid(matrix=self._matrix)
        sx, sy = start
        gx, gy = goal

        try:
            start_node = grid.node(sx, sy)
            goal_node  = grid.node(gx, gy)
        except IndexError:
            return None

        path, _ = self._finder.find_path(start_node, goal_node, grid)

        if not path:
            return None

        return [(node.x, node.y) for node in path]

    def update_obstacle(self, x: int, y: int, walkable: bool) -> None:
        """
        Dynamically mark a cell as obstacle (False) or free (True).
        """
        self._matrix[y][x] = 1 if walkable else 0
