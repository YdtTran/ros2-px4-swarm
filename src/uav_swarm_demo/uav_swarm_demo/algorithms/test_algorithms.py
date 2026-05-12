"""
Integration test for GlobalPlanner (pathfinding) + CollisionAvoidance (pyrvo).
"""

import math

try:
    from uav_swarm_demo.algorithms.planner import GlobalPlanner
    from uav_swarm_demo.algorithms.collision_avoidance import CollisionAvoidance
except ImportError:
    # Fallback: allow running as `python3 test_algorithms.py` from algorithms/
    from planner import GlobalPlanner
    from collision_avoidance import CollisionAvoidance


def dist(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


# ─── Test 1: A* GlobalPlanner ─────────────────────────────────────────────────
print("=" * 55)
print("Test 1: GlobalPlanner (A* via pathfinding library)")
print("=" * 55)

# matrix: 1=free, 0=obstacle  (row=Y, col=X)
matrix = [
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],   # wall, gap at x=5
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
]

planner = GlobalPlanner(matrix)
path = planner.plan(start=(0, 0), goal=(9, 9))

if path:
    print(f"✅ Path found: {len(path)} waypoints")
    print(f"   Start: {path[0]}  →  Goal: {path[-1]}")
else:
    print("❌ No path found")

# Blocked scenario
matrix2 = [[0] * 5 for _ in range(5)]
matrix2[0][0] = 1
matrix2[4][4] = 1
planner2 = GlobalPlanner(matrix2)
path2 = planner2.plan(start=(0, 0), goal=(4, 4))
print(f"✅ Blocked scenario: {'None (correct)' if path2 is None else 'ERROR'}")


# ─── Test 2: CollisionAvoidance (ORCA via pyrvo) ──────────────────────────────
print()
print("=" * 55)
print("Test 2: CollisionAvoidance (ORCA via pyrvo)")
print("=" * 55)

ca = CollisionAvoidance(time_step=0.1, radius=0.5, max_speed=2.0)

# 3 UAVs flying head-on toward origin from 3 directions
id0 = ca.add_agent( 0.0,  5.0)   # top    → prefers moving down
id1 = ca.add_agent(-5.0,  0.0)   # left   → prefers moving right
id2 = ca.add_agent( 5.0,  0.0)   # right  → prefers moving left

pref_vels = {
    id0: ( 0.0, -2.0),
    id1: ( 2.0,  0.0),
    id2: (-2.0,  0.0),
}

combined_r = 1.0   # 2 × radius
min_dist = float('inf')
collisions = 0

print(f"{'Step':>4}  {'UAV0':>14}  {'UAV1':>14}  {'UAV2':>14}  {'min_d':>6}")
print("-" * 65)

for step in range(80):
    for aid, (pvx, pvy) in pref_vels.items():
        ca.set_preferred_velocity(aid, pvx, pvy)
    ca.step()

    p0 = ca.get_position(id0)
    p1 = ca.get_position(id1)
    p2 = ca.get_position(id2)

    d01 = dist(p0, p1)
    d02 = dist(p0, p2)
    d12 = dist(p1, p2)
    step_min = min(d01, d02, d12)
    min_dist = min(min_dist, step_min)

    if d01 < combined_r or d02 < combined_r or d12 < combined_r:
        collisions += 1

    if step % 10 == 0:
        print(
            f"{step:>4}  "
            f"({p0[0]:5.2f},{p0[1]:5.2f})  "
            f"({p1[0]:5.2f},{p1[1]:5.2f})  "
            f"({p2[0]:5.2f},{p2[1]:5.2f})  "
            f"{step_min:6.3f}"
        )

print()
print(f"Min distance across all steps: {min_dist:.3f}m  (combined_r={combined_r}m)")
if collisions == 0:
    print("✅ No collisions!")
else:
    print(f"❌ {collisions} collision steps detected")
