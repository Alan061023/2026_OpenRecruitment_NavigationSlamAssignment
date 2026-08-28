## The Algorithm

`generate_path()` is a breadth-first search over the occupancy grid, with two changes from the 4-direction starter code:

1. **8-direction movement** — the search also expands diagonally, not just up/down/left/right, so it can cut across open space instead of moving in a staircase pattern.
2. **Corner-cut check** — a diagonal move is only allowed if at least one of the two orthogonal cells next to it is free. If both are occupied, the diagonal move is skipped even though the diagonal cell itself is technically open. This stops the path from "squeezing" through a corner gap that only exists on the grid, not in real life (the robot has width and can't actually fit through it).

```python
"""Freshie-editable path planner for the Python track."""
from collections import deque

from geometry_msgs.msg import PoseStamped

from planner_py.occupancy_grid_view import OccupancyGridView


def generate_path(
    start_pose: PoseStamped,
    goal_pose: PoseStamped,
    grid: OccupancyGridView,
) -> list:
    """BFS (8-direction, corner-safe) from start_pose to goal_pose; ordered (x, y) waypoints, empty if unreachable."""
    start = grid.world_to_grid(start_pose.pose.position.x, start_pose.pose.position.y)
    goal = grid.world_to_grid(goal_pose.pose.position.x, goal_pose.pose.position.y)

    if grid.is_occupied(*start) or grid.is_occupied(*goal):
        return []

    visited = {start: None}   # cell -> parent cell
    queue = deque([start])
    neighbors = [
        (1, 0), (-1, 0), (0, 1), (0, -1),      # up / down / left / right
        (1, 1), (1, -1), (-1, 1), (-1, -1),    # four diagonals
    ]

    while queue:
        cur = queue.popleft()
        if cur == goal:
            break
        for dx, dy in neighbors:
            nxt = (cur[0] + dx, cur[1] + dy)
            if nxt in visited or grid.is_occupied(*nxt):
                continue
            if dx != 0 and dy != 0:
                # diagonal move: forbid cutting through a corner pinch-point
                if grid.is_occupied(cur[0] + dx, cur[1]) and grid.is_occupied(cur[0], cur[1] + dy):
                    continue
            visited[nxt] = cur
            queue.append(nxt)

    if goal not in visited:
        return []

    path_cells = []
    node = goal
    while node is not None:
        path_cells.append(node)
        node = visited[node]
    path_cells.reverse()

    return [grid.grid_to_world(gx, gy) for gx, gy in path_cells]
```

**Why this:** the 4-direction starter code only moves in straight lines, so any diagonal route gets approximated as a staircase — the path ends up longer than it needs to be, and 40% of the benchmark score is based on how close your path length is to the straight-line distance. 8-direction movement fixes that. But 8-direction on its own is dangerous — it'll happily cut diagonally between two obstacles that are only touching at a corner, which a real robot with actual width can't do. The corner-cut check closes that gap.

---

## My Development Journey

I started on a VM because my machine runs a newer ROS2 version (Jazzy) than this assignment needs (Humble), so I used Docker inside the VM to get an isolated Ubuntu 22.04 + Humble environment. Just getting this environment installed and working took a lot longer than I expected — most of my early time went into environment setup, not the actual planner. I edited the code through VS Code (attached to the container via an extension), which was the main workflow for everything after that.

The first real change I made was just adding 4 more directions (8-direction movement instead of 4). That alone got me a 90+ score — but only once. Running the exact same code again after that, it kept getting stuck at the `open_corner` goal.

So I started debugging. I added the corner-cut checking function, and then a separate function to keep a safety margin/distance away from obstacles. Still stuck — couldn't get past `open_corner`. I tried setting that safety margin all the way down to 0 and all the way up to 500, and the result was exactly the same both times (still stuck). At that point it became clear this wasn't something that function could fix.

Next I tried swapping out the algorithm itself — instead of BFS, I implemented A*. Still stuck, at the same spot, in the same way. Out of about 100 test runs, only around 4 succeeded.

That's when I started suspecting the problem wasn't in my code at all. I asked AI about it, and it suggested the issue might be in the benchmark file itself, or in one of the other files I'm not allowed to touch under the assignment rules.

The turning point was testing the exact same code on my friend's computer — it succeeded every single time, reliably, though the score itself wasn't very high. That's when I realized it's probably a machine-level issue — possibly Docker, possibly the VM itself — rather than my code. My final conclusion is that it's most likely not a code problem.

Given all that, what I'm submitting is just the 8-direction + corner-cut version, along with a recording of one of the successful runs. On `world2_house` (the second map), the same code gets completely stuck and doesn't move at all. Due to time constraints, this is the version I'm submitting, along with some of the other code versions I tried along the way (attached separately).
