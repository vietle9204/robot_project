#!/usr/bin/env python3
import yaml
import numpy as np
from PIL import Image
import heapq
import os
from pathlib import Path

script_dir = Path(__file__).resolve().parent
src_dir = script_dir.parent.parent 
yaml_path = src_dir / 'maps' / 'my_map.yaml'
yaml_path = str(yaml_path)
print(f"[Method 2] Using relative path: {yaml_path}")

# Verify file exists
if not os.path.exists(yaml_path):
    raise FileNotFoundError(f"Map file not found at: {yaml_path}\nPlease check your workspace structure.")
print(f"Loading map from: {yaml_path}\n")

with open(yaml_path, 'r') as file:
    map_data = yaml.safe_load(file)

pgm_path = map_data['image']
resolution = map_data['resolution']
origin = map_data['origin']
occupied_thresh = map_data.get('occupied_thresh')
free_thresh = map_data.get('free_thresh')

img = Image.open(pgm_path)
img = np.array(img, dtype=np.float32) / 255.0 # Normalize to [0, 1]

rows, cols = img.shape

# Build occupancy grid
grid = np.zeros((rows, cols), dtype=np.int8)

grid[img >= occupied_thresh] = 0  # Occupied
grid[img <= free_thresh] = 1      # Free
grid[(img > free_thresh) & (img < occupied_thresh)] = -1  # Unknown

grid = np.flipud(grid)  # Flip vertically to match coordinate system

# Coordinate transformation functions
def world_to_grid(x, y):
    gx = int((x - origin[0]) / resolution)
    gy = int((y - origin[1]) / resolution)
    gy = rows - gy - 1  # Flip y-axis
    return gx, gy

def grid_to_world(gx, gy):
    x = gx * resolution + origin[0] + resolution / 2
    y = (rows - gy - 1) * resolution + origin[1] + resolution / 2
    return x, y

# A* Algorithm Implementation
def a_star(grid, start, goal):
    sx, sy = start
    gx, gy = goal

    if grid[sy, sx] == 1:
        print("Start is in obstacle!")
        return None
    if grid[gy, gx] == 1:
        print("Goal is in obstacle!")
        return None

    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (0, (sx, sy)))

    came_from = {}
    g_score = { (sx, sy): 0 }

    # 8-connected grid
    neighbors8 = [
        (1, 0), (-1, 0), (0, 1), (0, -1),
        (1, 1), (1, -1), (-1, 1), (-1, -1),
    ]

    def heuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == (gx, gy):
            # reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for dx, dy in neighbors8:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < cols and 0 <= ny < rows:
                if grid[ny, nx] == 1:
                    continue  # skip obstacles

                new_cost = g_score[current] + np.hypot(dx, dy)
                if (nx, ny) not in g_score or new_cost < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = new_cost
                    came_from[(nx, ny)] = current
                    f = new_cost + heuristic((nx, ny), (gx, gy))
                    heapq.heappush(open_set, (f, (nx, ny)))

    return None

# Test with example start and goal
start_world = (0.0, 0.0)  # Example start in world
goal_world = (10.32, 9.87)   # Example goal in world

start_grid = world_to_grid(*start_world)
goal_grid = world_to_grid(*goal_world)

print(f"Start grid: {start_grid}, Goal grid: {goal_grid}")
path = a_star(grid, start_grid, goal_grid)
if path:
    path_world = [grid_to_world(x, y) for x, y in path]
    print("Path found:")
    for p in path_world:
        print(f"{p}")
else:
    print("No path found.")
    