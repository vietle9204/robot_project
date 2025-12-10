#!/usr/bin/env python3
import yaml
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
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
negate = map_data.get('negate', 0)

img = Image.open(pgm_path)
img = np.array(img, dtype=np.float32) / 255.0

rows, cols = img.shape

# Build occupancy grid
grid = np.zeros((rows, cols), dtype=np.int8)

if negate == 0:
    # Standard: white=free, black=occupied
    grid[img <= free_thresh] = 1      # Dark = Occupied
    grid[img >= occupied_thresh] = 0  # Bright = Free
    grid[(img > free_thresh) & (img < occupied_thresh)] = -1  # Unknown
else:
    # Inverted
    grid[img >= occupied_thresh] = 1  # Occupied
    grid[img <= free_thresh] = 0      # Free
    grid[(img > free_thresh) & (img < occupied_thresh)] = -1  # Unknown

grid = np.flipud(grid)

# Visualize the map
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))

# Plot 1: Grid coordinates
ax1.imshow(grid, cmap='gray_r', origin='upper')
ax1.set_title('Occupancy Grid (Grid Coordinates)\nWhite=Free, Black=Occupied, Gray=Unknown')
ax1.set_xlabel('Grid X')
ax1.set_ylabel('Grid Y')
ax1.grid(True, alpha=0.3)

# Plot 2: World coordinates
extent = [origin[0], origin[0] + cols * resolution, 
          origin[1], origin[1] + rows * resolution]
ax2.imshow(np.flipud(grid), cmap='gray_r', extent=extent, origin='lower')
ax2.set_title('Occupancy Grid (World Coordinates)')
ax2.set_xlabel('World X (m)')
ax2.set_ylabel('World Y (m)')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
print("Map visualization displayed")
plt.show()