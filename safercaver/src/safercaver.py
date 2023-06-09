# Pickhacks 2023
# Curtis Brinker and Tanner May
# Python 3.10

"""
SaferCaver: A fast, on-device cave route finder

Steps:
1. Load cave map
2. Simulate lost spelunker scanning surroundings
3. Determine where in the cave the spelunker is
4. Determine the fastest path out of the cave for the spelunker
5. Plot path in 3D space that spelunker should follow
"""

from time import time

import numpy as np
import astar

from point_cloud_loader import load_point_cloud
from aligner import generate_spe_segment, voxelize, pc_from_np, align, draw_point_clouds
from find_path import point_cloud_region, create_find_neighbors, create_node_distance_func, draw_map
from draw_path import path_point_cloud



# Load the Calisto cave scan; it is the coolest looking one!
# Cave scans from: https://figshare.com/articles/dataset/LiDAR_Datasets_for_Hypogenic_and_Epigenic_Caves_in_Bahia_Brazil_2019_/16864147
# This is analogous to the spelunker downloading the map from the QR code posted outside of the cave
cave_point_cloud = load_point_cloud("calisto")
cave_entrance = [(35,15), (45,25)] 


start_time = time()

# Randomly select a segment of the cave to emulate a lost spelunker (spe) in that area >>>
cave_point_cloud_np = np.asarray(cave_point_cloud.points)

spe_point_cloud_arr, _ = generate_spe_segment(cave_point_cloud_np, 75_000)
spe_point_cloud = pc_from_np(spe_point_cloud_arr)
# <<<

# Align the spelunker's scan to the known map
spe_aligned = align(cave_point_cloud, spe_point_cloud)



start_xyz = np.asarray(spe_aligned.points).mean(axis=0)

for attempt in range(10):
    print("Attempt: ", attempt)
    # 10 attempts to find path out, if no path found in those we declare failure
    
    # Down sample the map so that A* runs faster and get numpy data
    cave_point_cloud_down_sampled = cave_point_cloud.random_down_sample(0.02)
    data = np.asarray(cave_point_cloud_down_sampled.points)

    # Find the closest node in the down sampled map
    start_location = np.linalg.norm(data - start_xyz, axis=1).argmin()

    # Select a point to reach, take a mean of points near the entrance
    goal_region_idx = point_cloud_region(
        cave_point_cloud_np,
        *cave_entrance
    )
    goal_xyz = cave_point_cloud_np[goal_region_idx].mean(axis=0)
    goal_location = np.linalg.norm(data - goal_xyz, axis=1).argmin()

    # Define neighbors as the 5 nearest down sampled points
    get_neighbors = create_find_neighbors(
        data,
        k=5
    )

    # Create measure of distance from nodes
    # (maps node idx to physical distance)
    node_distance = create_node_distance_func(data)
    
    try:
        escape_route = astar.find_path(
            start=start_location,
            goal=goal_location,
            neighbors_fnct=get_neighbors,
            heuristic_cost_estimate_fnct=node_distance,
            distance_between_fnct=node_distance
        )
        break
    except IndexError as e:
        # I think the library I am using is a little buggy
        # it works most of the time though...
        continue
    
escape_route = list(escape_route)

escape_route_xyz = data[escape_route]

escape_route_path = path_point_cloud(escape_route_xyz)

# Draw point clouds
buffer_size = 10
cave_width = np.max(cave_point_cloud_np[:, 0]) - np.min(cave_point_cloud_np[:, 0])
spe_width = np.max([np.max(spe_point_cloud_arr[:, 0]) - np.min(spe_point_cloud_arr[:, 0]), 30])

cave_voxel_cloud = voxelize(cave_point_cloud, voxel_size=0.1)
spe_voxel_cloud = voxelize(spe_point_cloud, voxel_size=0.1)

delta_t = time() - start_time
print(f"Solution found! (took {delta_t:>6.6f} seconds.")


draw_point_clouds(
    (cave_voxel_cloud, -(cave_width + spe_width + buffer_size), False),
    (spe_voxel_cloud, -(spe_width + buffer_size), False),
    (cave_voxel_cloud, 0, False),
    (spe_voxel_cloud, 0, True),
    (cave_voxel_cloud, cave_width + buffer_size, False),
    (spe_aligned, cave_width + buffer_size, True),
    (cave_voxel_cloud, 2 * cave_width + buffer_size, False),
    (escape_route_path, 2 * cave_width + buffer_size, True)
)


draw_map(
    cave_point_cloud_np,
    escape_route_xyz,
    start_xyz,
    goal_xyz
)




