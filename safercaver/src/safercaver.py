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
from aligner import generate_spe_segment, voxelize, pc_from_np, align
from find_path import point_cloud_region, create_find_neighbors, create_node_distance_func



# Load the Calisto cave scan; it is the coolest looking one!
# Cave scans from: https://figshare.com/articles/dataset/LiDAR_Datasets_for_Hypogenic_and_Epigenic_Caves_in_Bahia_Brazil_2019_/16864147
# This is analogous to the spelunker downloading the map from the QR code posted outside of the cave
cave_point_cloud = load_point_cloud("calisto")
cave_entrance = [(35,15), (45,25)] 

# Randomly select a segment of the cave to emulate a lost spelunker (spe) in that area >>>
cave_point_cloud_np = np.asarray(cave_point_cloud.points)

# These two are just for visualization later
spe_point_cloud_noiseless_np, segment_start = generate_spe_segment(cave_point_cloud_np, 75_000, noise=0)
spe_voxel_cloud_noiseless = voxelize(pc_from_np(spe_point_cloud_noiseless_np), voxel_size=0.1)

spe_point_cloud_arr, _ = generate_spe_segment(cave_point_cloud_np, 75_000, segment_start=segment_start)
spe_point_cloud = pc_from_np(spe_point_cloud_arr)
# <<<

# Align the spelunker's scan to the known map
spe_aligned = align(cave_point_cloud, spe_point_cloud)



start_xyz = np.asarray(spe_aligned.points).mean(axis=0)
start_time = time()

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
        print(e)
        continue
    
delta_t = time() - start_time
escape_route = list(escape_route)

escape_route_xyz = data[escape_route] 







