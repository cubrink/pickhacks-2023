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

import numpy as np

from point_cloud_loader import load_point_cloud
from aligner import generate_spe_segment, voxelize, pc_from_np, align


# Load the Calisto cave scan; it is the coolest looking one!
# Cave scans from: https://figshare.com/articles/dataset/LiDAR_Datasets_for_Hypogenic_and_Epigenic_Caves_in_Bahia_Brazil_2019_/16864147
# This is analogous to the spelunker downloading the map from the QR code posted outside of the cave
cave_point_cloud = load_point_cloud("calisto")

# Randomly select a segment of the cave to emulate a lost spelunker (spe) in that area >>>
cave_point_cloud_np = np.asarray(cave_point_cloud.points)

# These two are just for visualization later
spe_point_cloud_noiseless_np, segment_start = generate_spe_segment(cave_point_cloud_np, 75_000, noise=0)
spe_voxel_cloud_noiseless = voxelize(spe_point_cloud_noiseless_np, voxel_size=0.1)

spe_point_cloud_arr, _ = generate_spe_segment(cave_point_cloud_np, 75_000, segment_start=segment_start)
spe_point_cloud = pc_from_np(spe_point_cloud_arr)
# <<<

# Align the spelunker's scan to the known map
spe_aligned = align(cave_point_cloud, spe_point_cloud)







