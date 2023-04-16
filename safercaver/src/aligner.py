# Pickhacks 2023
# Safer Caver

# This is inspired by https://github.com/UP-RS-ESP/PointCloudWorkshop-May2022/blob/main/2_Alignment/ICP_Registration_ALS_UAV.ipynb

import copy

from pathlib import Path

import numpy as np
import open3d as o3d
import laspy
import distinctipy as colorpy

from scipy.spatial import cKDTree


def draw_1pcd(pcd):
    o3d.visualization.draw_geometries([pcd])


def draw_point_clouds(*draw_params):
    color_clouds = []

    colors = colorpy.get_colors(len(draw_params), [[1.0, 0.0, 0.0]])
    for draw_param, color in zip(draw_params, colors):
        point_cloud = draw_param[0]
        x_offset = draw_param[1]
        color_option = draw_param[2]

        pc_copy_arr = np.asarray(copy.deepcopy(point_cloud).points)
        pc_copy_arr[:, 0] += np.ones_like(pc_copy_arr[:, 0]) * x_offset

        color_clouds.append(pc_from_np(pc_copy_arr))

        if color_option:
            color_clouds[-1].paint_uniform_color(color)

    o3d.visualization.draw_geometries(color_clouds)


def read_las(path: Path, downsample: int = 10):
    """Read .las and return numpy array"""
    point_cloud = laspy.read(str(path))
    return np.vstack((point_cloud.x, point_cloud.y, point_cloud.z)).transpose()[::downsample]


def pc_from_np(point_arr):
    """Create o3d point cloud from numpy array"""
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(point_arr)

    return point_cloud


def voxelize(point_cloud, voxel_size=1.0, normals=True):
    """Convert point_cloud into voxel with dimension voxel_size (meters)
    Also get normals for later.
    """
    pc_vox = point_cloud.voxel_down_sample(voxel_size=voxel_size)

    if normals:
        pc_vox.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=3, max_nn=30))
        pc_vox.orient_normals_to_align_with_direction([0., 0., 1.])

    return pc_vox


def pc_distance(point_cloud1, point_cloud2):
    """Get modified hausdorf distance between two voxel clouds"""
    pc1_points = np.asarray(point_cloud1.points)
    pc2_points = np.asarray(point_cloud2.points)

    pc1_dist = cKDTree(pc1_points).query(pc2_points, k=1, workers=-1)
    pc2_dist = cKDTree(pc2_points).query(pc1_points, k=1, workers=-1)

    return np.max([np.mean(pc1_dist), np.mean(pc2_dist)])


def get_coarse_transform(point_cloud1, point_cloud2, distance, voxel_size=5.0):
    """Get coarse alignment transformation matrix.
    Down samples (ds) point clouds using large voxel size for speed up.
    point_cloud1 is the stationary pc (ie the map of the cave)
    point_cloud2 is the spelunkers pc (ie where they are lost)
    """
    pc1_ds = voxelize(point_cloud1, voxel_size, False)
    pc2_ds = voxelize(point_cloud2, voxel_size, False)

    # Get Fast Point Feature Histograms (FPFH)
    radius_feature = voxel_size * 5
    pc1_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pc1_ds, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    pc2_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pc2_ds, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    # Use RANSAC to get transform matrix
    distance_threshold = distance * 2
    ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        pc1_ds, pc2_ds, pc1_fpfh, pc2_fpfh, True, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))

    return ransac.transformation


def apply_transformation(point_cloud, transform):
    return copy.deepcopy(point_cloud).transform(transform)


def get_fine_transform(point_cloud1, point_cloud2, distance):
    """Get fine alignment transformation matrix.
    point_cloud1 is the stationary pc (ie the map of the cave)
    point_cloud2 is the spelunkers pc (ie where they are lost)
    """
    pc1 = copy.deepcopy(point_cloud1)

    max_correspondence_distance = distance * 2

    reg_p2p = o3d.pipelines.registration.registration_icp(point_cloud2, pc1, max_correspondence_distance,
                                                          estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint())
    return reg_p2p.transformation


def generate_spe_segment(cave_point_cloud_arr, segment_size, noise=0.1, segment_start=0):
    """Get chunk of cave_point_cloud that spe is stuck in and add some noise
    Noise value is based on Apple device LiDAR precision: https://www.nature.com/articles/s41598-021-01763-9
    """
    if not segment_start:
        lower_limit = cave_point_cloud_arr.shape[0] // 16  # Spe cannot get lost at beginning of cave
        segment_start = np.random.randint(lower_limit, 15 * lower_limit - segment_size)
    spe_point_cloud_arr = cave_point_cloud_arr[segment_start:segment_start + segment_size, :]
    spe_point_cloud_arr_noise = spe_point_cloud_arr + np.random.uniform(-noise, noise, size=spe_point_cloud_arr.shape)

    return spe_point_cloud_arr_noise, segment_start


if __name__ == "__main__":
    LAS_FILE = "Calisto.las"
    SPE_SEGMENT_SIZE = 75_000

    print("Loading LiDAR data...")
    cave_point_cloud_arr = read_las(Path("../res/point_clouds") / LAS_FILE, downsample=10)

    print("Creating point clouds...")
    cave_point_cloud = pc_from_np(cave_point_cloud_arr)
    cave_voxel_cloud = voxelize(cave_point_cloud, voxel_size=0.1)

    # Emulate what a spelunker (spe) would scan by segmenting chunk from data file and adding noise
    # noiseless vars are for draw output later
    spe_point_cloud_noiseless_arr, segment_start = generate_spe_segment(cave_point_cloud_arr, SPE_SEGMENT_SIZE, noise=0)
    spe_voxel_cloud_noiseless = voxelize(pc_from_np(spe_point_cloud_noiseless_arr), voxel_size=0.1)

    spe_point_cloud_arr, _ = generate_spe_segment(cave_point_cloud_arr, SPE_SEGMENT_SIZE, segment_start=segment_start)
    spe_point_cloud = pc_from_np(spe_point_cloud_arr)
    spe_voxel_cloud = voxelize(spe_point_cloud, voxel_size=0.1)

    print("Calculate coarse alignment transform...")
    initial_distance = pc_distance(cave_voxel_cloud, spe_voxel_cloud)
    coarse_transform = get_coarse_transform(cave_voxel_cloud, spe_voxel_cloud, initial_distance, voxel_size=2)

    print("Calculate fine alignment transform...")
    spe_coarse_alignment = apply_transformation(spe_voxel_cloud, coarse_transform)
    coarse_distance = pc_distance(cave_voxel_cloud, spe_coarse_alignment)
    fine_transform = get_fine_transform(cave_voxel_cloud, spe_coarse_alignment, coarse_distance)

    print("Apply transformation...")
    spe_fine_alignment = apply_transformation(spe_coarse_alignment, fine_transform)

    # Draw it pretty
    cave_width = np.max(cave_point_cloud_arr[:, 0]) - np.min(cave_point_cloud_arr[:, 0])
    spe_width = np.max([np.max(spe_point_cloud_arr[:, 0]) - np.min(spe_point_cloud_arr[:, 0]), 30])
    buffer = 10
    draw_point_clouds((cave_voxel_cloud, -(cave_width + spe_width + buffer), False),
                      (spe_voxel_cloud, -(spe_width + buffer), False),
                      (cave_voxel_cloud, 0, False),
                      (spe_voxel_cloud_noiseless, 0, True),
                      (cave_voxel_cloud, cave_width + buffer, False),
                      (spe_fine_alignment, cave_width + buffer, True))
