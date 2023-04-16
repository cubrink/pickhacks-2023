# Pickhacks 2023
# Safer Caver

# This is heavily inspired by https://github.com/UP-RS-ESP/PointCloudWorkshop-May2022/blob/main/2_Alignment/ICP_Registration_ALS_UAV.ipynb

import copy

from pathlib import Path

import numpy as np
import matplotlib.pyplot as pl
import open3d as o3d
import laspy

from scipy.spatial import cKDTree


def draw_1pcd(pcd):
    o3d.visualization.draw_geometries([pcd])


# Generate function to plot two point clouds in different colors
def draw_2pcd(pcd1, pcd2):
    pcd1_temp = copy.deepcopy(pcd1)
    pcd2_temp = copy.deepcopy(pcd2)
    pcd1_temp.paint_uniform_color([1, 0.706, 0])  # orange
    pcd2_temp.paint_uniform_color([0, 0.651, 0.929])  # cyan
    o3d.visualization.draw_geometries([pcd1_temp, pcd2_temp])


def draw_registration_result(source, target):#, transformation):
    """source is spe, target is cave"""
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])  # orange
    target_temp.paint_uniform_color([0, 0.651, 0.929])  # cyan
    #source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def get_colors(inp, colormap, vmin=None, vmax=None):
    if vmin == None:
        vmin = np.nanmin(inp)
    if vmax == None:
        vmax = np.nanmax(inp)
    norm = pl.Normalize(vmin, vmax)
    return colormap(norm(inp))


def get_colors_log(inp, colormap, vmin=None, vmax=None):
    if vmin == None:
        vmin = np.nanmin(np.log10(inp))
    if vmax == None:
        vmax = np.nanmax(np.log10(inp))
    norm = pl.Normalize(vmin, vmax)
    return colormap(norm(inp))


def read_las(path: Path, downsample: int = 10):
    """Read .las and return numpy array"""
    point_cloud = laspy.read(str(path))
    return np.vstack((point_cloud.x, point_cloud.y, point_cloud.z)).transpose()[::downsample]


def pc_from_np(point_arr):
    """Create o3d point cloud from numpy array"""
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(point_arr)

    return point_cloud


def voxelize(point_cloud, voxel_size=1, normals=True):
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


def get_coarse_transform(point_cloud1, point_cloud2, distance, voxel_size=5):
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


def generate_spe_segment(cave_point_cloud_arr, segment_size, noise=0.1):
    """Get chunk of cave_point_cloud that spe is stuck in and add some noise
    Noise value is based on Apple device LiDAR precision: https://www.nature.com/articles/s41598-021-01763-9
    """
    lower_limit = cave_point_cloud_arr.shape[0] // 16  # Spe cannot get lost at beginning of cave
    segment_start = np.random.randint(lower_limit, 15 * lower_limit - segment_size)
    spe_point_cloud_arr = cave_point_cloud_arr[segment_start:segment_start + segment_size, :]
    spe_point_cloud_arr_noise = spe_point_cloud_arr + np.random.uniform(-noise, noise, size=spe_point_cloud_arr.shape)

    return spe_point_cloud_arr_noise


if __name__ == "__main__":
    LAS_FILE = "Calisto.las"
    SPE_SEGMENT_SIZE = 75_000

    cave_point_cloud_arr = read_las(Path("../res/point_clouds") / LAS_FILE, downsample=10)

    cave_point_cloud = pc_from_np(cave_point_cloud_arr)
    cave_voxel_cloud = voxelize(cave_point_cloud, voxel_size=1)

    draw_1pcd(cave_point_cloud)
    draw_1pcd(cave_voxel_cloud)

    # Emulate what a spelunker (spe) would scan by segmenting chunk from data file and adding noise
    spe_point_cloud_arr = generate_spe_segment(cave_point_cloud_arr, SPE_SEGMENT_SIZE)
    spe_point_cloud = pc_from_np(spe_point_cloud_arr)
    spe_voxel_cloud = voxelize(spe_point_cloud, voxel_size=1)

    draw_1pcd(spe_point_cloud)
    draw_1pcd(spe_voxel_cloud)

    initial_distance = pc_distance(cave_voxel_cloud, spe_voxel_cloud)
    coarse_transform = get_coarse_transform(cave_voxel_cloud, spe_voxel_cloud, initial_distance)

    spe_coarse_alignment = apply_transformation(spe_voxel_cloud, coarse_transform)
    coarse_distance = pc_distance(cave_voxel_cloud, spe_coarse_alignment)
    fine_transform = get_fine_transform(cave_voxel_cloud, spe_coarse_alignment, coarse_distance)

    spe_fine_alignment = apply_transformation(spe_coarse_alignment, fine_transform)
    draw_registration_result(spe_fine_alignment, cave_voxel_cloud)
