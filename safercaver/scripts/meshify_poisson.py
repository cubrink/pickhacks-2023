#!/usr/bin/python

from pathlib import Path
import numpy as np
import laspy as lp

import open3d as o3d

if __name__ == '__main__':
    pc_paths = list(Path('pointclouds').glob('*.las'))
    
    # for idx, path in enumerate(pc_paths):
    #     print(f"{idx}: {path.name}")

    # pc_idx = int(input(f"Give index (0-{len(pc_paths)-1}): "))
    pc_idx = 5
    print("Using pc: ", pc_paths[pc_idx])
    #input_path='pointclouds/MorroVermelho.las'
    pc = lp.read(pc_paths[pc_idx])

    xyz = np.vstack((pc.x, pc.y, pc.z)).T
    # print(xyz)
    print(xyz.shape)

    # downscale
    xyz = xyz[::10]
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    # print('pcd:', pcd)

    # o3d.visualization.draw_geometries([pcd])

    
    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=12)
        # o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=6)
    )
    pcd.orient_normals_to_align_with_direction([0., 0., 1.])
    # o3d.visualization.draw_geometries([pcd])

    pcd = pcd.voxel_down_sample(voxel_size=0.1)
    print('pcd (after voxel): ', pcd)

    
    # o3d.visualization.draw_geometries([pcd_1m])
    


    # Create mesh
    bbox = pcd.get_axis_aligned_bounding_box()
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd,
        depth=10,
        linear_fit=True,
        scale=0.8
    )[0].crop(bbox)

    #mesh.remove_degenerate_triangles()
    #mesh.remove_duplicated_triangles()
    #mesh.remove_duplicated_vertices()
    #mesh.remove_non_manifold_edges()

    mesh.compute_triangle_normals()
    mesh.compute_vertex_normals()
    o3d.visualization.draw_geometries([mesh])


    # Save mesh
    o3d.io.write_triangle_mesh("mesh.obj", mesh)

    

