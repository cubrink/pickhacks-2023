#!/usr/bin/python

from pathlib import Path
import numpy as np
import laspy as lp

import open3d as o3d
import sys
if __name__ == '__main__':
    for path in map(Path, sys.argv[1:]):
        if path.suffix != '.las':
            print("Expected .las file, recieved ", path.name)
            continue
        else:
            print(path.name)
        pc = lp.read(path)
        xyz = np.vstack((pc.x, pc.y, pc.z)).T

        # downscale
        xyz = xyz[::10]
    
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.estimate_normals()
        pcd.orient_normals_to_align_with_direction([0., 0., 1.])
        pcd = pcd.voxel_down_sample(voxel_size=0.1)
        o3d.io.write_point_cloud(path.with_suffix('.xyzn').name, pcd)

