#!/usr/bin/python

import open3d as o3d
import sys

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Expected usage: view_xyz.py <point_cloud.xyz>")
        exit(1)

    for path in sys.argv[1:]:
        print(path)
        pcd = o3d.io.read_point_cloud(path)
        o3d.visualization.draw_geometries([pcd])
