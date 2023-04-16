#!/usr/bin/python

import numpy as np

import sys

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Expected point cloud file as argument")
        exit(1)

    import open3d as o3d
    import matplotlib.pyplot as plt
    pcd = o3d.io.read_point_cloud(sys.argv[1])
    xy = np.asarray(pcd.points)[:, :2]
    plt.scatter(xy[:,0], xy[:, 1], s=2)
    plt.show()
