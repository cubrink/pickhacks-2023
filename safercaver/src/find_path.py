import numpy as np
import open3d as o3d

from point_cloud_loader import load_point_cloud

if __name__ == '__main__':
    point_cloud = load_point_cloud('calisto')
    print(point_cloud)
