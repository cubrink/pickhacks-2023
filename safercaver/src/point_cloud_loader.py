import open3d as o3d
import numpy as np
import laspy as lp

from pathlib import Path

res_location = Path(__file__).parent.parent / 'res'

def load_point_cloud(name):
    if Path(name).exists():
        path = name
    else:
        point_cloud_location = res_location / 'point_clouds'
        point_cloud_files = point_cloud_location.iterdir()
        point_cloud_files = filter(
                lambda f: f.suffix != '.las', 
                point_cloud_files
        )
        point_clouds = {
            path.stem.lower(): path
            for path in point_cloud_files
        }
        if name.lower() in point_clouds:
            path = point_clouds[name]
        else:
            raise FileNotFoundError("No point cloud found for: ", name)

    point_cloud = o3d.io.read_point_cloud(path.as_posix())
    return point_cloud


def load_las(name):
    if Path(name).exists():
        path = name
    else:
        point_cloud_location = res_location / 'point_clouds'
        point_cloud_files = point_cloud_locations.glob('*.las')
        point_clouds = {
            p.stem.lower(): p
            for p in point_cloud_files
        }
        if name.lower() in point_clouds:
            path = point_clous[name]
        else:
            raise FileNotFoundError("No .las point cloud found for: ", name)

    pc = lp.read(path)
    xyz = np.vstack((pc.x, pc.y, pc.z)).T
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(xyz)

    return point_cloud


