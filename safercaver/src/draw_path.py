import numpy as np

from point_cloud_loader import load_point_cloud
from aligner import draw_point_clouds, pc_from_np


def multilinspace(values: np.array, n: int):
    points = np.linspace(values[:-1], values[1:], n).T[:, :-1].reshape(1, -1).squeeze()
    return np.append(points, values[-1])


def linear_point_cloud(start, end, resolution=30_000):
    start_end = np.vstack([start, end])
    pc_arr = np.vstack([multilinspace(start_end[:, i], resolution) for i in range(3)]).T

    return pc_from_np(pc_arr)


if __name__ == "__main__":
    point_cloud = load_point_cloud("calisto")
    point_cloud_arr = np.asarray(point_cloud.points)

    starti, endi = np.random.randint(len(point_cloud_arr), size=2)
    start = point_cloud_arr[starti]
    end = point_cloud_arr[endi]

    linear_pc = linear_point_cloud(start, end, resolution=10_000)

    draw_point_clouds(
        (point_cloud, 0, False),
        (linear_pc, 0, True)
    )
