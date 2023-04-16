import numpy as np
import open3d as o3d

from point_cloud_loader import load_point_cloud
from scipy.spatial import cKDTree

import matplotlib.pyplot as plt
import astar


def create_neighbors_func(point_cloud, radius):
    np_cloud = np.asarray(point_cloud.points)
    def neighbors(point):
        x, y, *_ = point
        xy_min = (x-radius, y-radius)
        xy_max = (x+radius, y+radius)
        print(f"{xy_min = }, {xy_max = }")
        return point_cloud_region(
            np_cloud,
            xy_min,
            xy_max,
        )
    return neighbors

def point_cloud_region(point_cloud, xy_min, xy_max):
    upper = np.alltrue(point_cloud < [*xy_max, 1e6], axis=1)
    lower = np.alltrue(point_cloud > [*xy_min, -1e6], axis=1)
    return (upper & lower).nonzero()[0]


def create_find_neighbors(point_cloud, k=5):
    def get_neighbors(node):
        distances = np.linalg.norm(
                point_cloud[node] - point_cloud,
                axis=1
        )
        neighbors = np.argpartition(distances, k+1)[:k+1]
        neighbors = neighbors[neighbors != node]
        return neighbors
    return get_neighbors

def create_node_distance_func(point_cloud):
    def distance(a, b):
        return np.linalg.norm(
            point_cloud[a] - point_cloud[b]
        )
    return distance

def create_heuristic(point_cloud, goal):
    def heuristic(node):
        return np.linalg.norm(
            point_cloud[node] - point_cloud[goal]
        )
    return heuristic

def draw_map(point_cloud, path, start, goal):
    plt.plot(point_cloud[:, 0], point_cloud[:, 1], s=2)
    plt.plot(path[:, 0], path[:, 1], s=10)
    plt.plot(start[0], start[1], s=50)
    plt.plot(goal[0], goal[1], s=50)
    plt.show()


if __name__ == '__main__':
    point_cloud = load_point_cloud('calisto')
    print(point_cloud)
    point_cloud = point_cloud.random_down_sample(0.02)
    print(point_cloud)
    
    entrance = [
        (35,15), (45, 25)
    ]

    data = np.asarray(point_cloud.points)

    # get_neighbors = create_neighbors_func(
    get_neighbors = create_find_neighbors(
        data,
        k=5,
    )

    home = np.random.choice(
        point_cloud_region(data, *entrance)
    )
    region = point_cloud_region(data, *entrance)
    print(f"{region.shape = }")
    np_cloud = np.asarray(point_cloud.points)

    random_point = np.random.randint(data.shape[0])
    print("Random point: ", random_point)
    neighbors = get_neighbors(random_point)

    print(f"{neighbors.shape = }")

    start = random_point
    goal = np.random.choice(region)
    heuristic = create_heuristic(data, goal)
    node_distance = create_node_distance_func(data)

    print(start, goal)
    print("Solving...")
    from time import time

    start_t = time()
    result = astar.find_path(
        start=start,
        goal=goal,
        neighbors_fnct=get_neighbors,
        heuristic_cost_estimate_fnct=node_distance,
        distance_between_fnct=node_distance
    )
    delta = time() - start_t
    print(f"Solution found ({delta:>6.6f})")
    result = list(result)


    plt.scatter(data[:,0], data[:,1], s=2)
    plt.scatter(data[result,0], data[result,1], s=15)
    plt.scatter(data[neighbors,0], data[neighbors,1], s=15)
    plt.scatter(data[region,0], data[region,1], s=25)
    plt.show()


