import math
import numpy as np
from heapq import heappush, heappop
import numpy as np
from .occupancy_map import OccupancyMap

def find_neighbors(current_node, range = 1):
    x = current_node[0]
    y = current_node[1]
    z = current_node[2]
    neighboring_coordinates = []
    for dx in [-range, 0, range]:
        for dy in [-range, 0, range]:
            for dz in [-range, 0, range]:
                # Skip the case where all offsets are 0 (same coordinate)
                if dx == 0 and dy == 0 and dz == 0:
                    continue
                neighboring_coordinates.append((x + dx, y + dy, z + dz))
    return neighboring_coordinates

def calculate_euclidean_distance(node1, node2):
    x_diff = node1[0] - node2[0]
    y_diff = node1[1] - node2[1]
    z_diff = node1[2] - node2[2]
    diff = np.sqrt(x_diff**2 + y_diff**2 + z_diff**2)
    return diff
    
# def update(queue, cost_lib, tree_lib, neighbor_node, current_node, cost):
#     heappush(queue, (cost, neighbor_node))
#     cost_lib[neighbor_node] = cost
#     tree_lib[neighbor_node] = current_node
#     return queue, cost_lib, tree_lib
    
def graph_search(world, resolution, margin, start, goal, if_astar):
    """
    Parameters:
        world,      World object representing the environment obstacles
        resolution, xyz resolution in meters for an occupancy map, shape=(3,)
        margin,     minimum allowed distance in meters from path to obstacles.
        start,      xyz position in meters, shape=(3,)
        goal,       xyz position in meters, shape=(3,)
        astar,      if True use A*, else use Dijkstra
    Output:
        return a tuple (path, expanded_nodes_num)
        path,       xyz position coordinates along the path in meters with
                    shape=(N,3). These are typically the centers of tree_lib
                    voxels of an occupancy map. The first point must be the
                    start and the last point must be the goal. If no path
                    exists, return None.
        expanded_nodes_num, the number of nodes that have been expanded
    """

    # 0. define coefficients
    if if_astar is True:
        heuristic_cost_coefficient = 1.3 # A star
    else:
        heuristic_cost_coefficient = 0.0 # Dijkstra's 
    queue = []

    # 1. get the occupancy map
    occupancy_map = OccupancyMap(world, resolution, margin)
    
    # 2. get the start point and goal point 
    start_node = tuple(occupancy_map.metric_to_index(start))
    goal_node = tuple(occupancy_map.metric_to_index(goal))

    # 3. initialize
    heappush(queue, (0, start_node))   # (cost, (x_index,y_index,z_index))
    cost_lib = {start_node: 0}   # define a dictionary to restore the cost
    tree_lib = {start_node: None}   # dictionary of points that actually moved to, record the child node

    # 4. search
    while queue:
        _, current_node = heappop(queue)
        if current_node == goal_node:
            print("Goal reached successfully !!!")
            break

        # 1. find the neighborhoods
        neighbor_nodes = find_neighbors(current_node)

        # 2. iterate all the neighborhoods
        for neighbor_node in neighbor_nodes:

            # 2.1 assume cost(neighbor) = cost(current) + cost(current to neighbor)
            if occupancy_map.is_occupied_index(neighbor_node):
                updated_cost = cost_lib[current_node] + 1000000   # if occupied, give this node a really huge cost
            else:   # if not occupied, cost(neighbor) = cost(current) + cost(current to neighbor)
                updated_cost = cost_lib[current_node] + calculate_euclidean_distance(current_node, neighbor_node) + heuristic_cost_coefficient * calculate_euclidean_distance(current_node, goal_node)

            # 2.2 check if cost(current) + cost(current to neighbor) < cost(neighbor), if yes, update cost(neighbor) as cost(current) + cost(current to neighbor)
            if (neighbor_node not in cost_lib) or (updated_cost < cost_lib[neighbor_node]):
                heappush(queue, (updated_cost, neighbor_node))
                cost_lib[neighbor_node] = updated_cost
                tree_lib[neighbor_node] = current_node
                

    # 5. generate the path
    print("Generating the path:")
    expanded_nodes_num = len(tree_lib)
    path = goal   # goal as root
    print(f',{path} ', end= '')
    node_path = goal_node   # goal as root

    if goal_node not in tree_lib:
        path = None
        print("Search failed !!!")

    while node_path != start_node:
        node_path = tree_lib[node_path]
        print(f',{node_path} ', end= '')
        waypoint = occupancy_map.index_to_metric_center(node_path)
        path = np.vstack([waypoint, path])
    path = np.vstack([start, path])

    # Return a tuple (path, expanded_nodes_num)
    return path, expanded_nodes_num