import math
from collections import defaultdict
from heapq import heapify, heappush, heappop

class Astar:
    def __init__(self, start_x, start_y, target_x, target_y, obs_center_x, obs_center_y,
                 obs_len, obs_width, padding, lm_padding, x_max, y_max, edge_length):
        '''
        Notation:
        i -> coordinate in x axis.
        j -> coordinate in y axis.
        obs_len -> Length of obstacle (x axis)
        obs_width -> Length of obstalce (y axis).
        padding -> Amount of padding in m.
        lm_padding -> Amount pf padding from LMs in m.
        x_max -> Length of arena in m.
        y_max -> Width of arena in m.
        edge_length -> grid square length in m.

        i-j and x-y both start from bottom left corner of the arena
        '''

        self.start_i = int(start_x/edge_length)
        self.start_j = int(start_y/edge_length)
        self.target_i = int(target_x/edge_length)
        self.target_j = int(target_y/edge_length)
        self.edge_length = edge_length

        self.obs_i = int(obs_center_x/edge_length)  # converting obstacle x-y to i-j
        self.obs_j = int(obs_center_y/edge_length)

        # Defining the range of coordinates bot shouldn't go inside
        self.obs_i_max = int((self.obs_i + (((obs_len/2) + padding))/ self.edge_length))
        self.obs_i_min = int((self.obs_i - (((obs_len/2) + padding))/ self.edge_length))

        self.obs_j_max = int((self.obs_j + (((obs_width/2) + padding))/ self.edge_length))
        self.obs_j_min = int((self.obs_j - (((obs_width/2) + padding))/ self.edge_length))

        # Ensuring bot doesn't run into landmarks
        self.i_max = int((x_max-lm_padding)/edge_length)
        self.j_max = int((y_max-lm_padding)/edge_length)

        self.i_min = int(lm_padding/edge_length)
        self.j_min = int(lm_padding/edge_length)

        self.lm_padding = lm_padding

    def distance_from_obstacle(self, i, j):
        if i < self.obs_i_min and j < self.obs_j_min:
            return abs(i-self.obs_i_min) + abs(j - self.obs_j_min)

        elif i >= self.obs_i_min and i <= self.obs_i_max and j < self.obs_j_min:
            return abs(j-self.obs_j_min)

        elif i> self.obs_i_max and j < self.obs_j_min:
            return abs(i-self.obs_i_max) + abs(j - self.obs_j_min)

        elif i > self.obs_i_max and j >= self.obs_j_min and j <= self.obs_j_max:
            return abs(i-self.obs_i_max)

        elif i > self.obs_i_max and j > self.obs_j_max:
            return abs(i-self.obs_i_max) + abs(j - self.obs_j_max)

        elif i >= self.obs_i_min and i<= self.obs_i_max and j > self.obs_j_max:
            return abs(j-self.obs_j_max)

        elif i < self.obs_i_min and j > self.obs_j_max:
            return abs(i-self.obs_i_min) + abs(j-self.obs_j_max)

        else:
            return abs(i-self.obs_i_min)

    def distance_from_boundary(self, i, j):
        return min(abs(i-self.i_min), abs(j-self.j_min),  abs(i-self.i_max), abs(j-self.j_max))

    # def manhattan_distance(self, start_i, start_j, target_i, target_j):
    #     return (abs(target_i - start_i) + abs(target_j - start_j))

    def euclidean_distance(self, start_i, start_j, target_i, target_j):
        return math.sqrt((target_i - start_i)**2 + (target_j - start_j)**2)*self.edge_length

    def heuristic_min_distance(self, i, j):
        return self.euclidean_distance(i, j, self.target_i, self.target_j)

    def heuristic_max_safety(self, i, j):
        return self.euclidean_distance(i, j, self.target_i, self.target_j) - self.distance_from_obstacle(i, j)

    def check_if_valid(self, i, j):
        # If within padding -> invalid
        if  i<= self.obs_i_max and i>= self.obs_i_min and j <=self.obs_j_max and j >= self.obs_j_min:
            return False
        # If outside of map
        elif i< self.i_min or j < self.j_min or i > self.i_max or j > self.j_max:
            return False
        else:
            return True
        
    def reconstruct_path(self, cameFrom):
        optimal_path = []
        node = (self.target_i, self.target_j)
        optimal_path.append(node)

        while node in cameFrom:
            node = cameFrom[node]
            optimal_path.append(node)

        return optimal_path[::-1]

    def search(self, minDistance):
        gScore = defaultdict(lambda:float("inf"))
        gScore[(self.start_i, self.start_j)] = 0
        cameFrom = {}

        visited = set()

        if minDistance:
            heuristic = self.heuristic_min_distance
        else:
            heuristic = self.heuristic_max_safety

        openSet = [(gScore[(self.start_i, self.start_j)]+heuristic(self.start_i, self.start_j), (self.start_i, self.start_j))]

        while (openSet):
            _, node = heappop(openSet)

            # Duplicate Node -> no need to search further
            if node in visited:
                continue

            # Adding poped node to visited set and optimalPath
            visited.add(node)

            # Goal reached -> no need to search further
            if node[0] == self.target_i and node[1] == self.target_j:
                break

            i = node[0]
            j = node[1]

            for neighbor in [(i+1, j), (i+1, j+1), (i+1, j-1), (i-1, j), (i-1, j+1), (i-1, j-1), (i, j+1), (i, j-1)]:
            # for neighbor in [(i+1, j), (i-1, j), (i, j+1), (i, j-1)]:
                # Checking to ensure that the neighbor is outside the padding area
                # Also, checks that the bot is within the arena
                if self.check_if_valid(neighbor[0], neighbor[1]):
                    old_g_neighbor = gScore[neighbor]
                    gScore[neighbor] = min(old_g_neighbor, gScore[node] + self.euclidean_distance(i, j, neighbor[0], neighbor[1]))


                    # If these is an improvement in gScore add it back to the heap
                    if gScore[neighbor] < old_g_neighbor:
                        heappush(openSet, (gScore[neighbor] + heuristic(neighbor[0], neighbor[1]), neighbor))
                        cameFrom[neighbor] = node

        print("done searching")
        return self.reconstruct_path(cameFrom)

def compress_path(path):
    '''
    path Nx2 array
    N >= 2
    '''
    path = np.array(path)
    compressed = []
    compressed.append(path[0])
    previous_dir = path[1] - path[0]
    for i in range(2, path.shape[0]):
      current_dir = path[i] - path[i-1]
      if current_dir[0] == previous_dir[0] and current_dir[1] == previous_dir[1]:
        continue
      else:
        compressed.append(path[i-1])
        previous_dir = current_dir
    return np.array(compressed)

def write_to_waypoints(filename, path):
    with open(filename, 'w') as f:
        for i in range(1, len(path)):
            curr, prev = path[i], path[i-1]
            diff = curr - prev
            ori = math.atan2(diff[1], diff[0])
            f.write(str(curr[0]) +","+ str(curr[1]) +","+ str(ori))
        f.write(str(path[-1][0])+","+str(path[-1][0][1])+","+str(0))

import matplotlib.pyplot as plt
import numpy as np

def plot_graph(planner, path):
    plot_array = np.array(path)
    boundary = np.array([
        [planner.i_max, planner.j_max],
        [planner.i_max, planner.j_min],
        [planner.i_min, planner.j_min],
        [planner.i_min, planner.j_max],
        [planner.i_max, planner.j_max],
    ])
    plt.plot(boundary[:,0], boundary[:,1], color='r')

    obs = np.array([
        [planner.obs_i_max, planner.obs_j_max],
        [planner.obs_i_max, planner.obs_j_min],
        [planner.obs_i_min, planner.obs_j_min],
        [planner.obs_i_min, planner.obs_j_max],
        [planner.obs_i_max, planner.obs_j_max],
    ])
    plt.plot(obs[:,0], obs[:,1], color='r')

    plt.plot(plot_array[:,0], plot_array[:,1])
    plt.xlim(0, planner.i_max+ (planner.lm_padding/planner.edge_length))
    plt.ylim(0, planner.j_max+ (planner.lm_padding/planner.edge_length))
    plt.grid(True)
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')

    ticks = np.linspace(0,
                        max(planner.i_max + planner.lm_padding/planner.edge_length, planner.j_max+ planner.lm_padding/planner.edge_length),
                        num=10
                    )
    ax.set_xticks(ticks)
    ax.set_yticks(ticks)

    plt.show()

"""# Min Distance"""

planner = Astar(1.5, 0.3, 0.3, 1.5, 0.9, 0.9,
                 0.45, 0.35, 0.20, 0.15, 1.8, 1.8, 0.01)
minDistPath = planner.search(True)
# plot_graph(planner, minDistPath)
compressed_path = compress_path(minDistPath)
plot_graph(planner, compressed_path)
print("Min Distance:", len(minDistPath), len(compressed_path))

"""# Max Safty"""

planner = Astar(1.5, 0.3, 0.3, 1.5, 0.9, 0.9,
                 0.45, 0.35, 0.20, 0.30, 1.8, 1.8, 0.01)
maxSafetyPath = planner.search(False)
# plot_graph(planner, maxSafetyPath)
compressed_path = compress_path(maxSafetyPath)
plot_graph(planner, compressed_path)
print("Max Safty", len(maxSafetyPath), len(compressed_path))