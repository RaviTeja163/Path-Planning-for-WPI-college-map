# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import math
from scipy import spatial
from PIL import Image


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size

        self.samples = []  # list of sampled points
        self.graph = nx.Graph()  # constructed graph
        self.path = []  # list of nodes of the found path

    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        obstacle_collision = False
        path = np.linspace([p1[0], p1[1]], [p2[0], p2[1]], num=10, dtype=int)
        for pt in path:
            if self.map_array[pt[0], pt[1]] == 0:
                obstacle_collision = True
                break
        return obstacle_collision

    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        self.samples.append((0, 0))
        row_points = int(math.sqrt(n_pts))
        col_points = int(math.sqrt(n_pts))
        for i in np.linspace(0, self.size_row - 1, row_points):
            for j in np.linspace(0, self.size_col - 1, col_points):
                if self.map_array[int(i), int(j)] == 1:  # checking for obstacles
                    self.samples.append((i, j))

    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        self.samples.append((0, 0))
        coordsList = np.random.randint(self.size_row, size=(n_pts, 2))
        for coords in coordsList:
            if self.map_array[coords[0], coords[1]] == 0:  # check for obstacles
                continue
            self.samples.append((coords[0], coords[1]))

    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        self.samples.append((0, 0))
        coordsList = np.random.randint(self.size_row, size=(n_pts, 2))
        for q1 in coordsList:
            q1_x = q1[0]
            q1_y = q1[1]
            q2_x = int(np.random.normal(q1_x, 10, 1))
            q2_y = int(np.random.normal(q1_y, 10, 1))
            if q2_x >= self.size_row or q2_y >= self.size_col or q2_x <= 0 or q2_y <= 0:  # checking for boundaries of the map
                continue
            if self.map_array[q1_x, q1_y] != self.map_array[q2_x, q2_y]:  # if one in obstacle space and other in free space
                # store the one which is in free space
                if self.map_array[q1_x, q1_y] == 1:
                    self.samples.append((q1_x, q1_y))
                elif self.map_array[q2_x, q2_y] == 1:
                    self.samples.append((q2_x, q2_y))

    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        self.samples.append((0, 0))
        coordsList = np.random.randint(self.size_row, size=(n_pts, 2))
        for q1 in coordsList:
            q1_x = q1[0]
            q1_y = q1[1]
            q2_x = int(np.random.normal(q1_x, 20, 1))
            q2_y = int(np.random.normal(q1_y, 20, 1))
            if q2_x >= self.size_row or q2_y >= self.size_col or q2_x <= 0 or q2_y <= 0:  # checking for boundaries of the map
                continue
            if self.map_array[q1_x, q1_y] == 0 and self.map_array[q2_x, q2_y] == 0:  # if both q1 and q2 are in obstacle space
                mid_x = int((q1_x + q2_x) / 2)
                mid_y = int((q1_y + q2_y) / 2)
                if self.map_array[mid_x, mid_y] == 1:  # if middle point is in free space
                    self.samples.append((mid_x, mid_y))


    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict(zip(range(len(self.samples)), node_pos))
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])

        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y', ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12, node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12, node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()

    def sample(self, n_pts=1000, sampling_method="uniform"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample, 
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        # Finding the pairs of points that need to be connected and computing their distance/weight. And storing them as
        # pairs = [(p_id0, p_id1, weight_01), (p_id0, p_id2, weight_02), (p_id1, p_id2, weight_12) ...]
        pairs = []
        p_ids = []
        KDtree = spatial.KDTree(np.array(self.samples))
        KDtree_pairs = KDtree.query_pairs(80)
        for pair in KDtree_pairs:
            point1 = self.samples[pair[0]]
            point2 = self.samples[pair[1]]
            distance = self.dis(point1, point2)
            if self.check_collision(point1, point2):
                continue
            if pair[0] not in p_ids:
                p_ids.append(pair[0])
            if pair[1] not in p_ids:
                p_ids.append(pair[1])
            pairs.append((pair[0], pair[1], distance))

        # Using sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01), 
        #                                     (p_id0, p_id2, weight_02), 
        #                                     (p_id1, p_id2, weight_12) ...])

        # 'p_id' here is an integer, representing the order of current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)], p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from(p_ids)
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" % (n_nodes, n_edges))

    def search(self, start, goal, sampling_method="uniform"):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])

        # Finding the pairs of points that need to be connected and computing their distance/weight. And storing them as
        # start_pairs = [(start_id, p_id0, weight_s0), (start_id, p_id1, weight_s1), (start_id, p_id2, weight_s2) ...]
        start_pairs = []
        goal_pairs = []

        start_goal_radius = 30
        if sampling_method == "bridge" or sampling_method == "gaussian":
            start_goal_radius = 100

        KDtree = spatial.KDTree(np.array(self.samples))
        KDtree_start_pairs = spatial.KDTree.query_ball_point(KDtree, start, start_goal_radius)
        KDtree_goal_pairs = spatial.KDTree.query_ball_point(KDtree, goal, start_goal_radius)
        for s in KDtree_start_pairs:
            if self.check_collision(start, self.samples[s]):
                continue
            distance = self.dis(start, self.samples[s])
            start_pairs.append(('start', s, distance))
        for g in KDtree_goal_pairs:
            if self.check_collision(goal, self.samples[g]):
                continue
            distance = self.dis(goal, self.samples[g])
            start_pairs.append(('goal', g, distance))

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)

        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            print("PATH", self.path)
            for i in range(1, len(self.path)-1):
                print(self.samples[self.path[i]])
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" % path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")

        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)


def load_map(file_path, resolution_scale):
    ''' Load map from an image and return a 2D binary numpy array
        where 0 represents obstacles and 1 represents free space
    '''
    # Load the image with grayscale
    img = Image.open(file_path).convert('L')
    # Rescale the image
    size_x, size_y = img.size
    new_x, new_y  = int(size_x*resolution_scale), int(size_y*resolution_scale)
    img = img.resize((new_x, new_y), Image.ANTIALIAS)

    map_array = np.asarray(img, dtype='uint8')

    # Get bianry image
    threshold = 127
    map_array = 1 * (map_array > threshold)

    # Result 2D numpy array
    return map_array


if __name__ == "__main__":
    # Load the map
    start = (200, 75)
    goal  = (30, 250)
    map_array = load_map("WPI_map.jpg", 0.3)

    # Planning class
    PRM_planner = PRM(map_array)

    # Search with PRM
    PRM_planner.sample(n_pts=50, sampling_method="uniform")
    PRM_planner.search(start, goal)
    PRM_planner.sample(n_pts=1000, sampling_method="random")
    PRM_planner.search(start, goal)
    PRM_planner.sample(n_pts=3000, sampling_method="gaussian")
    PRM_planner.search(start, goal)
    PRM_planner.sample(n_pts=30000, sampling_method="bridge")
    PRM_planner.search(start, goal)
