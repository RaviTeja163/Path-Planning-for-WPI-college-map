from queue import PriorityQueue
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import csv

def isValid(grid, vis, row, col):
    if (row < 0 or col < 0 or row >= len(grid) or col >= len(grid[0])):
        return False  # checking the matrix limits
    if grid[row][col] == 1:
        return False  # checking for obstacles
    if vis[row][col]:
        return False  # checking for already visited nodes
    return True

def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)
    '''
    path = []
    steps = 0
    found = False
    dRow = [0, 1, 0, -1]
    dCol = [1, 0, -1, 0]  # order of direction - right, down, left, up
    queue = PriorityQueue()
    queue.put((0, start))  # Priority 0 for the start cell
    rows = len(grid)
    columns = len(grid[0])
    grid_cost = [[1 for i in range(rows)] for i in range(columns)]  # cost is 1 for each cell in our grid
    visited = [[False for i in range(rows)] for i in range(columns)]
    visited[start[0]][start[1]] = True
    parent = dict()
    cost_so_far = dict()
    parent[(start[0], start[1])] = None
    cost_so_far[(start[0], start[1])] = 0
    while not queue.empty():
        print(queue.queue)
        node = queue.get()
        node = node[-1]
        x = node[0]
        y = node[1]
        steps = steps + 1
        if x == goal[0] and y == goal[1]:
            found = True
            break
        for i in range(4):
            adjx = x + dRow[i]
            adjy = y + dCol[i]
            if isValid(grid, visited, adjx, adjy):
                new_cost = cost_so_far[(x, y)] + grid_cost[adjx][adjy]
                heuristic = abs(goal[0] - adjx) + abs(goal[1] - adjy)
                if (adjx, adjy) not in cost_so_far or new_cost < cost_so_far[(adjx, adjy)]:
                    cost_so_far[(adjx, adjy)] = new_cost
                    priority = new_cost + heuristic
                    queue.put((priority, [adjx, adjy]))
                    parent[(adjx, adjy)] = (x, y)
                    visited[adjx][adjy] = True
    # finding the path
    u = (goal[0], goal[1])
    while u is not None:
        path.append(u)
        u = parent[u]
    path.reverse()
    path = [list(x) for x in path]
    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    return path, steps

# Load map, start and goal point.
def load_map(file_path):
    grid = []
    start = [0, 0]
    goal = [0, 0]
    # Load from the file
    with open(file_path, 'r') as map_file:
        reader = csv.reader(map_file)
        for i, row in enumerate(reader):
            # load start and goal point
            if i == 0:
                start[0] = int(row[1])
                start[1] = int(row[2])
            elif i == 1:
                goal[0] = int(row[1])
                goal[1] = int(row[2])
            # load the map
            else:
                int_row = [int(col) for col in row]
                grid.append(int_row)
    return grid, start, goal

# Draw final results
def draw_path(grid, path):
    # Visualization of the found path using matplotlib
    fig, ax = plt.subplots(1)
    ax.margins()
    # Draw map
    row = len(grid)     # map size
    col = len(grid[0])  # map size
    for i in range(row):
        for j in range(col):
            if grid[i][j]: 
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='k'))  # obstacle
            else:          
                ax.add_patch(Rectangle((j-0.5, i-0.5),1,1,edgecolor='k',facecolor='w'))  # free space
    # Draw path
    for x, y in path:
        ax.add_patch(Rectangle((y-0.5, x-0.5),1,1,edgecolor='k',facecolor='b'))          # path
    ax.add_patch(Rectangle((start[1]-0.5, start[0]-0.5),1,1,edgecolor='k',facecolor='g'))# start
    ax.add_patch(Rectangle((goal[1]-0.5, goal[0]-0.5),1,1,edgecolor='k',facecolor='r'))  # goal
    # Graph settings
    plt.title('A*')
    plt.axis('scaled')
    plt.gca().invert_yaxis()

if __name__ == "__main__":
    # Load the map
    grid, start, goal = load_map('map.csv')

    # Search
    aster_path, aster_steps = astar(grid, start, goal)

    # Show result
    draw_path(grid, aster_path)
    plt.show()