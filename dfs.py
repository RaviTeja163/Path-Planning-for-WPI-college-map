from queue import Queue
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

def dfs_util(grid, visited, x, y, goal, path):
    dRow = [0, 1, 0, -1]
    dCol = [1, 0, -1, 0]  # order of direction - right, down, left, up
    path.append((x, y))
    visited[x][y] = True
    if (x == goal[0] and y == goal[1]):
        return True

    for i in range(4):
        adjx = x + dRow[i]
        adjy = y + dCol[i]
        if not isValid(grid, visited, adjx, adjy):
            continue
        if dfs_util(grid, visited, adjx, adjy, goal, path):
            return True
            break
        visited[adjx][adjy] = True
    return False

def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm
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

    rows = len(grid)
    columns = len(grid[0])
    visited = [[False for i in range(rows)] for i in range(columns)]
    found = dfs_util(grid, visited, start[0], start[1], goal, path)
    path = [list(x) for x in path]
    steps = len(path)
    # print('final', path)
    if found:
        print(f"It takes {steps} steps to find a path using DFS")
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
    plt.title('DFS')
    plt.axis('scaled')
    plt.gca().invert_yaxis()

if __name__ == "__main__":
    # Load the map
    grid, start, goal = load_map('map.csv')

    # Search
    dfs_path, dfs_steps = dfs(grid, start, goal)

    # Show result
    draw_path(grid, dfs_path)
    plt.show()