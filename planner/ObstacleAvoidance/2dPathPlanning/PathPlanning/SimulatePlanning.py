
from PathPlanning.CreateObstacleField import *
from PathPlanning.Planners import *

# GridWorld class
class GridWorld:

    # Constructor
    def __init__(self, rho):
        self.grid = f(rho)
        self.rows, self.cols = len(self.grid), len(self.grid[0])
        self.createGraph()
        self.start = (7, 48)
        self.end = (102, 78)

    # Method to create graph
    def createGraph(self):
        self.graph = {}
        for i in range(self.rows):
            for j in range(self.cols):
                if self.grid[i][j] == 0:
                    self.graph[(i, j)] = []
                    if j < self.cols - 1 and self.grid[i][j + 1] == 0:
                        self.graph[(i, j)].append((i, j + 1))
                    if i < self.rows - 1 and self.grid[i + 1][j] == 0:
                        self.graph[(i, j)].append((i + 1, j))
                    if i > 0 and self.grid[i - 1][j] == 0:
                        self.graph[(i, j)].append((i - 1, j))
                    if j > 0 and self.grid[i][j - 1] == 0:
                        self.graph[(i, j)].append((i, j - 1))

    # Method to solve with DFS

    # Method to solve with Dijkstra's algorithm
    def solveDijkstra(self, draw=True):
        level, path, iterations, found = Dijkstra(self.graph, self.start, self.end)
        if found:
            print('Path found!')
            print('Dijkstra Number of iterations:', iterations)
            if draw:
                gridDijkstra = [row[:] for row in self.grid]
                gridDijkstra[self.start[0]][self.start[1]] = 2
                gridDijkstra[self.end[0]][self.end[1]] = 2
                for row, col in path:
                    gridDijkstra[row][col] = 3
                drawGrid(gridDijkstra, ['white', 'blue', 'black', 'green'])
        else:
            print('No valid path found!')

    # Method to solve with Random Planner
    def solveRandomPlanner(self, draw=True):
        traveled_path, iterations, found = RandomPlanner(self.graph, self.start, self.end)
        if found:
            print('Path found!')
            print('Random Planner Number of iterations:', iterations)
            if draw:
                gridRandom = [row[:] for row in self.grid]
                gridRandom[self.start[0]][self.start[1]] = 2
                gridRandom[self.end[0]][self.end[1]] = 2
                for row, col in traveled_path:
                    gridRandom[row][col] = 3
                drawGrid(gridRandom, ['white', 'blue', 'black', 'green'])
        else:
            print('No valid path found!')

# Main function to run program
def main():
    random.seed(1)
    draw = True
    # Prompt rho for input
    rho = float(input('Enter rho: '))
    # If valid rho, call f(rho)
    if 0 <= rho <= 1:
        simulation = GridWorld(rho)
        print('Select Search Algorithm:')
        print('  1: Dijkstra')
        print('  2: Random Planner')
        choice = input('Enter choice: ')
        if choice == '1':
            simulation.solveDijkstra(draw)
        elif choice == '2':
            simulation.solveRandomPlanner(draw)
    # Else print error
    else:
        print('Error: rho must be between 0 and 1')
    
# Run main function
if __name__ == '__main__':
    main()
