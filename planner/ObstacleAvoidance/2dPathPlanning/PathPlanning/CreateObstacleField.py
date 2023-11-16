"""
Create an Obstacle Field
"""

# Import random module
import random
# Import turtle module
import turtle

# Function to randomly distribute obstacles with coverage rho [0,1]
def f(rho):
    # Set grid size to 128 x 128
    rows, cols = 128, 128
    grid = [[0]*rows for i in range(cols)]
    numObstacles = rows*cols*rho//4
    # Place obstacles
    while numObstacles > 0:
        if placeObstacle(grid):
            numObstacles -= 1
    return grid

# Function to place obstacle
def placeObstacle(grid):
    rows, cols = len(grid), len(grid[0])
    # Randomize location and obstacle type
    i = random.randint(0, rows-1)
    j = random.randint(0, cols-1)
    obstacleType = random.randint(0, 3)
    # Try placing obstacle -- if successful, return True
    if obstacleType == 0:
        if i < rows-3 and sum([grid[i][j]]) == 0:
            grid[i][j] = 1
            return True
    #if obstacleType == 0:
        #if i < rows-3 and sum([grid[i][j], grid[i+1][j], grid[i+2][j], grid[i+3][j]]) == 0:
            #grid[i][j], grid[i+1][j], grid[i+2][j], grid[i+3][j] = 1, 1, 1, 1
            #return True
    #elif obstacleType == 1:
        #if i < rows-2 and j < cols-1 and sum([grid[i][j], grid[i][j+1], grid[i+1][j+1], grid[i+2][j+1]]) == 0:
            #grid[i][j], grid[i][j+1], grid[i+1][j+1], grid[i+2][j+1] = 1, 1, 1, 1
            #return True
    #elif obstacleType == 2:
       # if i < rows-2 and j < cols-1 and sum([grid[i][j], grid[i][j+1], grid[i+1][j+1], grid[i+2][j+1]]) == 0:
            #grid[i][j], grid[i][j+1], grid[i+1][j+1], grid[i+2][j+1] = 1, 1, 1, 1
            #return True
    #elif obstacleType == 3:
        #if i < rows-2 and j < cols-1 and sum([grid[i+1][j], grid[i][j+1], grid[i+1][j+1], grid[i+2][j+1]]) == 0:
            #grid[i+1][j], grid[i][j+1], grid[i+1][j+1], grid[i+2][j+1] = 1, 1, 1, 1
            #return True
    # Else unsuccessful, return False
    return False

# Function to draw grid
def drawGrid(grid, colors):
    rows, cols = len(grid), len(grid[0])
    side = 4
    # Speed up animation
    turtle.tracer(0, 0)
    # Set starting position of pen
    turtle.penup()
    turtle.back(side * cols / 2)
    turtle.left(90)
    turtle.forward(side * rows / 2)
    turtle.right(90)
    turtle.pendown()
    # Loop through grid
    for i in range(rows):
        for j in range(cols):
            # Draw blue if occupied square
            turtle.color('black', colors[grid[i][j]])
            turtle.begin_fill()
            # Draw square
            for k in range(4):
                turtle.forward(side)
                turtle.right(90)
            turtle.end_fill()
            turtle.forward(side)
        # Reset turtle for next row
        turtle.right(90)
        turtle.forward(side)
        turtle.left(90)
        turtle.backward(side * cols)
    turtle.pencolor('white')
    # Hide turtle
    turtle.hideturtle()
    # Update screen
    turtle.done()

if __name__ == '__main__':
    random.seed(1)
    # Prompt rho for input
    rho = float(input('Enter rho: '))
    # If valid rho, call f(rho)
    if 0 <= rho <= 1:
        grid = f(rho)
        # Draw grid
        drawGrid(grid, ['white', 'blue'])
    # Else print error
    else:
        print('Error: rho must be between 0 and 1')
