import cv2
import numpy as np
import math
import heapq
GRID = np.zeros((20, 20), dtype=int)
initial_pos=[]

# color_map =[(4,3),(11,10),(4,14)]
color_map=[]
# cells = [[[1, 0, 0, 1], [1, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 0, 0, 0], [1, 1, 0, 0], [1, 0, 0, 1], [1, 1, 0, 0], [1, 0, 1, 1], [1, 0, 0, 0], [1, 1, 0, 0], [1, 0, 1, 1], [1, 0, 1, 0], [1, 0, 0, 0], [1, 0, 1, 0], [1, 0, 0, 0], [1, 0, 1, 0], [1, 1, 0, 0]], [[0, 0, 0, 1], [1, 0, 0, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 1, 1, 0], [0, 0, 1, 1], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 0, 0], [1, 1, 0, 0], [0, 1, 0, 1], [1, 0, 0, 1], [0, 1, 1, 0], [1, 0, 0, 1], [0, 1, 1, 0]], [[0, 1, 0, 1], [0, 0, 0, 1], [1, 0, 1, 0], [1, 1, 0, 0], [0, 1, 1, 1], [0, 0, 1, 1], [1, 0, 0, 0], [1, 1, 0, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 1, 0, 0], [1, 0, 1, 1], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1], [0, 1, 1, 0], [0, 0, 0, 1], [1, 1, 1, 0], [0, 0, 1, 1], [1, 1, 1, 0]], [[0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 0], [0, 0, 1, 1], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 0, 0], [0, 1, 0, 1], [0, 1, 0, 1], [1, 1, 0, 1], [0, 1, 0, 1], [1, 0, 0, 1], [0, 1, 1, 0], [0, 1, 0, 1], [0, 0, 0, 1], [1, 1, 0, 0], [0, 1, 1, 1], [1, 0, 0, 1], [1, 0, 1, 0], [1, 1, 0, 0]], [[0, 0, 0, 1], [1, 1, 1, 0], [0, 0, 1, 1], [1, 0, 1, 0], [1, 0, 0, 0], [1, 1, 0, 0], [0, 1, 1, 1], [0, 0, 1, 1], [0, 1, 1, 0], [0, 1, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 0], [0, 0, 0, 1], [0, 1, 1, 0], [0, 0, 0, 1], [1, 0, 0, 0], [0, 1, 1, 0], [1, 0, 1, 1], [0, 1, 1, 0]], [[0, 0, 1, 1], [1, 0, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [0, 1, 1, 0], [0, 0, 1, 1], [1, 1, 1, 0], [1, 0, 0, 1], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 1, 1], [1, 1, 0, 0], [0, 0, 1, 1], [0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1], [1, 1, 0, 0], [1, 0, 0, 1], [1, 1, 0, 0]], [[1, 0, 0, 1], [1, 1, 0, 0], [0, 1, 0, 1], [0, 1, 0, 1], [1, 0, 0, 1], [1, 1, 0, 0], [1, 0, 0, 1], [0, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 1, 0, 0], [0, 0, 0, 1], [1, 1, 0, 0], [1, 1, 0, 1], [0, 0, 1, 1], [0, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 1], [0, 1, 1, 0], [0, 1, 0, 1]], [[0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [0, 1, 1, 0], [1, 0, 1, 1], [1, 1, 0, 0], [1, 0, 1, 1], [0, 1, 1, 0], [0, 1, 0, 1], [0, 0, 1, 1], [0, 1, 0, 0], [1, 1, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 0, 1, 0], [1, 0, 0, 0], [0, 1, 1, 0]], [[0, 1, 1, 1], [0, 1, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [0, 1, 1, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 1, 0, 0], [0, 1, 0, 1], [1, 0, 0, 1], [1, 1, 0, 0], [0, 0, 0, 1], [1, 0, 1, 0], [0, 0, 1, 0], [0, 0, 1, 0], [0, 1, 1, 0], [1, 0, 0, 1], [1, 1, 0, 0], [0, 0, 1, 1], [1, 1, 0, 0]], [[1, 0, 0, 1], [0, 1, 1, 0], [0, 1, 0, 1], [1, 0, 0, 1], [1, 0, 1, 0], [0, 1, 1, 0], [1, 1, 0, 1], [0, 0, 1, 1], [0, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1], [0, 1, 1, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0], [0, 1, 0, 1], [1, 1, 0, 1], [0, 1, 0, 1]], [[0, 0, 0, 1], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 1, 1], [1, 1, 0, 0], [1, 0, 1, 1], [0, 0, 0, 0], [1, 1, 1, 0], [0, 0, 0, 1], [0, 1, 0, 0], [0, 1, 0, 1], [1, 1, 0, 1], [0, 0, 1, 1], [1, 0, 1, 0], [1, 0, 0, 0], [1, 0, 0, 0], [1, 0, 0, 0], [0, 1, 1, 0], [0, 0, 1, 1], [0, 1, 0, 0]], [[0, 0, 0, 1], [1, 1, 0, 0], [1, 0, 1, 1], [1, 1, 0, 0], [0, 0, 1, 1], [1, 1, 0, 0], [0, 0, 1, 1], [1, 0, 0, 0], [0, 1, 1, 0], [0, 1, 0, 1], [0, 1, 0, 1], [0, 0, 0, 1], [1, 1, 0, 0], [1, 1, 0, 1], [0, 0, 0, 1], [0, 1, 0, 0], [0, 0, 0, 1], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 0, 0]], [[0, 1, 0, 1], [0, 0, 1, 1], [1, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 1], [0, 0, 1, 0], [1, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 1], [0, 1, 1, 0], [0, 0, 1, 1], [0, 1, 1, 0], [0, 0, 1, 1], [0, 0, 0, 0], [0, 1, 1, 0], [0, 1, 0, 1], [0, 0, 0, 1], [1, 1, 0, 0], [1, 0, 1, 1], [0, 1, 0, 0]], [[0, 1, 0, 1], [1, 0, 0, 1], [1, 1, 1, 0], [0, 1, 0, 1], [0, 1, 1, 1], [1, 0, 0, 1], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1], [1, 0, 0, 0], [1, 0, 1, 0], [1, 0, 1, 0], [1, 1, 1, 0], [0, 0, 1, 1], [1, 1, 0, 0], [0, 1, 1, 1], [0, 1, 1, 1], [0, 0, 0, 1], [1, 1, 0, 0], [0, 1, 1, 1]], [[0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 0], [0, 0, 1, 1], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 0, 1], [0, 0, 1, 0], [0, 0, 1, 0], [0, 0, 1, 0], [1, 1, 0, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 0, 0, 0], [0, 1, 1, 0], [1, 0, 0, 1], [1, 1, 0, 0], [0, 0, 0, 1], [0, 0, 1, 0], [1, 1, 0, 0]], [[0, 0, 1, 1], [1, 1, 0, 0], [0, 0, 1, 1], [1, 0, 1, 0], [1, 0, 1, 0], [1, 0, 0, 0], [0, 1, 1, 0], [1, 0, 0, 1], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 0, 1], [1, 1, 1, 0], [0, 0, 1, 1], [1, 1, 0, 0], [0, 1, 0, 1], [0, 0, 1, 1], [0, 0, 1, 0], [1, 1, 0, 0], [0, 1, 1, 1]], [[1, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 0], [1, 0, 0, 1], [1, 0, 1, 0], [0, 1, 1, 0], [1, 1, 0, 1], [0, 0, 0, 1], [1, 0, 0, 0], [1, 0, 1, 0], [1, 1, 0, 0], [0, 0, 0, 1], [1, 1, 0, 0], [1, 1, 0, 1], [0, 0, 0, 1], [0, 0, 1, 0], [1, 1, 0, 0], [1, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 0]], [[0, 0, 1, 1], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1], [1, 0, 1, 0], [1, 1, 0, 0], [0, 0, 1, 1], [0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 1], [0, 1, 1, 0], [0, 1, 0, 1], [0, 0, 1, 1], [0, 1, 0, 0], [0, 1, 1, 1], [1, 0, 0, 1], [0, 1, 1, 0], [0, 1, 0, 1], [1, 0, 0, 1], [0, 1, 1, 0]], [[1, 1, 0, 1], [0, 1, 0, 1], [0, 1, 1, 1], [0, 0, 1, 1], [1, 1, 0, 0], [0, 1, 0, 1], [1, 1, 0, 1], [1, 0, 0, 1], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 1, 0], [0, 1, 1, 0], [1, 0, 0, 1], [0, 1, 1, 0], [1, 0, 0, 1], [0, 1, 0, 0], [1, 0, 0, 1], [0, 1, 1, 0], [0, 1, 0, 1], [1, 1, 0, 1]], [[0, 0, 1, 1], [0, 0, 1, 0], [1, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 1, 1], [0, 1, 1, 0], [0, 0, 1, 1], [1, 0, 1, 0], [1, 0, 1, 0], [1, 1, 1, 0], [1, 0, 1, 1], [0, 0, 1, 0], [1, 0, 1, 0], [0, 1, 1, 0], [0, 0, 1, 1], [0, 0, 1, 0], [1, 0, 1, 0], [0, 0, 1, 0], [0, 1, 1, 0]]] 

frontier = []
explored = []
child_cell=(0,0)

# survivors =[(3,15),(10,11),(5,3)]
survivors=[]

reverse_direc = [[0,-1],[1,0],[0,1],[-1,0]]   #West,South,East,North
directions=[(-1,0),(0,1),(1,0),(0,-1)]    #North, East, South, West

from controller import Robot,Camera, Compass
from collections import deque
from math import floor

maze_size = 20

m = [[255] * maze_size for _ in range(maze_size)]   #Initialize the maze, m =maze

maze_copy = [[255] * maze_size for _ in range(maze_size)] 
damage_map = [[0] * maze_size for _ in range(maze_size)]

# goal=[0,0] 

# m[goal[0]][goal[1]] = 0 
# m[goal[0]][goal[1]] = 0

goal_number = 0  




# goals=[(3, 15), (10, 11), (5, 3)] 

goals=[]















direc = [[-1, 0], [0, 1], [1, 0], [0, -1]]    #North, East, South, West

cells = [[[2 for _ in range(4)] for _ in range(maze_size)] for _ in range(maze_size)]
  # for map the walls in the maze
 
cells[19][10][2]=1  # close the wall of starting position

speed = 6  # best value - 6
i=0 # use for starting position & direction calibration
j=0
k=0
theeta_z = 0  # use for precise left,right turn

colors=["Red","Yellow","Pink","Brown","Green"]

color_ranges = {
    "Red": ((0, 50), (0, 50), (150, 300)),
    "Yellow": ((0, 50), (200, 255), (200, 255)),
    "Pink": ((200, 255), (0, 50), (200, 255)),
    "Brown": ((0, 80), (90, 120), (100, 200)),
    "Green": ((0,50), (50, 100), (0, 50)),
}


if __name__ == "__main__":
    robot = Robot()
    TIME_STEP = 8

    
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    
    # camera = robot.getDevice('camera')  
    # camera.enable(TIME_STEP)
    
    camera = robot.getDevice('camera_1')  
    camera.enable(TIME_STEP)
    
    
    camera_2 = robot.getDevice('camera_2')  
    camera_2.enable(TIME_STEP)
    
    camera_3 = robot.getDevice('camera_3')  
    camera_3.enable(TIME_STEP)
    
    camera_4 = robot.getDevice('camera_4')  
    camera_4.enable(TIME_STEP)

    camera_5 = robot.getDevice('camera_5')  
    camera_5.enable(TIME_STEP)
        
    camera_front = robot.getDevice('camera_f')  
    camera_front.enable(TIME_STEP)
            
    width = camera_2.getWidth()
    height = camera_2.getHeight()
    
    
    width = camera.getWidth()
    height = camera.getHeight()
    
    gps = robot.getDevice('gps');
    gps.enable(TIME_STEP);

    left_motor.setPosition(float("inf"))
    right_motor.setPosition(float("inf"))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # left_motor_sensor = left_motor.getPositionSensor()
    # right_motor_sensor = right_motor.getPositionSensor()
    # left_motor_sensor.enable(TIME_STEP)
    # right_motor_sensor.enable(TIME_STEP)
    
    compass = robot.getDevice('compass')
    compass.enable(TIME_STEP)
    
    gyro = robot.getDevice("gyro")
    gyro.enable(TIME_STEP)



    proximity_sensors = []
    # sensor_names = ["ps7", "ps2", "ps4", "ps5",]  # Front, Right, Back , left
    sensor_names = ["front", "right","back","left","fl","fr"]
    for name in sensor_names:
        sensor = robot.getDevice(name)
        sensor.enable(TIME_STEP)
        proximity_sensors.append(sensor)
       
    # robot_position = [gps_to_cell_coordinates(gps)]
    # print(robot_position)
    
    
    def detect_color(average_color):
        r, g, b = average_color
        for color_name, ((r_min, r_max), (g_min, g_max), (b_min, b_max)) in color_ranges.items():
            if r_min <= r <= r_max and g_min <= g <= g_max and b_min <= b <= b_max:
                return color_name
        return "Unknown"
        
        
    def get_starting_pos(gps):
        
        
        
        gps_x = gps[0]
        gps_y = gps[1]
    
        maze_size = 20 
        square_size = 0.25  
        origin_offset = (maze_size / 2) * square_size 
        # Calculate row and column
        row = floor((origin_offset+0.5 - gps_y) / square_size)
        col = floor((gps_x + origin_offset) / square_size)
        # print(gps_values)
        return row, col
        # gps_values = gps.getValues()
        # print(gps_values)
        # robot_position = [gps_to_cell_coordinates(gps_values)]
        # print(robot_position)

    def floodfill(x, y):
        q = deque()
        q.append((x, y))
    
        while q:
            cx, cy = q.popleft()
            for dx, dy in direc:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx <= (maze_size-1) and 0 <= ny <= (maze_size-1) and m[nx][ny] == 255:
                    m[nx][ny] = m[cx][cy] + 1
                    q.append((nx, ny))
                    
       
    def floodfill_again(x, y,Maze):
        
        for i in range(maze_size):
            for j in range(maze_size):
                Maze[i][j] = 255  # Reset the maze with high values 
        Maze[x][y] = 0  
    
        queue = deque([(x,y)])  
    
        while queue:
            cx, cy = queue.popleft()
    
            for i, (dx, dy) in enumerate(direc):
                nx, ny = cx + dx, cy + dy
    
                if 0 <= nx < maze_size and 0 <= ny < maze_size:
                  
                    if cells[cx][cy][i] == 1:
                        continue
    
                   
                    if Maze[nx][ny] == 255:
                        Maze[nx][ny] = Maze[cx][cy] + 1
                        queue.append((nx, ny))     
                        


    def floodfill_last(x, y,Maze):
        # global Maze
        for i in range(maze_size):
            for j in range(maze_size):
                Maze[i][j] = 255  # Reset the maze with high values   
        Maze[x][y] = 0  
    
        queue = deque([(x,y)])  
    
        while queue:
            cx, cy = queue.popleft()
    
            for i, (dx, dy) in enumerate(direc):
                nx, ny = cx + dx, cy + dy
    
                if 0 <= nx < maze_size and 0 <= ny < maze_size:
                  
                    if cells[cx][cy][i] == 1 or cells[cx][cy][i] == 2:
                        continue
    
                   
                    if Maze[nx][ny] == 255:
                        Maze[nx][ny] = Maze[cx][cy] + 1
                        queue.append((nx, ny))   
                        
        
    def wall_mapping(x, y):
        global cells
        global robot_direction
        max_x, max_y = len(cells) - 1, len(cells[0]) - 1 
    
        if robot_direction == 0:  # North
            if proximity_sensors[0].getValue() > 1000:  # Front
                cells[x][y][0] = 1  
                if x > 0:
                    cells[x-1][y][2] = 1  
            else:
                cells[x][y][0] = 0
                if x > 0:
                    cells[x-1][y][2] = 0
            if proximity_sensors[1].getValue() > 1000:  # Right
                cells[x][y][1] = 1  # East wall
                if y < max_y:
                    cells[x][y+1][3] = 1  
            else:
                cells[x][y][1] = 0
                if y < max_y:
                    cells[x][y+1][3] = 0
            if proximity_sensors[2].getValue() > 1000:  # Back
                cells[x][y][2] = 1  # South wall
                if x < max_x:
                    cells[x+1][y][0] = 1 
            else:
                cells[x][y][2] = 0
                if x < max_x:
                    cells[x+1][y][0] = 0
            if proximity_sensors[3].getValue() > 1000:  # Left
                cells[x][y][3] = 1  # West wall
                if y > 0:
                    cells[x][y-1][1] = 1  
            else:
                cells[x][y][3] = 0
                if y > 0:
                    cells[x][y-1][1] = 0
    
        elif robot_direction == 1:  # East
            if proximity_sensors[0].getValue() > 1000:  # Front
                cells[x][y][1] = 1  # East wall
                if y < max_y:
                    cells[x][y+1][3] = 1 
            else:
                cells[x][y][1] = 0
                if y < max_y:
                    cells[x][y+1][3] = 0
            if proximity_sensors[1].getValue() > 1000:  # Right
                cells[x][y][2] = 1  # South wall
                if x < max_x:
                    cells[x+1][y][0] = 1  
            else:
                cells[x][y][2] = 0
                if x < max_x:
                    cells[x+1][y][0] = 0
            if proximity_sensors[2].getValue() > 1000:  # Back
                cells[x][y][3] = 1  # West wall
                if y > 0:
                    cells[x][y-1][1] = 1 
            else:
                cells[x][y][3] = 0  # West wall
                if y > 0:
                    cells[x][y-1][1] = 0                  

            if proximity_sensors[3].getValue() > 1000:  # Left
                cells[x][y][0] = 1  # North wall
                if x > 0:
                    cells[x-1][y][2] = 1  
            else:
                cells[x][y][0] = 0
                if x > 0:
                    cells[x-1][y][2] = 0    
        elif robot_direction == 2:  # South
            if proximity_sensors[0].getValue() > 1000:  # Front
                cells[x][y][2] = 1  # South wall
                if x < max_x:
                    cells[x+1][y][0] = 1 
            else:
                cells[x][y][2] = 0
                if x < max_x:
                    cells[x+1][y][0] = 0
            if proximity_sensors[1].getValue() > 1000:  # Right
                cells[x][y][3] = 1  # West wall
                # print("hi_2")
                if y > 0:
                    cells[x][y-1][1] = 1 
            else:
                cells[x][y][3] = 0
                if y > 0:
                    cells[x][y-1][1] = 0
            if proximity_sensors[2].getValue() > 1000:  # Back
                cells[x][y][0] = 1  # North wall
                if x > 0:
                    cells[x-1][y][2] = 1  
            else:
                cells[x][y][0] = 0
                if x > 0:
                    cells[x-1][y][2] = 0
            if proximity_sensors[3].getValue() > 1000:  # Left
                cells[x][y][1] = 1  # East wall
                if y < max_y:
                    cells[x][y+1][3] = 1
            else:
                cells[x][y][1] = 0
                if y < max_y:
                    cells[x][y+1][3] = 0
    
        elif robot_direction == 3:  # West
            if proximity_sensors[0].getValue() > 1000:  # Front
                cells[x][y][3] = 1  # West wall
                if y > 0:
                    cells[x][y-1][1] = 1 
            else:
                cells[x][y][3] = 0
                if y > 0:
                    cells[x][y-1][1] = 0 
            if proximity_sensors[1].getValue() > 1000:  # Right
                cells[x][y][0] = 1  # North wall
                if x > 0:
                    cells[x-1][y][2] = 1 
            else:
                cells[x][y][0] = 0
                if x > 0:
                    cells[x-1][y][2] = 0
            if proximity_sensors[2].getValue() > 1000:  # Back
                cells[x][y][1] = 1  # East wall
                if y < max_y:
                    cells[x][y+1][3] = 1  
            else:
                cells[x][y][2] = 0
                if x < max_x:
                    cells[x+1][y][0] = 0
            if proximity_sensors[3].getValue() > 1000:  # Left
                cells[x][y][2] = 1  # South wall
                if x < max_x:
                    cells[x+1][y][0] = 1  
            else:
                cells[x][y][2] = 0
                if x < max_x:
                    cells[x+1][y][0] = 0
                    
                
    
    def find_robot_direction(x,y):
        global robot_direction
        global robot_position
        motor_stop()
        if robot_direction == 0:
            if (x,y) == (-1,0): 
                robot_direction=0
                # while proximity_sensors[0].getValue() < 100:
                moveForward();
                robot_position[0]=robot_position[0]-1
            elif (x,y) == (0,1):
                robot_direction =1
                turnRight();
                robot_position[1]=robot_position[1]+1
            elif (x,y) == (1,0):
                robot_direction =2
                turnBack();
                robot_position[0]+=1
            elif (x,y) == (0,-1): 
                robot_direction =3
                turnLeft();
                robot_position[1]-=1
        elif robot_direction == 1:
            if (x,y) == (0,1):
                moveForward();
                robot_position[1]+=1
            elif (x,y) == (-1,0):
                turnLeft();   
                robot_direction=0
                robot_position[0]-=1    
            elif (x,y) == (1,0):
                turnRight();   
                robot_direction=2
                robot_position[0]+=1 
            elif (x,y) == (0,-1):
                turnBack();   
                robot_direction=3
                robot_position[1]-=1 
        elif robot_direction == 2:
            if (x,y) == (0,-1):
                turnRight()
                robot_direction=3
                robot_position[1]-=1 
            elif (x,y) == (1,0):
                moveForward()
                robot_position[0]+=1
            elif (x,y)==(0,1):
                turnLeft();
                robot_direction=1
                robot_position[1]+=1              
            elif (x,y)==(-1,0):
                turnBack();
                robot_direction = 0;
                robot_position[0]-=1 
        elif robot_direction == 3:
            if (x,y) == (-1,0):
                turnRight();
                robot_direction = 0
                robot_position[0]-=1 
            elif (x,y) == (1,0):
                turnLeft()
                robot_direction = 2
                robot_position[0]+=1
            elif (x,y) == (0,-1):
                moveForward();
                robot_position[1]-=1 
            elif (x,y) == (0,1):
                turnBack();
                robot_direction = 1;
                robot_position[1]+=1 

            
        # if ( robot_position[0] == goal[0]) and (robot_position[1]==goal[1]):
            # print("reached to the goal!!!")
            # motor_stop()
            # break
                     
                     
    def motor_stop():
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        start_time = robot.getTime()
        while robot.getTime() - start_time < 0.5:
            robot.step(TIME_STEP)



     
     
    def turnRight():
        global theeta_z
        left_motor.setVelocity(0.45)  
        right_motor.setVelocity(-0.45)
        while (theeta_z > -1.564/4):  #-1.565
            robot.step(TIME_STEP)
            wx,wy,wz = gyro.getValues()[0] , gyro.getValues()[1] , gyro.getValues()[2]
            theeta_z += wz*TIME_STEP*0.001
            # print(compass.getValues()[0])
            # print(theeta_z);        
        left_motor.setVelocity(0)
        right_motor.setVelocity(0) 
        theeta_z = 0 
        moveForward()

    def turnRight_3_75():
        global theeta_z
        left_motor.setVelocity(0.45)  
        right_motor.setVelocity(-0.45)
        while (theeta_z > -1.564/4):  #-1.565
            robot.step(TIME_STEP)
            wx,wy,wz = gyro.getValues()[0] , gyro.getValues()[1] , gyro.getValues()[2]
            theeta_z += wz*TIME_STEP*0.001
            # print(compass.getValues()[0])
            # print(theeta_z);        
        left_motor.setVelocity(0)
        right_motor.setVelocity(0) 
        theeta_z = 0 
        moveForward_3_75()    
            

    def turnLeft():
        global theeta_z
        left_motor.setVelocity(-0.45)  
        right_motor.setVelocity(0.45)
        while (theeta_z < 1.564/4):
            robot.step(TIME_STEP)
            wx,wy,wz = gyro.getValues()[0] , gyro.getValues()[1] , gyro.getValues()[2]
            theeta_z += wz*TIME_STEP*0.001
            # print(theeta_z);
        left_motor.setVelocity(0)
        right_motor.setVelocity(0) 
        theeta_z = 0 
        moveForward()

    # def turnBack():
        # left_motor.setVelocity(-3)  
        # right_motor.setVelocity(3) 
           
        # turn_duration = 1.5
        # start_time = robot.getTime()
        # while robot.getTime() - start_time < turn_duration:
            # robot.step(TIME_STEP)
        # print("turn finiseh")
        # left_motor.setVelocity(0)
        # right_motor.setVelocity(0)            
        # moveForward()
        
    def turnBack():
        global theeta_z
        left_motor.setVelocity(-0.2)  
        right_motor.setVelocity(0.2)
        while (theeta_z < 3.14/4):
            robot.step(TIME_STEP)
            wx,wy,wz = gyro.getValues()[0] , gyro.getValues()[1] , gyro.getValues()[2]
            theeta_z += wz*TIME_STEP*0.001
            # print(theeta_z);
        left_motor.setVelocity(0)
        right_motor.setVelocity(0) 
        theeta_z = 0 
        moveForward()         
    
    def turnBack_3_75():
        global theeta_z
        left_motor.setVelocity(-0.2)  
        right_motor.setVelocity(0.2)
        while (theeta_z < 3.14/4):
            robot.step(TIME_STEP)
            wx,wy,wz = gyro.getValues()[0] , gyro.getValues()[1] , gyro.getValues()[2]
            theeta_z += wz*TIME_STEP*0.001
            # print(theeta_z);
        left_motor.setVelocity(0)
        right_motor.setVelocity(0) 
        theeta_z = 0 
        moveForward_3_75()      
    
    
    def moveForward():
        distance = 0.25  
        wheel_velocity = speed 
        wheel_radius = 0.02
        

        
        forward_duration = distance / (wheel_velocity * wheel_radius)
        left_motor.setVelocity(wheel_velocity)
        right_motor.setVelocity(wheel_velocity)
        
        

            
        start_time = robot.getTime()
        
        while robot.getTime() - start_time < forward_duration:

            if proximity_sensors[0].getValue() > 3500: # Check there is wall infront of the robot
                print("Obstacle detected! Stopping.")
                motor_stop()
                return  

            
            robot.step(TIME_STEP)
            
       
        motor_stop()
        # print("first step done")
   

    def moveForward_3_75():
        distance = 0.375 
        wheel_velocity = speed 
        wheel_radius = 0.02
        
       
        forward_duration = distance / (wheel_velocity * wheel_radius)
        left_motor.setVelocity(wheel_velocity)
        right_motor.setVelocity(wheel_velocity)
                         
        start_time = robot.getTime()
        
        while robot.getTime() - start_time < forward_duration:

            if proximity_sensors[0].getValue() > 3500: # Check there is wall infront of the robot
                print("Obstacle detected! Stopping.")
                motor_stop()
                return              
            robot.step(TIME_STEP)                 
        motor_stop()   
             
     
    def move_next_pos(x,y,robot_direction,Maze):
        for i, (dx, dy) in enumerate(direc):
            nx, ny = x + dx, y + dy
            if (0 <= nx < maze_size) and (0 <= ny < maze_size) and (Maze[nx][ny]==Maze[x][y]-1) :
                if cells[x][y][i] == 1:
                    continue
                else:
                    find_robot_direction(dx,dy)
                    break
                    
    def move_back_pos(x,y,robot_direction,Maze):
        for i, (dx, dy) in enumerate(direc):
            nx, ny = x + dx, y + dy
            if (0 <= nx < maze_size) and (0 <= ny < maze_size) and (Maze[nx][ny]==Maze[x][y]-1) :
                if cells[x][y][i] == 1:
                    continue
                else:
                    find_robot_direction(dx,dy)
                    break        
        
    def is_stuck(x, y,Maze):
        for i, (dx, dy) in enumerate(direc):
            nx, ny = x + dx, y + dy
            if (0 <= nx < maze_size) and (0 <= ny < maze_size):
                if cells[x][y][i] == 0 and Maze[nx][ny] < Maze[x][y]:
                    return False  # Not stuck
        return True  # Stuck    
        
    # def check_goal(x,y):
        # if robot_position[0] == goal[0] and robot_position[1] == goal[1]:
            # print("Reached the goal!")
            # motor_stop()
            # return True
               
        # return False  

    def check_goal(x,y):
        global m
        global goal_number
        if robot_position[0] == goal[0] and robot_position[1] == goal[1]:
            print("Reached the goal!")
            floodfill_last(goal[0],goal[1],m)
            print(m)
            come_back(19,10);
            # check_color();
            # goal_number+=1
            # if goal_number<5:
                # m[goal[goal_number][0]][goal[goal_number][1]] = 0
                # floodfill(goal[goal_number][0],goal[goal_number][1])
            motor_stop()
            # if robot_position[0] == goal[-1][0] and robot_position[1] == goal[-1][1]:
            motor_stop()
            return True
           
              
        return False  
    def come_back(x,y):
        print(maze_copy)
        floodfill_again(x,y,maze_copy)
        while not (robot_position[0] == x and robot_position[1] == y):
            print(f"[{robot_position[0]}],[{robot_position[1]}]")
            wall_mapping(robot_position[0],robot_position[1])
            print(maze_copy)
            floodfill_again(x,y,maze_copy)
            move_back_pos(robot_position[0],robot_position[1],robot_direction,maze_copy) 
             
        if (robot_position[0] == x and robot_position[1] == y):   
            floodfill_last(x,y,maze_copy)    
            print(maze_copy)
            motor_stop();
            print(m)
            fast_run(robot_position[0],robot_position[1],robot_direction);
        
    def fast_run(x,y,robot_direction):
        if (x == 19 and y == 10):
            while not (robot_position[0] == goal[0] and robot_position[1] == goal[1]):
                print(f"[{robot_position[0]}],[{robot_position[1]}]")
                move_next_pos(robot_position[0],robot_position[1],robot_direction,m)
            print("finished run to goal")
        if (robot_position[0] == goal[0] and robot_position[1] == goal[1]):
            while not (robot_position[0] == 19 and robot_position[1] == 10):
                move_back_pos(robot_position[0],robot_position[1],robot_direction,maze_copy)
            print("finished run to start")
       
    def check_color():
        left_motor.setVelocity(1)  
        right_motor.setVelocity(-1)         
        turn_duration = 10
        start_time = robot.getTime()
        while robot.getTime() - start_time < turn_duration:
            robot.step(TIME_STEP)
            if goal_color()==colors[goal_number]:
                print(f"reach goal: {goal_color()}")
                continue

        left_motor.setVelocity(0)
        right_motor.setVelocity(0) 
        
        left_motor.setVelocity(-1)  
        right_motor.setVelocity(1)         
        turn_duration = 10
        start_time = robot.getTime()
        while robot.getTime() - start_time < turn_duration:
            robot.step(TIME_STEP)

        left_motor.setVelocity(0)
        right_motor.setVelocity(0)       
    
    def get_starting_direction(compass_values):
        if round(compass_values[0])==1:return 0
        elif round(compass_values[1])==1:return 1
        elif round(compass_values[0])==-1:return 2
        elif round(compass_values[1])==-1:return 3    
    
    
    def goal_color(cam):
        image = cam.getImage()
        width = cam.getWidth()
        height = cam.getHeight()
    
      
        image_array = np.frombuffer(image, dtype=np.uint8).reshape((height, width, 4))  # RGBA format
    
      
        center_region = image_array[height // 2 - 10:height // 2 + 10, width // 2 - 10:width // 2 + 10, :3]
        average_color = np.mean(center_region, axis=(0, 1))  # Compute average RGB values
        print(average_color)
        # Detect the color
        detected_color = detect_color(average_color)
        # print(f"Detected Color: {detected_color}")
           
        return detected_color  
        
        
    def move_next_cell(x,y,robot_direction):
        global child_cell
        array=[]
        for i, (dx, dy) in enumerate(reverse_direc):
            nx, ny = x + dx, y + dy
            if (0 <= nx < maze_size) and (0 <= ny < maze_size):
                if cells[x][y][3-i] == 1:
                    continue
                else:
                    child_cell = (nx,ny)
                    print(f"child cell is : {child_cell}")
                   
                    if child_cell in explored: continue         
                    else: 
                        array.append((dx,dy))
                        frontier.append(child_cell)
                        print(frontier)
                        explored.append(child_cell)
        # find_robot_direction(array[-1][0],array[-1][1])
        # print((array[-1][0],array[-1][1]))
                    
        if len(array) == 0:  # If no new cells available, BACKTRACK
            if len(frontier) > 0:
                backtrack_cell = frontier[-1]  # Get last unexplored node
                print(f"Backtracking to {backtrack_cell}")
                floodfill_again(backtrack_cell[0],backtrack_cell[1],m)
                while not (robot_position[0]==backtrack_cell[0] and robot_position[1]==backtrack_cell[1]):
                    wall_mapping(robot_position[0],robot_position[1])
                    floodfill_again(backtrack_cell[0],backtrack_cell[1],m)
                    move_next_pos(robot_position[0],robot_position[1],robot_direction,m)
                # find_robot_direction(backtrack_cell[0]-robot_position[0], backtrack_cell[1]-robot_position[1])
        else:
            find_robot_direction(array[-1][0], array[-1][1])
            print((array[-1][0], array[-1][1]))  
                                
      
      
    def check_survivors():
        # print(goal_color(camera_2))
        if goal_color(camera_2)=="Green" or goal_color(camera_3)=="Green" or goal_color(camera_4)=="Green" or goal_color(camera_5)=="Green":
            survivors.append((robot_position[0],robot_position[1])) 
            print("suvivor found")



    def check_goals_reached(x,y):
        global m
        global goal_number
        if goal_number<3:
            if robot_position[0] == goals[goal_number][0] and robot_position[1] == goals[goal_number][1]:
                print(f"Reached the goal! : {goal_number}")
                waiting()
                # check_color();
                goal_number+=1
                if goal_number<3:
                    m[goals[goal_number][0]][goals[goal_number][1]] = 0
                    floodfill_again(goals[goal_number][0],goals[goal_number][1],m)
                # motor_stop()
                if robot_position[0] == goals[-1][0] and robot_position[1] == goals[-1][1]:
                    motor_stop()
                    return True
           
              
        return False 
        
        
    def waiting():
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)  
        t= robot.getTime()
        while robot.getTime()-t<3:
            print("waiting !!!")
            robot.step(TIME_STEP)
            
    def fast_run_start():
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)  
        t= robot.getTime()
        while robot.getTime()-t<3:
            print("fast_run_start waiting !!!")
            robot.step(TIME_STEP)
        

    def block_red(color_map,survivors):
        global cells
        x =  [item for item in color_map if item not in survivors]
        for i in x:
            for j in range(4):
                cells[i[0]][i[1]][j]=1


    def go_to_start():
        while robot_position[0]!=initial_pos[0] or robot_position[1]!=initial_pos[1]:
            wall_mapping(robot_position[0],robot_position[1])
            cells[initial_pos[0]][initial_pos[1]][2]=1
            floodfill_again(initial_pos[0],initial_pos[1],m)
            move_next_pos(robot_position[0],robot_position[1],robot_direction,m)
        turnRight_3_75()
        return
        

    def update_damage_map(fire_pitches):
        for (x, y) in fire_pitches:
            GRID[x][y] = 999
            for dx, dy in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < maze_size and 0 <= ny < maze_size:
                    GRID[nx][ny] = 99    
                    
                    
    def floodfill_with_damage(x, y, Maze):
        for i in range(maze_size):
            for j in range(maze_size):
                Maze[i][j] = float('inf')
        Maze[x][y] = damage_map[x][y]
    
        queue = deque([(x, y)])
        while queue:
            cx, cy = queue.popleft()
            for i, (dx, dy) in enumerate(direc):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < maze_size and 0 <= ny < maze_size:
                    if cells[cx][cy][i] == 1:
                        continue
                    new_cost = Maze[cx][cy] + damage_map[nx][ny]
                    if new_cost < Maze[nx][ny]:
                        Maze[nx][ny] = new_cost
                        queue.append((nx, ny))                    

    def rescue_survivor(survivor):
        floodfill_with_damage(survivor[0], survivor[1], m)
        while not (robot_position[0] == survivor[0] and robot_position[1] == survivor[1]):
            move_next_pos(robot_position[0], robot_position[1], robot_direction, m)
        print(f"Rescued survivor at {survivor}")
        survivors.remove(survivor)    
                                    
  
    def heuristic(a, b):
        """Manhattan Distance Heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def a_star(start, goal, grid, cells):
        """A* Algorithm prioritizing safety over shortest distance"""
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        cost_so_far = {start: 0}
    
        while open_list:
            _, current = heapq.heappop(open_list)
    
            if current == goal:
                break
    
            for i, (dx, dy) in enumerate(direc):
                neighbor = (current[0] + dx, current[1] + dy)
    
                # Check if neighbor is within bounds
                if 0 <= neighbor[0] < len(grid) and 0 <= neighbor[1] < len(grid[0]):
    
                    # Check if there's a wall blocking movement
                    if cells[current[0]][current[1]][i] == 1:
                        continue  
    
                    # Assign cost based on danger levels
                    terrain_cost = grid[neighbor[0], neighbor[1]]
    
                    # Compute new cost
                    new_cost = cost_so_far[current] + terrain_cost
    
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic(neighbor, goal)
                        heapq.heappush(open_list, (priority, neighbor))
                        came_from[neighbor] = current
    
        return reconstruct_path(came_from, start, goal)
    
    def reconstruct_path(came_from, start, goal):
        """Reconstructs the Path from A*"""
        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from.get(current, start)
        path.append(start)
        path.reverse()
        return path            
      
    def rescue_mission(goal):
        path = a_star(tuple(robot_position), goal, GRID, cells)
        print(path)
        for i in path:
            print(i)
            curr_x,curr_y=robot_position[0],robot_position[1]
            find_robot_direction(i[0]-curr_x,i[1]-curr_y)
            
                                
    # floodfill(goal[0],goal[1])
    # floodfill(goal[0],goal[1])
   
    while robot.step(TIME_STEP) != -1:
        # if i==0:        # because i want run this only once
            # moveForward_3_75()
            # gps_values = gps.getValues()
            # compass_values = compass.getValues()
            
            # robot_position = list(get_starting_pos(gps_values))
            # robot_direction = get_starting_direction(compass_values)
            
            # frontier.append(tuple(robot_position))
            # explored.append(tuple(robot_position))  
            
            # i+=1
            
            # print(robot_position)
            # print(robot_direction)
            
            # initial_pos = robot_position
            

            

            
        # x,y = robot_position
        
        # if j==0:  # because i want run this only once

            # while len(explored)<400:   
                # x,y = robot_position
                # print(f"explored list length is : {len(explored)}")
                # wall_mapping(x,y)
                # cells[initial_pos[0]][initial_pos[1]][2]=1 
                # print(cells)
                # if len(frontier) > 0:
                    # current_cell = frontier.pop()
                    # robot_position = list(current_cell)
                    # print(robot_direction)
                    # if goal_color(camera)=="Red":
                        # print("Red Square")
                        # color_map.append((robot_position[0], robot_position[1]))
                    # move_next_cell(robot_position[0], robot_position[1], robot_direction)
                    # print(cells)
                    # check_survivors()
            # print(f"Fire Pitches are : {color_map}")
            # print(f"Survivors are : {survivors}")
            # update_damage_map(color_map)
            
            # print(robot_position)
            # print(cells)
            
            # wall_mapping(x,y)
            # cells[initial_pos[0]][initial_pos[1]][2]=1 
            # go_to_start();
            # print("dry run end !!!")
            # fast_run_start();
            # print("fast run started !!!")
            # turnBack_3_75()
            # robot_direction=0
            # j+=1
            
        # print(cells)    
        # rescue_mission(survivors[goal_number])
        # waiting()   
        # goal_number+=1
        # if goal_number==3:
            # rescue_mission((initial_pos[0],initial_pos[1]))
            # go_to_start();
            # break
            
        left_motor.setVelocity(0.5)  
        right_motor.setVelocity(1)  