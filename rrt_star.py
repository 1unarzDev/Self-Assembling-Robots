import sys, pygame, math
from pygame import *
import numpy as np
import cv2 as cv
from video import *
import motors
import geopandas as gpd
from shapely.geometry import Point, Polygon
import time

class Node(object):
    def __init__(self, point, parent, cost=0, ID=None):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent
        self.cost = cost
        self.ID = ID

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

obs, centers_x, centers_y, corners, ids, angles, YDIM, XDIM = video_detection(cap)

windowSize = [XDIM, YDIM]
delta = 20.0
GAME_LEVEL = 1
GOAL_RADIUS = 10
MIN_DISTANCE_TO_ADD = 1.0
NUMNODES = 20000
NUMOPTIMIZATION = 200
FPS = 1000
pygame.init()
fpsClock = pygame.time.Clock()
screen = pygame.display.set_mode(windowSize)
white = 255, 255, 255
black = 25, 25, 25
light_red = 255, 220, 220
dark_red = 150, 0, 0
red = 255, 0, 0
blue = 0, 255, 0
green = 0, 0, 255
cyan = 0,180,105
purple = (100, 0, 255)

count = 0

robot_length = 225
robot_width = 206.5
wheelbase = 156
marker_length = 59.5
marker_to_center_distance = 14

def get_length_ratio():
        obs, centers_x, centers_y, corners, ids, angles, YDIM, XDIM = video_detection(cap)

        aruco_side_length = find_marker_side_length(corners[0][0])
        length_ratio = aruco_side_length/marker_length

        return length_ratio

def get_orientation():
    obs, centers_x, centers_y, corners, ids, angles, YDIM, XDIM = video_detection(cap)
    angle = angles[0] - 45
    return angle

def dist(p1,p2):    # Distance between two points
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def point_circle_collision(p1, p2, radius):
    distance = dist(p1,p2)
    return distance <= radius

def step_from_to(p1,p2):
    if dist(p1,p2) < delta:
        return p2
    else:
        theta = np.arctan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + delta*np.cos(theta), p1[1] + delta*np.sin(theta)

def point_inside_polygon(point, polygon):
    x, y = point
    n = len(polygon)
    inside = False
    for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i+1) % n]
            if min(y1, y2) < y <= max(y1, y2) and x <= max(x1, x2):
                if y1 != y2:
                    x_intersect = (y - y1) * (x2 - x1) / (y2 - y1) +x1
                if x1 == x2 or x <= x_intersect:
                    inside = not inside
    
    return inside

def line_inside_polygon(line, polygon):
    p1, p2 = line
    for i in range(len(polygon)):
        p3, p4 = polygon[i], polygon[(i + 1) % len(polygon)]
        if do_lines_intersect(p1, p2, p3, p4):
             return True
    return False

def do_lines_intersect(p1, p2, p3, p4):
    o1 = orientation(p1, p2, p3)
    o2 = orientation(p1, p2, p4)
    o3 = orientation(p3, p4, p1)
    o4 = orientation(p3, p4, p2)

    if o1 != o2 and o3 != o4:
        return True
    
    if o1 == 0 and is_on_segment(p1, p3, p2):
        return True
    if o2 == 0 and is_on_segment(p1, p4, p2):
        return True
    if o3 == 0 and is_on_segment(p3, p1, p4):
        return True
    if o4 == 0 and is_on_segment(p3, p2, p4):
        return True
    
    return False

def orientation(p, q, r):
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        return 0
    return 1 if val > 0 else 2

def is_on_segment(p, q, r):
    return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])) 

def lineCollides(p1, p2, goal=None, test=None):

    if obs == None:
        return False

    p1_array = np.array([])
    p2_array = np.array([])

    p1_array = np.append(p1_array, p1)
    p2_array = np.append(p2_array, p2)

    line = [p1_array, p2_array]

    if (goal != None):
        radius = goal[1]
        circle_origin = goal[0]

        dist1 = dist(p1, circle_origin)
        dist2 = dist(p2, circle_origin)

        if dist1 < radius and dist2 < radius: 
            return True

        if p2[0] - p1[0] == 0: 
            m = None
            b = p1[0]
        else:
            m = ((p2[1]-p1[1])/(p2[0]-p1[0]))
            b = p1[1] - m * p1[0]

        def equation(x):
            return m*x+b 

        if test != None:
            for x in range(0, XDIM):
                y = equation(x)
                pygame.draw.line(screen, red, (x, y), (x + 1, y + 1))

            print(f"M:{m} and B:{b}")

        distance_to_line = abs(circle_origin[1] - m * circle_origin[0] - b) / np.sqrt(m**2+1)

        y_min_x = -(b/m)
        y_max_x = (YDIM-b)/m

        for polygon in obs:
            if line_inside_polygon([[y_min_x, 0],[y_max_x, YDIM]], polygon):
                return False
        return distance_to_line <= radius 

    for polygon in obs:
        if line_inside_polygon(line, polygon):
            return True
    return False    

def pointCollides(p1):

    if obs == None:
        return False

    for polygon in obs:
        if point_inside_polygon(p1, polygon):
            return True
    return False       

def calculate_cost(node):
    if node.parent is None:
        cost = float('inf')
    else:
        cost = 0

    while node.parent is not None:
        cost += dist(node.parent.point, node.point)
        node = node.parent
    return cost

def find_most_optimal_branch(nodes):
    best_node = None
    best_cost = float('inf')
    for node in nodes:
        if node.parent is not None:
            if point_circle_collision(node.point, goalPoint.point, GOAL_RADIUS) and not lineCollides(goalPoint.point,node.point):    
                cost = calculate_cost(node)
                if cost < best_cost:
                    best_node = node
                    best_cost = cost

    return best_node

def check_straight_line_path(goal, nodes):
    if len(nodes) == 0:
        return False  # No nodes in the tree yet

    last_added_node = nodes[-1]  # Get the last added node

    if not lineCollides(last_added_node.point, goal.point):
        return True  # Straight line path is clear

    return False

def daughterless_nodes(nodes):

    global parentNodes, daughterlessNodes
    parentNodes = np.array([])
    daughterlessNodes = np.array([])

    for node in nodes:
        if node.ID != None and node.parent != None:
            parentNodes = np.append(parentNodes, node.parent.ID)

    for node in nodes: 
        if node.ID != None and node.parent != None and node.ID not in parentNodes:
            daughterlessNodes = np.append(daughterlessNodes, node)
                 
def find_marker_side_length(corners):
    if len(corners) != 4:
        raise ValueError("The corners list must contain exactly 4 points.")
    
    # Calculate the distances between corners
    distances = []
    for i in range(len(corners)):
        for j in range(i + 1, len(corners)):
            dist = np.linalg.norm(np.array(corners[i]) - np.array(corners[j]))
            distances.append(dist)
    
    # Find the minimum distance (assuming opposite corners have the longest distance)
    side_length = min(distances)
    
    return side_length

def add_point_along_path(bottomRight, center, distance):
    # Calculate the vector from center to bottomRight
    vector_to_bottom_right = np.array(bottomRight) - np.array(center)
    
    # Normalize the vector
    normalized_vector = vector_to_bottom_right / np.linalg.norm(vector_to_bottom_right)
    
    # Calculate the coordinates of the new point
    new_point = np.array(bottomRight) + distance * normalized_vector
    
    # Return the coordinates of the new point
    return tuple(new_point)

def get_random_clear():
    while True:
        p = np.random.rand()*XDIM, np.random.rand()*YDIM
        noCollision = pointCollides(p)
        if noCollision == False:
            return p

def init_obstacles(): 

    if obs == None:
        return

    for polygon in obs:
        polygon_tuple = []

        for point in polygon:
            polygon_tuple.append((int(point[0]),int(point[1])))

        pygame.draw.polygon(screen, light_red, polygon_tuple)

    for polygon in original_obstacles:
        polygon_tuple = []

        for point in polygon:
            polygon_tuple.append((int(point[0]),int(point[1])))

        pygame.draw.polygon(screen, black, polygon_tuple)

def define_start(start_pos):
    global nodes

    nodes = []

    initialPoint = Node(start_pos, None)
    nodes.append(initialPoint) # Start in the center
    pygame.draw.circle(screen, red, initialPoint.point, GOAL_RADIUS)
    pygame.draw.circle(screen, dark_red, (centers_x[0], centers_y[0]), GOAL_RADIUS/3)

def reset():

    global path, unoptimizedPath, goal_found, foundNext, path_made, count, additional_nodes, goalPoint, straightLineOptimize, ID, optimization_nodes, original_obstacles

    global obs, centers_x, centers_y, corners, ids, angles, YDIM, XDIM

    move_robot = 0

    while move_robot <= 1:

        obs, centers_x, centers_y, corners, ids, angles, YDIM, XDIM = video_detection(cap)

        if obs != None:
            original_obstacles = obs.copy()

            for i in range(len(obs)):
                polygon = Polygon(obs[i])
                
                buffer = polygon.buffer(robot_width/2)

                buffer = np.array(buffer.exterior.coords.xy).T

                obs[i] = buffer

        aruco_side_length = find_marker_side_length(corners[0][0])
        length_ratio = aruco_side_length/marker_length

        (topLeft, topRight, bottomRight, bottomLeft) = corners[0][0]
        center = (centers_x[0], centers_y[0])

        robot_center = add_point_along_path(bottomRight, center, marker_to_center_distance*length_ratio)

        screen.fill(white)
        init_obstacles()
            
        if pointCollides(robot_center) == False:
            move_robot += 1            
        else:
            pygame.display.set_caption('MOVE THE ROBOT')

    define_start(robot_center)

    count, additional_nodes, ID = (0,)*3
    optimization_nodes = NUMOPTIMIZATION
    path = np.array([]).reshape(0,2)
    unoptimizedPath = np.array([]).reshape(0,2)

    goalPoint = Node(None, None)

    straightLineOptimize, goal_found, foundNext, path_made = (False,)*4


def main():
   
    global path, unoptimizedPath, goal_found, foundNext, path_made, count, additional_nodes, goalPoint, straightLineOptimize, ID, optimization_nodes

    optimization_nodes = NUMOPTIMIZATION
    initialPoint = Node(None, None)
    goalPoint = Node(None, None)
    currentState = 'init'

    count, additional_nodes, ID = (0,)*3
    path = np.array([]).reshape(0,2)
    unoptimizedPath = np.array([]).reshape(0,2)
    straightLineOptimize, goal_found, foundNext, path_made = (False,)*4

    use_path = False

    reset()

    while use_path == False:

        if currentState == 'init':
            pygame.display.set_caption('Select Goal Point')
            fpsClock.tick(0)
        elif currentState == 'optimalPathFound' and not path_made:            
            currNode = goalNode
            pygame.display.set_caption('Goal Reached')
            
            while currNode.parent != None:
                pygame.draw.line(screen,red,currNode.point,currNode.parent.point, 5)
                path = np.vstack([path, list(currNode.point)])
                currNode = currNode.parent

            path = np.vstack([path, list(currNode.point)])

            for i in range(len(unoptimizedPath) - 1):
                pygame.draw.line(screen, purple, unoptimizedPath[i], unoptimizedPath[i + 1], 5)

            path_made = True
                
        elif currentState == 'buildTree':
            pygame.display.set_caption('Performing RRT')

            if count < NUMNODES + optimization_nodes:
                foundNext = False
                straightLineOptimize = False

                if check_straight_line_path(goalPoint, nodes) and not goal_found:                       
                    parentNode = nodes[0]
                    for p in nodes:
                        if dist(p.point, goalPoint.point) <= dist(parentNode.point, goalPoint.point):
                            if lineCollides(goalPoint.point,p.point) == False and pointCollides(goalPoint.point) == False:
                                parentNode = p
                                newnode = goalPoint.point
                                foundNext = True
                                straightLineOptimize = True

                                if(count == 0):
                                    goal_found = True
                                    additional_nodes = optimization_nodes + 1

                while foundNext == False:
                    rand = get_random_clear()
                    parentNode = nodes[0]
                    for p in nodes:
                        if dist(p.point,rand) <= dist(parentNode.point,rand):
                            newPoint = step_from_to(p.point,rand)
                            if not lineCollides(newPoint,p.point) and not pointCollides(newPoint):
                                parentNode = p
                                newnode = newPoint
                                foundNext = True

                if not straightLineOptimize and dist(parentNode.point, newnode) <= GOAL_RADIUS:
                    foundNext = False
                    continue
                for p in nodes:
                    if dist(p.point,newnode) + p.cost <= dist(parentNode.point,newnode) + parentNode.cost:
                            if lineCollides(newnode,p.point) == False and pointCollides(newnode) == False:
                                parentNode = p

                nodes.append(Node(newnode, parentNode, parentNode.cost+dist(newnode,parentNode.point), ID))
                ID += 1

                if (point_circle_collision(newnode, goalPoint.point, GOAL_RADIUS) or lineCollides(newnode, parentNode.point, (goalPoint.point, GOAL_RADIUS))):
                    goal_found = True 

                if goal_found:
                    if  additional_nodes == 0:
                        unoptimizedGoalNode = goalPoint
                        unoptimizedGoalNode.parent = parentNode
                        unoptimizedGoalNode.cost = parentNode.cost + dist(unoptimizedGoalNode.point, parentNode.point)
                        unoptimizedCurrNode = unoptimizedGoalNode

                        while unoptimizedCurrNode.parent != None:
                            pygame.draw.line(screen, purple, unoptimizedCurrNode.point, unoptimizedCurrNode.parent.point)
                            unoptimizedPath = np.vstack([unoptimizedPath, list(unoptimizedCurrNode.point)])
                            unoptimizedCurrNode = unoptimizedCurrNode.parent

                            unoptimizedPath = np.vstack([unoptimizedPath, list(unoptimizedCurrNode.point)])

                        additional_nodes +=1

                    elif (additional_nodes > optimization_nodes):
                        currentState = 'optimalPathFound'   

                        daughterless_nodes(nodes)

                        for node in daughterlessNodes:
                            if not lineCollides(goalPoint.point,node.point):
                                nodes.append(Node(goalPoint.point, node, node.point+dist(node.point,goalPoint.point)))

                        goalNode = find_most_optimal_branch(nodes)                   
                    else:
                        additional_nodes += 1
                else:
                    count += 1
                
                pygame.draw.circle(screen, blue, newnode, 2)
                pygame.draw.line(screen,cyan,parentNode.point,newnode)
                
            else:
                print("Ran out of nodes... :(")
                return

        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                print(path)
                print(f"Path: {path}")
                print(f"\n\nUnoptimized Path: {unoptimizedPath}")
                cap.release()
                sys.exit("Exiting")
            if e.type == MOUSEBUTTONDOWN:
                print('mouse down')
                if currentState == 'init':
                    print('goal point set: '+str(e.pos))
                    if pointCollides(e.pos) == False:
                        goalPoint = Node(e.pos,None)
                        pygame.draw.circle(screen, green, goalPoint.point, GOAL_RADIUS)
                        currentState = 'buildTree'
                else:
                    currentState = 'init'
                    reset()
            if e.type == KEYUP and e.key == K_SPACE:
                if path_made:
                    use_path = True
                else:
                    print("Optimization Over")
                    optimization_nodes = 0


        pygame.display.update()
        fpsClock.tick(FPS)

if __name__ == '__main__':

    while True:
        main()

        """
        While the code below could kinda work,
        I still need to find a way to handle/calculate 
        - Robot size
        - Robot center position
        - Robot off path position
        - Robot wheelbase
        - Robot max speed
        
        """

        """
        Just a few notes for writing the rest of this script:
        - The robot is 225mm long
        - The wheelbase of the robot is 156mm
        - The wheel diameter is 59mm
        - The marker is 59.5mm long
        - If you can take the length of the marker in px, then you can
        do markerpx/59.5 = robotpx/225, so robotpx = 225 * markerpx / 59.5
        - The robot is 206.5mm wide including the axles
        - Since the length and width are similar, you can just a 112.5 radius
        increase in size along every side of each polygon
        - The bottom right corner of the marker is about 14mm from the center of the robot 

        """

        obs, centers_x, centers_y, corners, ids, angles, YDIM, XDIM = video_detection(cap)

        aruco_side_length = find_marker_side_length(corners[0][0])
        length_ratio = aruco_side_length/marker_length

        (topLeft, topRight, bottomRight, bottomLeft) = corners[0][0]
        center = (centers_x[0], centers_y[0])

        robot_center = add_point_along_path(bottomRight, center, marker_to_center_distance*length_ratio)    

        print(f"Path: {path}")

        print(f"Unoptimized Path: {unoptimizedPath}")

        print(f"Start: {robot_center} Goal: {goalPoint}")

        motors.motor_movements(path, robot_center[0], robot_center[1], wheelbase*length_ratio, XDIM, YDIM, get_orientation())

        obs, centers_x, centers_y, corners, ids, angles, YDIM, XDIM = video_detection(cap)

        aruco_side_length = find_marker_side_length(corners[0][0])
        length_ratio = aruco_side_length/marker_length

        (topLeft, topRight, bottomRight, bottomLeft) = corners[0][0]
        center = (centers_x[0], centers_y[0])

        new_robot_center = add_point_along_path(bottomRight, center, marker_to_center_distance*length_ratio)

        error = dist(goalPoint.point, new_robot_center) 

        print(f"Error: {error}")