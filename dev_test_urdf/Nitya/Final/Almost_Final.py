#!/usr/bin/env python
# coding: utf-8

# In[1]:


import pybullet as p
import pybullet_data
import time
import numpy as np
import cvxpy as cp
import threading
import random
import tkinter as tk
from tkinter import messagebox
from tkinter import Tk, Label, IntVar, Scale, HORIZONTAL
from tkinter import Tk, Canvas, Label
import xml.etree.ElementTree as ET
from scipy.spatial import cKDTree
import heapq
from tkinter import *
import matplotlib.pyplot as plt
import math


# In[2]:


##Global Variables
sphere_ids = []
N = 60
m = 3.0  # mass of the drone
g = 9.81  # gravitational acceleration
gravity_acceleration = np.array([0, 0, -g])
max_thrust_z = 80.0 # Maximum thrust in z (vertical) direction in N
max_thrust_xy = 40.0 # Maximum thrust in x and y directions in N
max_total_thrust = 80.0 # Maximum total thrust in N (sum of x, y, z)
k_drag_per_m = np.array([0.5, 0.5, 0.8]) / m
global_mpc_value = np.zeros(3)
buffer = 0.25
a_start_buffer = 0.35
mpc_buffer = 0.25
threshold_mpc = 0.25
N = 15
freq = 15
rrt_max_iterations = 5000
cell_size = 0.25
merge_threshold = 0.5
space_outside_building = 1
position_noise_std = 0.02
velocity_noise_std = 0.03
ROOM_URDF_PATH = "assets/room.urdf"
DRONE_URDF_PATH = "assets/cf2x.urdf"
ROOM_HEIGHT = 2.0
target_x_relative = 1.5
target_y_relative = 1.5
target_z_relative = 1.5
highlight_position = None
num_rows=2    # rows
num_cols=2  # columns
spacing=7.5  # distance between buildings
floors_min=3
floors_max=3
startPos = [0, -0.5, 2.15]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
white_alpha = 0.3
red_alpha = 0.3
sphere_id = None
droneId = None
mins = None
maxs = None
merged_mins = None
merged_maxs = None
map_indices = None
bounds = None
physicsClient = None
top_floor_id = None
default_a_start_adjusted_min = None
default_a_start_adjusted_max = None
default_adjusted_min = None
default_adjusted_max = None
graph = None
buildings = None
start_position_to_repeat = None
goal_position_to_repeat = None


# In[3]:


def parse_collision_boxes(urdf_file):
    # Parse the URDF file
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    boxes = []
    
    for link in root.findall("link"):
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            box = geometry.find("box")
            if box is not None:
                size = list(map(float, box.attrib["size"].split()))
                origin_elem = collision.find("origin")
                if origin_elem is not None:
                    origin = list(map(float, origin_elem.attrib["xyz"].split()))
                    rpy = list(map(float, origin_elem.attrib["rpy"].split()))
                else:
                    origin = [0.0, 0.0, 0.0]
                    rpy = [0.0, 0.0, 0.0]
                boxes.append((size, origin, rpy))
    return boxes


# In[4]:


def generate_obstacles(num_floors, urdf_file, floor_height, rotation_step=90, building_base=[0, 0, 0]):
    collision_boxes = parse_collision_boxes(urdf_file)
    all_obstacles = []
    mins = []
    maxs = []

    for floor in range(num_floors):
        # print("[DEBUG] Floor:", floor)
        floor_yaw = math.radians(rotation_step * floor)
        floor_rotation_rpy = [0.0, 0.0, floor_yaw]
        R_floor = euler_to_rotation_matrix(floor_rotation_rpy)
        # print("[DEBUG] R_floor:\n", R_floor)
        floor_z = floor * floor_height

        for size, origin, rpy in collision_boxes:
            R_obs = euler_to_rotation_matrix(rpy)
            # print("[DEBUG] R_obs:\n", R_obs)
            
            # size = [sx, sy, sz]
            sx, sy, sz = size
            hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0

            # Local corners of the box (centered at [0,0,0])
            local_corners = np.array([
                [-hx, -hy, -hz],
                [-hx, -hy,  hz],
                [-hx,  hy, -hz],
                [-hx,  hy,  hz],
                [ hx, -hy, -hz],
                [ hx, -hy,  hz],
                [ hx,  hy, -hz],
                [ hx,  hy,  hz],
            ])
            # print("[DEBUG] Local_corners:\n", local_corners)

            # Apply obstacle rotation and translation: X_room = R_obs*X_local + origin
            room_corners = np.dot(local_corners, R_obs.T) + np.array(origin)
            # print("[DEBUG] room_corners:\n", room_corners)

            # Apply floor rotation and building base translation
            # X_world = building_base + [0,0,floor_z] + R_floor * X_room
            world_corners = np.dot(room_corners, R_floor.T) + np.array([building_base[0], building_base[1], building_base[2] + floor_z])
            # print("[DEBUG] world_corners:\n", world_corners)

            min_coords = world_corners.min(axis=0)
            max_coords = world_corners.max(axis=0)
            mins.append(min_coords)
            maxs.append(max_coords)

            final_center = 0.5 * (min_coords + max_coords)
            final_size = max_coords - min_coords

            all_obstacles.append({
                "center": final_center.tolist(),
                "size": final_size.tolist()
            })

    return all_obstacles, mins, maxs


# In[5]:


def euler_to_rotation_matrix(rpy):
    roll, pitch, yaw = rpy
    cx, cy, cz = math.cos(roll), math.cos(pitch), math.cos(yaw)
    sx, sy, sz = math.sin(roll), math.sin(pitch), math.sin(yaw)
    R = np.array([
        [cy*cz, cz*sx*sy - sz*cx, cz*cx*sy + sz*sx],
        [cy*sz, sz*sx*sy + cz*cx, sz*cx*sy - cz*sx],
        [-sy,   cy*sx,            cy*cx]
    ])
    return R


# In[6]:


def generate_grid_buildings(num_rows=4, num_cols=5, spacing=10.0, floors_min=3, floors_max=5):
    """
    Generate a grid of building positions and floor counts. 
    Returns a list of (x, y, num_floors).
    """
    num_buildings = num_rows * num_cols
    x_offset = (num_cols - 1) * spacing / 2.0
    y_offset = (num_rows - 1) * spacing / 2.0
    buildings = []

    for row in range(num_rows):
        for col in range(num_cols):
            building_x = col * spacing - x_offset
            building_y = row * spacing - y_offset
            num_floors = random.randint(floors_min, floors_max)
            buildings.append((building_x, building_y, num_floors))

    return buildings


# In[7]:


def create_buildings_and_obstacles(buildings,
                                   room_urdf_path,
                                   room_height,
                                   pybullet_client_id):
    """
    For each building in 'buildings', load the URDF floors and generate obstacles.
    Return a list of obstacle sets, one for each building.
    """
    global white_alpha
    all_buildings_obstacles = []
    all_mins = []
    all_maxs = []

    for (building_x, building_y, num_floors) in buildings:
        # Create the URDF floors
        for floor in range(num_floors):
            floor_z = floor * room_height
            yaw_degrees = floor * 90
            room_orientation = p.getQuaternionFromEuler([0, 0, math.radians(yaw_degrees)])
            room_id = p.loadURDF(
                room_urdf_path,
                [building_x, building_y, floor_z],
                room_orientation,
                useFixedBase=True,
                physicsClientId=pybullet_client_id
            )
            # Make them semi-transparent
            visual_shapes = p.getVisualShapeData(room_id, physicsClientId=pybullet_client_id)
            for shape in visual_shapes:
                shape_index = shape[1]
                p.changeVisualShape(
                    room_id,
                    shape_index,
                    rgbaColor=[1, 1, 1, white_alpha],
                    physicsClientId=pybullet_client_id
                )

        # Generate obstacles for this building
        obstacles, mins, maxs = generate_obstacles(
            num_floors=num_floors,
            urdf_file=room_urdf_path,
            floor_height=room_height,
            rotation_step=90,
            building_base=[building_x, building_y, 0]
        )
        all_buildings_obstacles.append(obstacles)
        all_mins += mins
        all_maxs += maxs

    return all_buildings_obstacles, all_mins, all_maxs


# In[8]:


def highlight_random_floor(buildings,
                              room_urdf_path,
                              room_height,
                              pybullet_client_id):
    """
    Pick a random building from 'buildings', highlight its top floor in red,
    and return the index of that chosen building.
    """
    global red_alpha
    chosen_building_idx = random.randint(0, len(buildings) - 1)
    b_x, b_y, b_floors = buildings[chosen_building_idx]

    chosen_floor_idx = random.randint(0, b_floors - 1)

    # Compute top floor orientation
    yaw_degrees = (chosen_floor_idx) * 90
    top_floor_z = (chosen_floor_idx) * room_height
    top_floor_orientation = p.getQuaternionFromEuler([0, 0, math.radians(yaw_degrees)])

    # Load the top floor URDF again, only for highlighting
    top_floor_id = p.loadURDF(
        room_urdf_path,
        [b_x, b_y, top_floor_z],
        top_floor_orientation,
        useFixedBase=True,
        physicsClientId=pybullet_client_id
    )
    # Make top floor red
    top_visual_shapes = p.getVisualShapeData(top_floor_id, physicsClientId=pybullet_client_id)
    for shape in top_visual_shapes:
        shape_index = shape[1]
        p.changeVisualShape(
            top_floor_id,
            shape_index,
            rgbaColor=[1, 0, 0, red_alpha],
            physicsClientId=pybullet_client_id
        )
    return [b_x, b_y, top_floor_z], top_floor_id


# In[9]:


def dehighlight_floor(top_floor_id, pybullet_client_id):
    global white_alpha
    top_visual_shapes = p.getVisualShapeData(top_floor_id, physicsClientId=pybullet_client_id)
    for shape in top_visual_shapes:
        shape_index = shape[1]
        p.changeVisualShape(
            top_floor_id,
            shape_index,
            rgbaColor=[1, 1, 1, white_alpha],
            physicsClientId=pybullet_client_id
        )


# In[10]:


def merge_bounding_boxes(mins, maxs, threshold):
    N = len(mins)

    def can_merge(box1, box2):
        """Check if two boxes can be merged based on the threshold."""
        for axis in range(3):  # Check x, y, z axes
            if box1[1][axis] < box2[0][axis] - threshold or box2[1][axis] < box1[0][axis] - threshold:
                return False
        return True

    def merge_two_boxes(box1, box2):
        """Merge two boxes into a single box."""
        merged_min = np.minimum(box1[0], box2[0])
        merged_max = np.maximum(box1[1], box2[1])
        return (merged_min, merged_max)

    def merge_boxes(mins, maxs):
        """Merge boxes iteratively until no more merges are possible."""
        boxes = [(np.array(mins[i]), np.array(maxs[i])) for i in range(len(mins))]
        merged = True

        while merged:
            merged = False
            new_boxes = []
            visited = [False] * len(boxes)

            for i in range(len(boxes)):
                if visited[i]:
                    continue

                current_box = boxes[i]
                visited[i] = True

                for j in range(i + 1, len(boxes)):
                    if visited[j]:
                        continue

                    if can_merge(current_box, boxes[j]):
                        current_box = merge_two_boxes(current_box, boxes[j])
                        visited[j] = True
                        merged = True

                new_boxes.append(current_box)

            boxes = new_boxes

        return boxes

    merged_boxes = merge_boxes(mins, maxs)

    # Split the merged boxes back into mins and maxs
    merged_mins = [box[0] for box in merged_boxes]
    merged_maxs = [box[1] for box in merged_boxes]

    # Map original boxes to merged boxes
    map_indices = []
    for min_box, max_box in zip(mins, maxs):
        for i, merged_box in enumerate(merged_boxes):
            if all(min_box >= merged_box[0]) and all(max_box <= merged_box[1]):
                map_indices.append(i)
                break

    # # Print details
    # print("Original number of boxes:", N)
    # print("Final number of boxes:", len(merged_mins))

    return merged_mins, merged_maxs, map_indices


# In[11]:


def is_point_in_cuboid(point, min_corner, max_corner):
    """
    Check if a point is inside an axis-aligned cuboid.
    """
    return all(min_corner[i] <= point[i] <= max_corner[i] for i in range(3))

def does_line_segment_intersect_cuboid(p1, p2, min_corner, max_corner):
    """
    Efficiently checks if a line segment intersects an axis-aligned cuboid (AABB).
    """
    d = p2 - p1  # Direction vector of the line segment
    t_min, t_max = 0, 1  # Parametric range for the segment

    for axis in range(3):  # Iterate over x, y, z axes
        if d[axis] != 0:  # The segment is not parallel to this axis
            t1 = (min_corner[axis] - p1[axis]) / d[axis]
            t2 = (max_corner[axis] - p1[axis]) / d[axis]

            t_near = min(t1, t2)
            t_far = max(t1, t2)

            t_min = max(t_min, t_near)
            t_max = min(t_max, t_far)

            if t_min > t_max:  # The segment exits before entering
                return False
        else:  # The segment is parallel to this axis
            if p1[axis] < min_corner[axis] or p1[axis] > max_corner[axis]:
                return False  # The segment is outside the slab on this axis

    return True  # The segment intersects the cuboid


# In[12]:


def merge_irrelevant_boxes(start_point, goal_point, mins, maxs, merged_mins, merged_maxs, map_indices):
    merges_bools = [] #True means expand
    final_mins = []
    final_maxs = []
    for i in range(len(merged_mins)):
        merges_bools.append(is_point_in_cuboid(start_point, merged_mins[i], merged_maxs[i]) or is_point_in_cuboid(goal_point, merged_mins[i], merged_maxs[i]))

    for i in range(len(mins)):
        if merges_bools[map_indices[i]]:
            final_mins.append(mins[i])
            final_maxs.append(maxs[i])

    for i in range(len(merges_bools)):
        if not merges_bools[i]:
            final_mins.append(merged_mins[i])
            final_maxs.append(merged_maxs[i])

    return final_mins, final_maxs


# In[13]:


def find_bounds(mins, maxs, buffer, space_outside_building):
    global_min = np.array([float('inf'), float('inf'), float('inf')])
    global_max = np.array([-float('inf'), -float('inf'), -float('inf')])

    for i in range(len(mins)):    
        global_min = np.minimum(global_min, mins[i])
        global_max = np.maximum(global_max, maxs[i])
    
    bounds = np.vstack((global_min, global_max)).T
    bounds[0, 0] -= space_outside_building
    bounds[0, 1] += space_outside_building
    bounds[1, 0] -= space_outside_building
    bounds[1, 1] += space_outside_building    
    bounds[2, 0] += buffer
    #bounds[2, 1] += space_outside_building  #Please comment after adding windows
    bounds[2, 1] -= buffer   #Please uncomment after addding windows
    
    return bounds


# In[14]:


def adjusted_min_max(final_mins, final_maxs, buffer):
    adjusted_min = np.array(final_mins) - buffer
    adjusted_max = np.array(final_maxs) + buffer
    return adjusted_min, adjusted_max


# In[15]:


def check_collision(point, adjusted_min, adjusted_max):
    for i in range(len(adjusted_min)):
        if is_point_in_cuboid(point, adjusted_min[i], adjusted_max[i]):
            return True
    return False


# In[16]:


def check_collision_line(point1, point2, adjusted_min, adjusted_max):
    for i in range(len(adjusted_min)):
        if does_line_segment_intersect_cuboid(point1, point2, adjusted_min[i], adjusted_max[i]):
            return True
    return False


# In[17]:


def update_sphere_color(sphere_id, sphere_position, adjusted_min, adjusted_max):
    # Condition: Change color if the sphere's x position is greater than 1
    if check_collision(sphere_position, adjusted_min, adjusted_max):
        # Set color to red
        p.changeVisualShape(sphere_id, -1, rgbaColor=[1, 0, 0, 1])
    else:
        # Set color to green
        p.changeVisualShape(sphere_id, -1, rgbaColor=[0, 1, 0, 1])


# In[18]:


def euclidean_distance(point1, point2):
    point1 = np.array(point1)  # Convert to numpy array if not already
    point2 = np.array(point2)  # Convert to numpy array if not already
    return np.linalg.norm(point1 - point2)


# In[19]:


def find_node_grid(bounds, cell_size):
    rows = int(np.ceil((bounds[1, 1] - bounds[1, 0]) / cell_size))
    cols = int(np.ceil((bounds[0, 1] - bounds[0, 0]) / cell_size))
    depths = int(np.ceil((bounds[2, 1] - bounds[2, 0]) / cell_size)) 
    
    # Create a 2D grid of coordinates
    node_grid = np.zeros((rows, cols, depths, 3))  
    
    # Fill the grid with node coordinates
    for i in range(rows):  # Y-axis
        for j in range(cols):  # X-axis
            for k in range(depths):  # Z-axis
                x = bounds[0, 0] + j * cell_size
                y = bounds[1, 0] + i * cell_size
                z = bounds[2, 0] + k * cell_size
                node_grid[i, j, k] = [x, y, z]

    return node_grid


# In[20]:


def find_collision_free_array(node_grid, adjusted_min, adjusted_max):
    collision_free_nodes = []
    for coord in node_grid.reshape(-1, 3):  # Flatten to a (N, 3) array
        if not check_collision(coord, adjusted_min, adjusted_max):  # If not in collision
            collision_free_nodes.append(tuple(coord))
    
    # Convert to NumPy array and round coordinates
    collision_free_array = np.array(collision_free_nodes)
    collision_free_array = np.round(collision_free_array, 3)
    
    return collision_free_array


# In[21]:


def build_graph(collision_free_array, cell_size):
    """
    Builds a graph from collision-free nodes using KD-tree for neighbor search.

    Args:
        collision_free_array (np.ndarray): Array of collision-free nodes (N, 3).
        cell_size (float): The size of the grid cells.

    Returns:
        dict: Graph with nodes as keys and neighbors as values.
    """
    graph = {}
    kd_tree = cKDTree(collision_free_array)
    radius = np.sqrt(3 * cell_size**2) + 0.001  # Neighbor search radius

    for node in collision_free_array:
        neighbors = []
        # Find all neighbors within the radius
        indices = kd_tree.query_ball_point(node, radius)
        for idx in indices:
            neighbor = collision_free_array[idx]
            if not np.array_equal(node, neighbor):
                distance = np.linalg.norm(node - neighbor)
                neighbors.append((tuple(neighbor), distance))
        graph[tuple(node)] = neighbors
    return graph


# In[22]:


def reconstruct_path(parents, current_node, start_node):
    path = []
    while current_node != start_node:
        path.insert(0, current_node)
        current_node = parents[current_node]
    path.insert(0, start_node)
    return path

def get_neighbors(graph, node):
    return graph.get(node, [])  # Already stored as (neighbor, cost)

def heuristic(node, goal, cache=None):
    if cache is None:
        cache = {}
    if (node, goal) not in cache:
        cache[(node, goal)] = np.linalg.norm(np.array(node) - np.array(goal))
    return cache[(node, goal)]


def get_node_with_lowest_f_score(open_set, g, stop_node):
    lowest_f_score = float('inf')
    lowest_node = None
    for node in open_set:
        f_score = g[node] + heuristic(node, stop_node)
        if f_score < lowest_f_score:
            lowest_node = node
            lowest_f_score = f_score
    return lowest_node


# In[23]:


def show_astar_warning_popup():
    # Initialize Tkinter
    root = tk.Tk()
    root.withdraw()  # Hide the main window

    # Show a warning message box
    messagebox.showwarning("Warning", "Start or Goal not inside graph (or path not found)")


# In[24]:


def aStarAlgo(start_node, stop_node, heuristic_func, graph, cell_size):
    global highlight_position
    global top_floor_id, physicsClient
    graph_nodes = np.array(list(graph.keys()))
    kd_tree = cKDTree(graph_nodes)

    # Query both start and goal together within the specified cell_size radius
    # Limit the radius to cell_size for efficiency
    radius = cell_size
    
    # Query nearest nodes within the radius
    start_idx = kd_tree.query(start_node, distance_upper_bound=radius)[1]  # Index of the nearest start node within radius
    goal_idx = kd_tree.query(stop_node, distance_upper_bound=radius)[1]    # Index of the nearest goal node within radius
    
    # If no valid node found within cell_size, we return None or could handle the error differently
    if start_idx == len(graph_nodes):  # No valid start node found within the radius
        highlight_position = None
        dehighlight_floor(top_floor_id, physicsClient)
        show_astar_warning_popup()
        print(f"No valid start node found within {radius} distance.")
        return [start_node]
    if goal_idx == len(graph_nodes):  # No valid goal node found within the radius
        highlight_position = None
        dehighlight_floor(top_floor_id, physicsClient)
        show_astar_warning_popup()
        print(f"No valid goal node found within {radius} distance.")
        return [start_node]
    
    start_node = tuple(graph_nodes[start_idx])
    stop_node = tuple(graph_nodes[goal_idx])
    
    open_set = []  # Min-heap priority queue
    closed_set = set()
    g = {}  # Cost from start to each node
    parents = {}  # Parent node for reconstructing path
    
    heapq.heappush(open_set, (0, start_node))  # Push (f_score, node)
    g[start_node] = 0
    parents[start_node] = None

    while open_set:
        # Pop node with lowest f_score
        _, current_node = heapq.heappop(open_set)
        
        if current_node == stop_node:  # Goal reached
            return reconstruct_path(parents, current_node, start_node)
        
        closed_set.add(current_node)

        for neighbor, cost in get_neighbors(graph, current_node):
            if neighbor in closed_set:
                continue

            tentative_g_score = g[current_node] + cost
            if neighbor not in g or tentative_g_score < g[neighbor]:
                parents[neighbor] = current_node
                g[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic_func(neighbor, stop_node)
                heapq.heappush(open_set, (f_score, neighbor))
    
    highlight_position = None
    dehighlight_floor(top_floor_id, physicsClient)
    show_astar_warning_popup()
    return [start_node]


# In[25]:


def show_warning_popup():
    # Initialize Tkinter
    root = tk.Tk()
    root.withdraw()  # Hide the main window

    # Show a warning message box
    messagebox.showwarning("Warning", "Not enough iterations!")


# In[26]:


def RRT_build(q_0, q_goal, n, bounds, adjusted_min, adjusted_max):
    V = [q_0]  # List of vertices (nodes)
    E = []  # List of edges (connections between nodes)
    global goal_reached
    goal_reached = False
    count = 0

    collision_free_goal = not check_collision_line(q_0, q_goal, adjusted_min, adjusted_max)

    # If the path to the goal is also collision-free, add the goal and edge, then stop
    if collision_free_goal:
        #print('path to goal good')
        E.append([q_0, q_goal])
        V.append(q_goal)
        goal_reached = True
        # show_success_popup()
    
    for _ in range(n):
        count = count + 1
        #print('New sample:', count)
        if goal_reached:  # If the goal is reached, exit the loop
            break
        
        q = np.array([random.uniform(bounds[0, 0], bounds[0, 1]), random.uniform(bounds[1, 0], bounds[1, 1]), random.uniform(bounds[2, 0], bounds[2, 1])])

        # Keep sampling new points if they collide with an obstacle
        while check_collision(q, adjusted_min, adjusted_max):
            q = np.array([random.uniform(bounds[0, 0], bounds[0, 1]), random.uniform(bounds[1, 0], bounds[1, 1]), random.uniform(bounds[2, 0], bounds[2, 1])])
        
        # Find the closest existing node in the tree
        distances = [np.linalg.norm(q - v) for v in V]
        closest_neighbour = V[np.argmin(distances)]

        # Interpolate between the new point and the closest neighbour
        collision_free = not check_collision_line(q, closest_neighbour, adjusted_min, adjusted_max)

        # If the path is collision-free, add the point and edge to the tree
        if collision_free:
            V.append(q)
            E.append([q, closest_neighbour])
            #print('collision free')

            # Check if the path to the goal is collision-free
            collision_free_goal = not check_collision_line(q, q_goal, adjusted_min, adjusted_max)

            # If the path to the goal is also collision-free, add the goal and edge, then stop
            if collision_free_goal:
                #print('path to goal good')
                E.append([q, q_goal])
                V.append(q_goal)
                goal_reached = True
                # show_success_popup()
                # break
    return V, E


# In[27]:


def find_RRT_path(E, q_0, q_goal):
    global top_floor_id, physicsClient
    global highlight_position
    global goal_reached
    if goal_reached == False:
        path = [q_0]
        print("No path is found within given iterations")
        highlight_position = None
        dehighlight_floor(top_floor_id, physicsClient)
        show_warning_popup()
    else:
        path = [q_goal, E[-1][0]]  # Start path with the goal
        while not np.allclose(path[-1], q_0, atol=1e-6):  # Loop until the start is reached
            for edge in E:
                if np.allclose(edge[0], path[-1], atol=1e-6):  # Find the edge leading to the last point in the path
                    path.append(edge[1])  # Add the next point in the path
                    break
        path.reverse()  # Reverse the path to go from start to goal
    return path


# In[28]:


def rrt_final(q_0, q_goal, n, bounds, adjusted_min, adjusted_max):
    #print("Initial:", q_0, "Goal: ", q_goal)
    V, E = RRT_build(q_0, q_goal, n, bounds, adjusted_min, adjusted_max)
    #print("V: ", V, "E: ", E)
    path = find_RRT_path(E, q_0, q_goal)
    #print(path)
    return path


# In[29]:


def update_sphere_positions(positions):
    # Access the global sphere_ids
    global sphere_ids
    # For example: move each sphere randomly every time
    for i in range(positions.shape[1]):
        new_position = positions[:, i]
        p.resetBasePositionAndOrientation(sphere_ids[i], list(new_position), [0, 0, 0, 1])  # Update position


# In[30]:


def get_initial_sphere_positions(num_spheres=N):
    # Example function to generate initial positions for all spheres
    positions = []
    for _ in range(num_spheres):
        x = 0
        y = 0
        z = 0
        positions.append([x, y, z])
    return positions


# In[31]:


def mpc_control(N, x_init, v_init, x_target, dt, buffer):
    weight_tracking = 1.0*np.eye(3)
    weight_input = 1e-3*np.diag([1, 1, 0.5])
    penalty_slack = 1e6  # Penalty for slack variables
    cost = 0.
    constraints = []
    global m
    global g
    global gravity_acceleration
    global max_thrust_z
    global max_thrust_xy
    global max_total_thrust
    global k_drag_per_m
    
    # Create the optimization variables
    x = cp.Variable((3, N + 1))
    v = cp.Variable((3, N + 1))
    u = cp.Variable((3, N))
    slack_radial = cp.Variable(N, nonneg=True)
    slack_projection = cp.Variable(N, nonneg=True)

    # Direction of the cylinder (axis)
    d = x_target - x_init
    d_norm = np.linalg.norm(d)
    d_unit = d / d_norm

    for k in range(N):
        # Tracking cost
        cost += cp.quad_form(x[:, k] - x_target, weight_tracking)
        cost += cp.quad_form(u[:, k], weight_input)
        cost += penalty_slack * (slack_radial[k] + slack_projection[k])
        
        # State dynamics constraints
        constraints += [x[:, k+1] == x[:, k] + dt * v[:, k]]
        constraints += [v[:, k+1] == v[:, k] + dt * (u[:, k] / m + gravity_acceleration - cp.multiply(k_drag_per_m, v[:, k]))]
        
        # Thrust constraints
        constraints += [cp.abs(u[0, k]) <= max_thrust_xy]
        constraints += [cp.abs(u[1, k]) <= max_thrust_xy]
        constraints += [cp.abs(u[2, k]) <= max_thrust_z]
        constraints += [cp.norm(u[:, k], 2) <= max_total_thrust]

        # Cylinder radial constraint
        # Projection of x onto the axis of the cylinder
        projection = x_init + cp.multiply((x[:, k] - x_init).T @ d_unit, d_unit)
        radial_distance = cp.norm(x[:, k] - projection, 2)
        constraints += [radial_distance <= buffer + slack_radial[k]]

        # Extended cylinder length constraint (allow up to r distance before x_init and after x_target)
        projection_length = (x[:, k] - x_init).T @ d_unit
        constraints += [projection_length <= d_norm + buffer + slack_projection[k]]  # Allow up to r distance beyond x_target

    # Initial conditions
    constraints += [x[:, 0] == x_init]
    constraints += [v[:, 0] == v_init]
    constraints += [v[:, N] == np.zeros(3)]
    
    # Solves the problem
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve(solver=cp.CLARABEL)
    return x[:, 1:].value, v[:, 1:].value, u.value


# In[32]:


def mpc_control_hover(N, x_init, v_init, x_target, dt, buffer):
    weight_tracking = 1.0*np.eye(3)
    weight_input = 1e-3*np.diag([1, 1, 0.5])
    penalty_slack = 1e6  # Penalty for slack variables
    cost = 0.
    constraints = []
    global m
    global g
    global gravity_acceleration
    global max_thrust_z
    global max_thrust_xy
    global max_total_thrust
    global k_drag_per_m
    
    # Create the optimization variables
    x = cp.Variable((3, N + 1))
    v = cp.Variable((3, N + 1))
    u = cp.Variable((3, N))
    slack_radial = cp.Variable(N, nonneg=True)

    for k in range(N):
        # Tracking cost
        cost += cp.quad_form(x[:, k] - x_target, weight_tracking)
        cost += cp.quad_form(u[:, k], weight_input)
        cost += penalty_slack * slack_radial[k]
        
        # State dynamics constraints
        constraints += [x[:, k+1] == x[:, k] + dt * v[:, k]]
        constraints += [v[:, k+1] == v[:, k] + dt * (u[:, k] / m + gravity_acceleration - cp.multiply(k_drag_per_m, v[:, k]))]
        
        # Thrust constraints
        constraints += [cp.abs(u[0, k]) <= max_thrust_xy]
        constraints += [cp.abs(u[1, k]) <= max_thrust_xy]
        constraints += [cp.abs(u[2, k]) <= max_thrust_z]
        constraints += [cp.norm(u[:, k], 2) <= max_total_thrust]

        # radial constraint
        radial_distance = cp.norm(x[:, k] - x_target, 2)
        constraints += [radial_distance <= buffer + slack_radial[k]]

    # Initial conditions
    constraints += [x[:, 0] == x_init]
    constraints += [v[:, 0] == v_init]
    constraints += [v[:, N] == np.zeros(3)]
    
    # Solves the problem
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve(solver=cp.CLARABEL)
    return x[:, 1:].value, v[:, 1:].value, u.value


# In[33]:


def draw_or_remove_lines(points, draw=True, line_color=[0, 0, 0], line_width=5.0):
    """
    Draws or removes lines connecting all the points in the provided numpy array in the PyBullet environment.

    Args:
        points (np.ndarray): A numpy array of shape (N, 3) containing the coordinates of the points.
        draw (bool): If True, draw the lines. If False, remove all previously drawn lines.
        line_color (list): A list of 3 or 4 elements representing the RGB(A) color of the line.
        line_width (float): The width of the line (only applicable when drawing lines).
    Returns:
        list: A list of IDs for the drawn lines (if `draw=True`).
    """
    if not isinstance(points, np.ndarray):
        points = np.array(points)
    if points.shape[1] != 3:
        raise ValueError("Each point in 'points' must have exactly 3 coordinates.")
    if len(points) < 2:
        return None
        #raise ValueError("At least two points are required to draw a line.")

    # Store debug line IDs
    line_ids = []

    if draw:
        for i in range(len(points) - 1):
            start_point = points[i]
            end_point = points[i + 1]
            line_id = p.addUserDebugLine(
                start_point, end_point, lineColorRGB=line_color[:3], lineWidth=line_width
            )
            line_ids.append(line_id)
        return line_ids
    else:
        # Remove all debug items (or you can use a list of IDs if tracked)
        p.removeAllUserDebugItems()
        return None


# In[34]:


def total_path_length(vectors):
    # Ensure the list is a numpy array for vectorized operations
    vectors = np.array(vectors)
    
    # Calculate the sum of distances between consecutive points
    path_length = 0
    for i in range(1, len(vectors)):
        # Compute the Euclidean distance between vectors[i-1] and vectors[i]
        path_length += np.linalg.norm(vectors[i] - vectors[i-1])
    
    return path_length


# In[35]:


def show_messagebox(title, message):
    # Use the main thread's tkinter context to create the message box
    root = tk.Toplevel()  # Use Toplevel for a new dialog without creating a new Tk instance
    root.withdraw()  # Hide the Toplevel window (acts as a dummy parent for the messagebox)
    messagebox.showinfo(title, message)
    root.destroy()  # Destroy the Toplevel instance after showing the message


# In[36]:


def follower_mpc_rrt(robot_id, target_position, bounds, buffer, adjusted_min, adjusted_max):
    global global_mpc_value
    global threshold_mpc
    global N
    global freq
    global rrt_max_iterations
    global position_noise_std
    global velocity_noise_std
    robot_position, robot_orientation = p.getBasePositionAndOrientation(robot_id)
    robot_position = np.array(robot_position)
    target_position = np.array(target_position)

    start_time = time.time()
    
    path = rrt_final(robot_position, target_position, rrt_max_iterations, bounds, adjusted_min, adjusted_max)

    end_time = time.time()
    elapsed_time = end_time - start_time
    
    draw_or_remove_lines(np.array(path))
    velocity = np.zeros(3)
    mpc_counter = 0
    total_sim_time = 0
    fuel_units = 0
    
    i = 0
    while i < len(path):
        while (i + 1) < len(path) and not check_collision_line(robot_position, path[i + 1], adjusted_min, adjusted_max) and mpc_control(N, robot_position, velocity, path[i + 1], 1 / (3*freq), buffer)[0] is not None:
            i += 1
        target_position = path[i]
        while euclidean_distance(robot_position, target_position) > threshold_mpc:
            while (i + 1) < len(path) and not check_collision_line(robot_position, path[i + 1], adjusted_min, adjusted_max) and mpc_control(N, robot_position, velocity, path[i + 1], 1 / (3*freq), buffer)[0] is not None:
                i += 1
            target_position = path[i]
            next_states, next_velocities, thrusts = mpc_control(N, robot_position, velocity, target_position, 1 / freq, buffer)
            while next_states is None:
                mpc_counter += 1
                print("MPC error count:", mpc_counter)
                print("robot_position:", robot_position)
                print("velocity:", velocity)
                print("target_position:", target_position)
                next_states, next_velocities, thrusts = mpc_control(N, robot_position, velocity, target_position, 1 / (freq + mpc_counter), buffer)
            total_sim_time += 1 / (freq + mpc_counter)
            update_sphere_positions(next_states)
            next_state = next_states[:, 0]
            next_velocity = next_velocities[:, 0]
            thrust = thrusts[:, 0]                
            global_mpc_value = thrust
            fuel_units += np.linalg.norm(thrust) / (freq + mpc_counter)
            mpc_counter = 0

            position_noise = np.random.normal(0, position_noise_std, next_state.shape)
            velocity_noise = np.random.normal(0, velocity_noise_std, next_velocity.shape)
            
            next_state += position_noise
            next_velocity += velocity_noise
            
            robot_position = next_state
            velocity = next_velocity
            p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=list(robot_position))
            p.resetBasePositionAndOrientation(robot_id, list(robot_position), robot_orientation)
            p.stepSimulation()

        i += 1

    additional_info1 = f"Path length: {total_path_length(path):.4f}"
    additional_info2 = f"Sim execution time: {total_sim_time:.4f} seconds"
    additional_info3 = f"Fuel units: {fuel_units:.4f}"
    title = "RRT Path"
    message = f"Time taken to find path: {elapsed_time:.6f} seconds\n{additional_info1}\n{additional_info2}\n{additional_info3}"
    message_thread = threading.Thread(target=show_messagebox, args=(title, message))
    message_thread.start()
    
    global_mpc_value = np.zeros(3)
    draw_or_remove_lines(np.array(path), False)
    update_sphere_positions(np.zeros((3, N)))
    follower_mpc_hover(robot_id, target_position, buffer)


# In[37]:


def follower_mpc_astar(robot_id, target_position, heuristic_func, buffer, graph, adjusted_min, adjusted_max):
    global global_mpc_value
    global threshold_mpc
    global N
    global freq
    global cell_size
    global position_noise_std
    global velocity_noise_std    
    
    robot_position, robot_orientation = p.getBasePositionAndOrientation(robot_id)
    robot_position = np.array(robot_position)
    target_position = np.array(target_position)

    start_time = time.time()
    
    path = aStarAlgo(robot_position, target_position, heuristic_func, graph, cell_size)

    end_time = time.time()
    elapsed_time = end_time - start_time
    
    draw_or_remove_lines(np.array(path))
    velocity = np.zeros(3)
    total_sim_time = 0
    mpc_counter = 0
    fuel_units = 0
 
    i = 0
    while i < len(path):
        while (i + 1) < len(path) and not check_collision_line(robot_position, path[i + 1], adjusted_min, adjusted_max) and mpc_control(N, robot_position, velocity, path[i + 1], 1 / (3*freq), buffer)[0] is not None:
            i += 1
        target_position = path[i]
        while euclidean_distance(robot_position, target_position) > threshold_mpc:
            while (i + 1) < len(path) and not check_collision_line(robot_position, path[i + 1], adjusted_min, adjusted_max) and mpc_control(N, robot_position, velocity, path[i + 1], 1 / (3*freq), buffer)[0] is not None:
                i += 1
            target_position = path[i]
            next_states, next_velocities, thrusts = mpc_control(N, robot_position, velocity, target_position, 1 / freq, buffer)
            while next_states is None:
                mpc_counter += 1
                print("MPC error count:", mpc_counter)
                print("robot_position:", robot_position)
                print("velocity:", velocity)
                print("target_position:", target_position)
                next_states, next_velocities, thrusts = mpc_control(N, robot_position, velocity, target_position, 1 / (freq + mpc_counter), buffer)
            total_sim_time += 1 / (freq + mpc_counter)
            update_sphere_positions(next_states)
            next_state = next_states[:, 0]
            next_velocity = next_velocities[:, 0]
            thrust = thrusts[:, 0]            
            global_mpc_value = thrust
            fuel_units += np.linalg.norm(thrust) / (freq + mpc_counter)   
            mpc_counter = 0

            position_noise = np.random.normal(0, position_noise_std, next_state.shape)
            velocity_noise = np.random.normal(0, velocity_noise_std, next_velocity.shape)
            
            next_state += position_noise
            next_velocity += velocity_noise
            
            robot_position = next_state
            velocity = next_velocity
            p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=list(robot_position))
            p.resetBasePositionAndOrientation(robot_id, list(robot_position), robot_orientation)
            p.stepSimulation()

        i += 1

    additional_info1 = f"Path length: {total_path_length(path):.4f}"
    additional_info2 = f"Sim execution time: {total_sim_time:.4f} seconds"
    additional_info3 = f"Fuel units: {fuel_units:.4f}"
    title = "A* Path"
    message = f"Time taken to find path: {elapsed_time:.6f} seconds\n{additional_info1}\n{additional_info2}\n{additional_info3}"
    message_thread = threading.Thread(target=show_messagebox, args=(title, message))
    message_thread.start()
    
    global_mpc_value = np.zeros(3)
    draw_or_remove_lines(np.array(path), False)
    update_sphere_positions(np.zeros((3, N)))
    follower_mpc_hover(robot_id, target_position, buffer)


# In[38]:


def follower_mpc_hover(robot_id, target_position, buffer):
    global start_position_to_repeat
    global goal_position_to_repeat
    global global_mpc_value
    global threshold_mpc
    global N
    global freq
    global position_noise_std
    global velocity_noise_std
    global droneId
    global sphere_id
    global startOrientation
    global bounds
    global mpc_buffer, graph, default_adjusted_min, default_adjusted_max
    global mins, maxs, merged_mins, merged_maxs, map_indices
    
    robot_position, robot_orientation = p.getBasePositionAndOrientation(robot_id)
    robot_position = np.array(robot_position)
    target_position = np.array(target_position)
    velocity = np.zeros(3)
    mpc_counter = 0
 
    while True:
        keys = p.getKeyboardEvents()  # Ensure keys are updated
        if ord('q') in keys:
            if keys[ord('q')] & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
                p.disconnect()
                break  # Exit the loop when 'p' is pressed

        if ord('f') in keys:
            if keys[ord('f')] & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
                randomizer()
                
        if ord('t') in keys:
            if keys[ord('t')] & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
                if goal_position_to_repeat is not None:
                    p.changeVisualShape(sphere_id, -1, rgbaColor=[0, 0, 1, 0])
                    start_point = start_position_to_repeat
                    goal_point = goal_position_to_repeat
                    p.resetBasePositionAndOrientation(droneId, list(start_point), startOrientation)
                    p.resetDebugVisualizerCamera(cameraDistance=0.8*np.linalg.norm(goal_point - start_point), cameraYaw=50, cameraPitch=-30, cameraTargetPosition=list(np.mean((goal_point, start_point), axis=0)))
                    final_mins, final_maxs = merge_irrelevant_boxes(start_point, goal_point, mins, maxs, merged_mins, merged_maxs, map_indices)
                    adjusted_min, adjusted_max = adjusted_min_max(final_mins, final_maxs, buffer)
                    follower_mpc_rrt(droneId, list(goal_point), bounds, mpc_buffer, adjusted_min, adjusted_max)
                    
        if ord('z') in keys:
            if keys[ord('z')] & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
                if highlight_position is not None:
                    p.changeVisualShape(sphere_id, -1, rgbaColor=[0, 0, 1, 0])
                    start_point = start_position_to_repeat
                    goal_point = goal_position_to_repeat
                    p.resetBasePositionAndOrientation(droneId, list(start_point), startOrientation)
                    p.resetDebugVisualizerCamera(cameraDistance=0.8*np.linalg.norm(goal_point - start_point), cameraYaw=50, cameraPitch=-30, cameraTargetPosition=list(np.mean((goal_point, start_point), axis=0)))
                    follower_mpc_astar(droneId, list(goal_point), heuristic, mpc_buffer, graph, default_adjusted_min, default_adjusted_max)
        
        next_states, next_velocities, thrusts = mpc_control_hover(N, robot_position, velocity, target_position, 1 / freq, buffer)
        while next_states is None:
            mpc_counter += 1
            print("MPC error count:", mpc_counter)
            print("robot_position:", robot_position)
            print("velocity:", velocity)
            print("target_position:", target_position)
            next_states, next_velocities, thrusts = mpc_control_hover(N, robot_position, velocity, target_position, 1 / (freq + mpc_counter), buffer)
        mpc_counter = 0
        next_state = next_states[:, 0]
        next_velocity = next_velocities[:, 0]
        thrust = thrusts[:, 0]          
        global_mpc_value = thrust

        position_noise = np.random.normal(0, position_noise_std, next_state.shape)
        velocity_noise = np.random.normal(0, velocity_noise_std, next_velocity.shape)
        
        next_state += position_noise
        next_velocity += velocity_noise
        
        robot_position = next_state
        velocity = next_velocity
        p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=list(target_position))
        p.resetBasePositionAndOrientation(robot_id, list(robot_position), robot_orientation)
        p.stepSimulation()

    global_mpc_value = np.zeros(3)


# In[39]:


def randomizer():
    global sphere_id
    global droneId
    global highlight_position
    global target_x_relative
    global target_y_relative
    global target_z_relative
    global buffer
    global mins, maxs, merged_mins, merged_maxs, map_indices
    global mpc_buffer, graph, default_adjusted_min, default_adjusted_max
    global default_a_start_adjusted_min, default_a_start_adjusted_max
    global ROOM_URDF_PATH, ROOM_HEIGHT
    global bounds
    global physicsClient
    global top_floor_id
    global buildings
    global start_position_to_repeat
    global goal_position_to_repeat
    global startOrientation

    p.resetDebugVisualizerCamera(cameraDistance=0.5*np.linalg.norm(bounds[:, 1] - bounds[:, 0]), cameraYaw=50-50, cameraPitch=-30, cameraTargetPosition=list(bounds.mean(axis = 1)))
    if highlight_position is not None:
        dehighlight_floor(top_floor_id, physicsClient)
    [b_x, b_y, top_floor_z], top_floor_id = highlight_random_floor(buildings, ROOM_URDF_PATH, ROOM_HEIGHT, physicsClient)
    highlight_position = [b_x+target_x_relative, b_y+target_y_relative, top_floor_z+target_z_relative]
    robot_position, robot_orientation = p.getBasePositionAndOrientation(sphere_id)
    p.resetBasePositionAndOrientation(sphere_id, highlight_position, robot_orientation)
    update_sphere_color(sphere_id, np.array(robot_position), default_a_start_adjusted_min, default_a_start_adjusted_max)

    while True:
        keys = p.getKeyboardEvents()  # Ensure keys are updated
        if ord('q') in keys:
            if keys[ord('q')] & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
                p.disconnect()
                break  # Exit the loop when 'p' is pressed
        if ord('f') in keys:
            if keys[ord('f')] & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
                if highlight_position is not None:
                    dehighlight_floor(top_floor_id, physicsClient)
                [b_x, b_y, top_floor_z], top_floor_id = highlight_random_floor(buildings, ROOM_URDF_PATH, ROOM_HEIGHT, physicsClient)
                highlight_position = [b_x+target_x_relative, b_y+target_y_relative, top_floor_z+target_z_relative]
                robot_position, robot_orientation = p.getBasePositionAndOrientation(sphere_id)
                p.resetBasePositionAndOrientation(sphere_id, highlight_position, robot_orientation)
                update_sphere_color(sphere_id, np.array(robot_position), default_a_start_adjusted_min, default_a_start_adjusted_max)

        if ord('r') in keys:
            if keys[ord('r')] & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
                if highlight_position is not None:
                    p.changeVisualShape(sphere_id, -1, rgbaColor=[0, 0, 1, 0])
                    robot_position, robot_orientation = p.getBasePositionAndOrientation(droneId)
                    start_point = np.array(robot_position)
                    goal_point = np.array(highlight_position)
                    start_position_to_repeat = start_point
                    goal_position_to_repeat = goal_point
                    p.resetDebugVisualizerCamera(cameraDistance=0.8*np.linalg.norm(goal_point - start_point), cameraYaw=50, cameraPitch=-30, cameraTargetPosition=list(np.mean((goal_point, start_point), axis=0)))
                    final_mins, final_maxs = merge_irrelevant_boxes(start_point, goal_point, mins, maxs, merged_mins, merged_maxs, map_indices)
                    adjusted_min, adjusted_max = adjusted_min_max(final_mins, final_maxs, buffer)
                    follower_mpc_rrt(droneId, highlight_position, bounds, mpc_buffer, adjusted_min, adjusted_max)

        if ord('t') in keys:
            if keys[ord('t')] & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
                if goal_position_to_repeat is not None:
                    p.changeVisualShape(sphere_id, -1, rgbaColor=[0, 0, 1, 0])
                    start_point = start_position_to_repeat
                    goal_point = goal_position_to_repeat
                    p.resetBasePositionAndOrientation(droneId, list(start_point), startOrientation)
                    p.resetDebugVisualizerCamera(cameraDistance=0.8*np.linalg.norm(goal_point - start_point), cameraYaw=50, cameraPitch=-30, cameraTargetPosition=list(np.mean((goal_point, start_point), axis=0)))
                    final_mins, final_maxs = merge_irrelevant_boxes(start_point, goal_point, mins, maxs, merged_mins, merged_maxs, map_indices)
                    adjusted_min, adjusted_max = adjusted_min_max(final_mins, final_maxs, buffer)
                    follower_mpc_rrt(droneId, list(goal_point), bounds, mpc_buffer, adjusted_min, adjusted_max)
                    
        if ord('a') in keys:
            if keys[ord('a')] & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
                if highlight_position is not None:
                    p.changeVisualShape(sphere_id, -1, rgbaColor=[0, 0, 1, 0])
                    robot_position, robot_orientation = p.getBasePositionAndOrientation(droneId)
                    start_point = np.array(robot_position)
                    goal_point = np.array(highlight_position)
                    start_position_to_repeat = start_point
                    goal_position_to_repeat = goal_point
                    p.resetDebugVisualizerCamera(cameraDistance=0.8*np.linalg.norm(goal_point - start_point), cameraYaw=50, cameraPitch=-30, cameraTargetPosition=list(np.mean((goal_point, start_point), axis=0)))
                    follower_mpc_astar(droneId, highlight_position, heuristic, mpc_buffer, graph, default_adjusted_min, default_adjusted_max)

        if ord('z') in keys:
            if keys[ord('z')] & (p.KEY_IS_DOWN | p.KEY_WAS_TRIGGERED):
                if highlight_position is not None:
                    p.changeVisualShape(sphere_id, -1, rgbaColor=[0, 0, 1, 0])
                    start_point = start_position_to_repeat
                    goal_point = goal_position_to_repeat
                    p.resetBasePositionAndOrientation(droneId, list(start_point), startOrientation)
                    p.resetDebugVisualizerCamera(cameraDistance=0.8*np.linalg.norm(goal_point - start_point), cameraYaw=50, cameraPitch=-30, cameraTargetPosition=list(np.mean((goal_point, start_point), axis=0)))
                    follower_mpc_astar(droneId, list(goal_point), heuristic, mpc_buffer, graph, default_adjusted_min, default_adjusted_max)


# In[40]:


def run_simulation():
    global physicsClient
    global mins, maxs
    global default_adjusted_min, default_adjusted_max
    global default_a_start_adjusted_min, default_a_start_adjusted_max
    global merged_mins, merged_maxs, map_indices
    global bounds
    global graph, sphere_id, droneId
    global buildings
    global startPos, startOrientation
    global DRONE_URDF_PATH, ROOM_URDF_PATH
    
    physicsClient = p.connect(p.GUI)
    
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    
    # Load environment
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")  # Ground plane
    
    buildings = generate_grid_buildings(num_rows, num_cols, spacing, floors_min, floors_max)
    all_buildings_obstacles, mins, maxs = create_buildings_and_obstacles(buildings, ROOM_URDF_PATH, ROOM_HEIGHT, physicsClient)
    default_adjusted_min, default_adjusted_max = adjusted_min_max(mins, maxs, buffer)
    default_a_start_adjusted_min, default_a_start_adjusted_max = adjusted_min_max(mins, maxs, a_start_buffer)
    merged_mins, merged_maxs, map_indices = merge_bounding_boxes(mins, maxs, merge_threshold)
    bounds = find_bounds(mins, maxs, buffer, space_outside_building)
    merged_mins, merged_maxs, map_indices = merge_bounding_boxes(mins, maxs, merge_threshold)
    p.resetDebugVisualizerCamera(cameraDistance=0.5*np.linalg.norm(bounds[:, 1] - bounds[:, 0]), cameraYaw=50-50, cameraPitch=-30, cameraTargetPosition=list(bounds.mean(axis = 1)))


    start_time = time.time()
    
    node_grid = find_node_grid(bounds, cell_size)
    collision_free_array = find_collision_free_array(node_grid, default_a_start_adjusted_min, default_a_start_adjusted_max)
    graph = build_graph(collision_free_array, cell_size)

    end_time = time.time()
    elapsed_time = end_time - start_time
    title = "Forming A* Graph"
    message = f"Time taken: {elapsed_time:.4f} seconds"
    message_thread = threading.Thread(target=show_messagebox, args=(title, message))
    message_thread.start()
    
    sphere_radius = 0.05
    sphere_visual_shape = p.GEOM_SPHERE
    sphere_color = [1, 1, 1, 1]  # White color
    
    sphere_positions = get_initial_sphere_positions()  # Get positions for spheres
    
    for position in sphere_positions:
        sphere_visual = p.createVisualShape(sphere_visual_shape, radius=sphere_radius, rgbaColor=sphere_color)
        sphere_idx = p.createMultiBody(baseMass=0,  # Static object
                                      baseVisualShapeIndex=sphere_visual,
                                      basePosition=position)
        sphere_ids.append(sphere_idx)
    
    # Add a sphere to the simulation
    sphere_radius = 0.1
    sphere_visual_shape = p.GEOM_SPHERE
    sphere_visual = p.createVisualShape(sphere_visual_shape, radius=sphere_radius, rgbaColor=[1, 0, 0, 1])
    sphere_start_position = [0, 0, -2.15]
    
    sphere_id = p.createMultiBody(baseMass=0,  # Static object
                                  baseVisualShapeIndex=sphere_visual,
                                  basePosition=sphere_start_position)
    
    # Load the visual shape from the URDF file
    droneId = p.loadURDF(DRONE_URDF_PATH, basePosition=startPos, useFixedBase=True)
    
    # Adjust the camera to focus on the drone
    follower_mpc_hover(droneId, startPos, buffer)
    
    # Disconnect
    p.disconnect()


# In[41]:


def create_progress_gui():
    global global_mpc_value  # Access the global variable (np.array of shape (3,))
    global max_thrust_z, max_thrust_xy, max_total_thrust  # Access thrust limits

    root = Tk()
    root.title("Drone Thrust")  # Window title

    # X Thrust (MPC Value 0)
    label_x = Label(root, text=f"X thrust (-{max_thrust_xy} to {max_thrust_xy})", font=("Arial", 14))
    label_x.pack(pady=10)

    canvas_width = 400  # Width of the canvas
    canvas_height = 40  # Height of the canvas
    canvas_x = Canvas(root, width=canvas_width, height=canvas_height, bg="black", highlightthickness=0)
    canvas_x.pack(pady=20)

    # Y Thrust (MPC Value 1)
    label_y = Label(root, text=f"Y thrust (-{max_thrust_xy} to {max_thrust_xy})", font=("Arial", 14))
    label_y.pack(pady=10)

    canvas_y = Canvas(root, width=40, height=400, bg="black", highlightthickness=0)  # Height increased
    canvas_y.pack(pady=20)

    # Z Thrust (MPC Value 2, Range -max_thrust_z to max_thrust_z)
    label_z = Label(root, text=f"Z thrust (-{max_thrust_z} to {max_thrust_z})", font=("Arial", 14))
    label_z.pack(pady=10)

    canvas_z = Canvas(root, width=400, height=40, bg="black", highlightthickness=0)  # Width increased
    canvas_z.pack(pady=20)

    # Total Thrust (Norm of MPC Values)
    label_total = Label(root, text=f"Total thrust limit (0 to {max_total_thrust})", font=("Arial", 14))
    label_total.pack(pady=10)

    canvas_total = Canvas(root, width=400, height=40, bg="black", highlightthickness=0)
    canvas_total.pack(pady=20)

    # Display values for smooth transitions
    display_values = [0, 0, 0, 0]  # Mutable list for smoothing all four bars

    # Function to always return red color
    def get_color(value, value_range):
        # Return red color (255, 0, 0)
        return '#ff0000'  # Hex representation of red color

    # Function to update all bars dynamically
    def update_bars():
        nonlocal display_values  # Access the mutable display values
        target_values = np.array(global_mpc_value)  # Ensure target_values is a NumPy array

        # Calculate the norm of the global_mpc_value for the total thrust bar
        total_thrust = np.linalg.norm(target_values)

        # Smoothly adjust the display values towards the targets
        smoothing_factor = 0.4  # Adjust the rate of change (lower is smoother)
        for i in range(3):
            display_values[i] += (target_values[i] - display_values[i]) * smoothing_factor
        display_values[3] += (total_thrust - display_values[3]) * smoothing_factor

        # Clear the canvases
        canvas_x.delete("all")
        canvas_y.delete("all")
        canvas_z.delete("all")
        canvas_total.delete("all")

        # Update X Thrust Bar (MPC Value 0)
        rect_center_x = (display_values[0] + max_thrust_xy) / (2 * max_thrust_xy) * canvas_width
        rect_width_x = 60  # Doubled the width of the colored region
        rect_left_x = max(0, rect_center_x - rect_width_x / 2)
        rect_right_x = min(canvas_width, rect_center_x + rect_width_x / 2)
        color_x = get_color(display_values[0], (-max_thrust_xy, max_thrust_xy))
        canvas_x.create_rectangle(rect_left_x, 0, rect_right_x, canvas_height, fill=color_x, outline="")

        # Update Y Thrust Bar (MPC Value 1)
        rect_center_y = (max_thrust_xy - display_values[1]) / (2 * max_thrust_xy) * canvas_y.winfo_height()
        rect_height_y = 60  # Doubled the height of the colored region
        rect_top_y = max(0, rect_center_y - rect_height_y / 2)
        rect_bottom_y = min(canvas_y.winfo_height(), rect_center_y + rect_height_y / 2)
        color_y = get_color(display_values[1], (-max_thrust_xy, max_thrust_xy))
        canvas_y.create_rectangle(0, rect_top_y, canvas_y.winfo_width(), rect_bottom_y, fill=color_y, outline="")

        # Update Z Thrust Bar (MPC Value 2, Range -max_thrust_z to max_thrust_z)
        rect_center_z = (display_values[2] + max_thrust_z) / (2 * max_thrust_z) * canvas_width
        rect_width_z = 60  # Doubled the width of the colored region
        rect_left_z = max(0, rect_center_z - rect_width_z / 2)
        rect_right_z = min(canvas_width, rect_center_z + rect_width_z / 2)
        color_z = get_color(display_values[2], (-max_thrust_z, max_thrust_z))
        canvas_z.create_rectangle(rect_left_z, 0, rect_right_z, canvas_height, fill=color_z, outline="")

        # Update Total Thrust Bar (Norm of MPC Values)
        rect_total = display_values[3] / max_total_thrust * canvas_width
        canvas_total.create_rectangle(0, 0, rect_total, canvas_height, fill="#ff0000", outline="")

        # Schedule the next update
        root.after(50, update_bars)  # Update every 50 ms

    update_bars()  # Start the periodic update
    root.mainloop()


# In[42]:


def display_key_info():
    """Create a GUI to display key information."""
    key_info_root = Tk()
    key_info_root.title("Key Information")

    # Instructions
    instructions = [
        "Q key: Quit",
        "F key: Find a random floor as goal",
        "A key: Implement A* to goal",
        "R key: Implement RRT to goal",
        "Z key: Repeat previous goal with A*",
        "T key: Repeat previous goal with RRT",
    ]

    # Display each instruction in a label
    for instruction in instructions:
        label = Label(key_info_root, text=instruction, font=("Arial", 14))
        label.pack(pady=5)

    key_info_root.mainloop()

def run_key_info_thread():
    """Run the key information GUI in a separate thread."""
    key_info_thread = threading.Thread(target=display_key_info)
    key_info_thread.daemon = True  # Ensure the thread exits when the main program does
    key_info_thread.start()


# In[ ]:


simulation_thread = threading.Thread(target=run_simulation)
simulation_thread.daemon = True
simulation_thread.start()
run_key_info_thread()
create_progress_gui()

