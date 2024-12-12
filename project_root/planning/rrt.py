import random
import math
import pybullet as p

class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2 + (node1.z - node2.z) ** 2)

def nearest_node(tree, node):
    return min(tree, key=lambda n: distance(n, node))

def is_collision_free(node1, node2, obstacles, clearance, step_size=0.1):
    dx = node2.x - node1.x
    dy = node2.y - node1.y
    dz = node2.z - node1.z
    distance_between_nodes = math.sqrt(dx**2 + dy**2 + dz**2)

    if distance_between_nodes < step_size:
        return True

    steps = int(distance_between_nodes / step_size)

    debug_point_id = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 1, 0, 0.5])

    for i in range(steps + 1):
        alpha = i / steps
        x = node1.x + alpha * dx
        y = node1.y + alpha * dy
        z = node1.z + alpha * dz
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=debug_point_id, basePosition=[x, y, z])
        if point_in_obstacle((x, y, z), obstacles, clearance):
            return False
    return True

def point_in_obstacle(point, obstacles, clearance):
    x, y, z = point
    for obs in obstacles:
        obs_center = obs["center"]
        obs_size = obs["size"]
        half_size = [(dim + clearance) / 2 for dim in obs_size]

        min_corner = [obs_center[i] - half_size[i] for i in range(3)]
        max_corner = [obs_center[i] + half_size[i] for i in range(3)]

        epsilon = 1e-5
        if all(min_corner[i] + epsilon <= point[i] <= max_corner[i] - epsilon for i in range(3)):
            #print("[DEBUG] Point inside obstacle:", point)
            #print("[DEBUG] Obstacle bounds:", min_corner, max_corner)
            return True
    return False

def calculate_building_bounds(obstacles, floor_height, chosen_num_floors):
    x_max = float('-inf')
    x_min = float('inf')
    y_max = float('-inf')
    y_min = float('inf')

    for obs in obstacles:
        center, size = obs["center"], obs["size"]
        half_size_x = size[0] / 2
        half_size_y = size[1] / 2

        x_max = max(x_max, center[0] + half_size_x)-0.08
        x_min = min(x_min, center[0] - half_size_x)+0.08
        y_max = max(y_max, center[1] + half_size_y)-0.08
        y_min = min(y_min, center[1] - half_size_y)+0.08

    # Calculate total building height
    building_height = floor_height * chosen_num_floors

    return {
        "min_x": x_min,
        "max_x": x_max,
        "min_y": y_min,
        "max_y": y_max,
        "building_height": building_height
    }




def generate_path_rrt(start, goal, obstacles, step_size=0.2, max_iters=2000, clearance=0.5, floor_height=2.0, chosen_num_floors=1):
    # Use the correct floor_height here
    
    bounds = calculate_building_bounds(obstacles, floor_height=floor_height, chosen_num_floors=chosen_num_floors)
    print(f"Building bounds: {bounds}")

    start_node = Node(*start)
    goal_node = Node(*goal)
    tree = [start_node]

    collision_free_count = 0
    in_collision_count = 0

    for i in range(max_iters):
        # Sample a random point within adjusted bounds
        rand_point = Node(
            random.uniform(bounds["min_x"], bounds["max_x"]),
            random.uniform(bounds["min_y"], bounds["max_y"]),
            random.uniform(0, bounds["building_height"])
        ) 

        # Check if rand_point is in obstacle
        if point_in_obstacle((rand_point.x, rand_point.y, rand_point.z), obstacles, clearance):
            #visualize_random_point([rand_point.x, rand_point.y, rand_point.z], color=[0, 0, 0, 1])  # Black
            in_collision_count += 1
            continue
        else:
            #visualize_random_point([rand_point.x, rand_point.y, rand_point.z], color=[0, 1, 0, 1])  # Green
            collision_free_count += 1

        # Find nearest node in the tree
        nearest = nearest_node(tree, rand_point)

        # Steer towards the random point
        dx = rand_point.x - nearest.x
        dy = rand_point.y - nearest.y
        dz = rand_point.z - nearest.z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist == 0:
            continue

        # Move only one step_size towards the rand_point
        scale = step_size / dist
        new_node = Node(
            nearest.x + dx * scale,
            nearest.y + dy * scale,
            nearest.z + dz * scale
        )
        new_node.parent = nearest

        # Check if the line from nearest to new_node is collision-free
        if is_collision_free(nearest, new_node, obstacles, clearance, step_size=0.05):
            #print(f"New node at ({new_node.x}, {new_node.y}, {new_node.z}) is collision-free")
            tree.append(new_node)

            # Check if we reached the goal
            if distance(new_node, goal_node) < step_size:
                print("Goal reached!")
                goal_node.parent = new_node
                tree.append(goal_node)
                break
        #else:
            #print(f"New node at ({new_node.x}, {new_node.y}, {new_node.z}) is in collision")

    path = []
    current = goal_node
    while current is not None:
        path.append((current.x, current.y, current.z))
        current = current.parent

    print(f"Collision-free points: {collision_free_count}")
    print(f"Points in collision: {in_collision_count}")

    return path[::-1]

def interpolate_path(path, interp_step=0.1):
    if not path or len(path) < 2:
        return path
    new_path = []
    for i in range(len(path) - 1):
        start_wp = path[i]
        end_wp = path[i + 1]
        dist = math.sqrt(
            (end_wp[0] - start_wp[0])**2
            + (end_wp[1] - start_wp[1])**2
            + (end_wp[2] - start_wp[2])**2
        )
        steps = int(math.ceil(dist / interp_step))
        for s in range(steps):
            alpha = s / float(steps)
            x = start_wp[0] * (1 - alpha) + end_wp[0] * alpha
            y = start_wp[1] * (1 - alpha) + end_wp[1] * alpha
            z = start_wp[2] * (1 - alpha) + end_wp[2] * alpha
            new_path.append((x, y, z))
    new_path.append(path[-1])
    return new_path

def visualize_path_as_line(path, color=[1, 0, 1], line_width=2):
    if len(path) < 2:
        print("[DEBUG] Path has less than 2 waypoints. Nothing to visualize.")
        return
    for i in range(len(path) - 1):
        start = path[i]
        end = path[i + 1]
        p.addUserDebugLine(start, end, lineColorRGB=color, lineWidth=line_width)
    print("[DEBUG] Path visualized.")

def visualize_random_point(point, radius=0.1, color=[0, 0, 1, 0.5]):
    sphere_visual = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=color
    )
    sphere_body = p.createMultiBody(
        baseMass=0,  # Static object
        baseVisualShapeIndex=sphere_visual,
        basePosition=point
    )
    return sphere_body


def check_collision(coords, obstacles, clearance=0.0):
    """
    Check if a given point collides with any obstacle.

    :param coords: Coordinates of the point [x, y, z].
    :param obstacles: List of obstacle dictionaries.
    :param clearance: Clearance buffer around obstacles.
    :return: True if collision occurs, False otherwise.
    """
    x, y, z = coords
    for obs in obstacles:
        obs_center = obs["center"]
        obs_size = obs["size"]
        half_size = [(dim + clearance) / 2 for dim in obs_size]

        min_corner = [obs_center[i] - half_size[i] for i in range(3)]
        max_corner = [obs_center[i] + half_size[i] for i in range(3)]

        if all(min_corner[i] <= coords[i] <= max_corner[i] for i in range(3)):
            return True
    return False
