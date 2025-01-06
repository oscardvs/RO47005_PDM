import random
import math
import pybullet as p

class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.parent = None
        self.cost = 0.0

def distance(node1, node2):
    return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2 + (node1.z - node2.z) ** 2)

def nearest_node(tree, node):
    return min(tree, key=lambda n: distance(n, node))

def near_nodes(tree, new_node, rewire_radius):
    near = []
    for n in tree:
        if distance(n, new_node) < rewire_radius:
            near.append(n)
    return near

def line_collision_free(node1, node2, obstacles, clearance, step_size=0.05):
    dx = node2.x - node1.x
    dy = node2.y - node1.y
    dz = node2.z - node1.z
    dist = distance(node1, node2)

    # Handle the case where dist is NaN or zero
    if math.isnan(dist) or dist == 0:
        return False

    steps = max(int(dist / step_size), 1)

    for i in range(steps + 1):
        alpha = i / steps
        x = node1.x + alpha * dx
        y = node1.y + alpha * dy
        z = node1.z + alpha * dz

        if point_in_obstacle((x, y, z), obstacles, clearance):
            return False
    return True

def point_in_obstacle(point, obstacles, clearance):
    x, y, z = point
    epsilon = 1e-5
    for obs in obstacles:
        obs_center = obs["center"]
        obs_size = obs["size"]
        half_size = [(dim + clearance) / 2 for dim in obs_size]

        min_corner = [obs_center[i] - half_size[i] for i in range(3)]
        max_corner = [obs_center[i] + half_size[i] for i in range(3)]

        if all(min_corner[i] + epsilon <= point[i] <= max_corner[i] - epsilon for i in range(3)):
            return True
    return False

def check_collision(coords, obstacles, clearance=0.0):
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

def calculate_building_bounds(obstacles, floor_height, chosen_num_floors):
    # If no obstacles, define a default bounding box
    if len(obstacles) == 0:
        # Default bounds, e.g., a 10x10x(building_height) region around the origin
        # Adjust as needed if your scenario differs
        building_height = floor_height * chosen_num_floors
        return {
            "min_x": -5,
            "max_x": 5,
            "min_y": -5,
            "max_y": 5,
            "building_height": building_height
        }

    x_max = float('-inf')
    x_min = float('inf')
    y_max = float('-inf')
    y_min = float('inf')

    for obs in obstacles:
        center, size = obs["center"], obs["size"]
        half_size_x = size[0] / 2
        half_size_y = size[1] / 2

        x_max = max(x_max, center[0] + half_size_x)
        x_min = min(x_min, center[0] - half_size_x)
        y_max = max(y_max, center[1] + half_size_y)
        y_min = min(y_min, center[1] - half_size_y)

    building_height = floor_height * chosen_num_floors

    # Handle cases where bounds are still infinite (e.g., no obstacles)
    if math.isinf(x_min) or math.isinf(x_max) or math.isinf(y_min) or math.isinf(y_max):
        x_min, x_max = -5, 5
        y_min, y_max = -5, 5

    return {
        "min_x": x_min,
        "max_x": x_max,
        "min_y": y_min,
        "max_y": y_max,
        "building_height": building_height
    }

class RRTStarPlanner:
    def __init__(self, start, goal, obstacles, step_size=0.2, max_iters=2000, clearance=0.5, 
                 floor_height=2.0, chosen_num_floors=1, rewire_radius=1.0):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.step_size = step_size
        self.max_iters = max_iters
        self.clearance = clearance
        self.floor_height = floor_height
        self.chosen_num_floors = chosen_num_floors
        self.rewire_radius = rewire_radius

        self.bounds = calculate_building_bounds(self.obstacles, self.floor_height, self.chosen_num_floors)
        print(f"Building bounds: {self.bounds}")

        self.start_node = Node(*self.start)
        self.start_node.cost = 0.0
        self.goal_node = Node(*self.goal)
        self.tree = [self.start_node]

        self.iterations_done = 0
        self.path_found = False
        self.final_path = None

    def step_planner(self, iterations=100):
        for _ in range(iterations):
            if self.iterations_done >= self.max_iters:
                # Max iterations reached, stop.
                break

            self.iterations_done += 1
            rand_point = self.sample_random_point()

            # Check if rand_point is in obstacle
            if point_in_obstacle((rand_point.x, rand_point.y, rand_point.z), self.obstacles, self.clearance):
                continue

            # Find nearest node in the tree
            nearest = nearest_node(self.tree, rand_point)

            # Extend towards the random point
            new_node = self.extend_node(nearest, rand_point)
            if new_node is None:
                # Extension failed due to collision
                continue

            # Add new node to the tree
            self.tree.append(new_node)

            # Rewire step
            self.rewire(new_node)

            # Check if we reached the goal
            if distance(new_node, self.goal_node) < self.step_size:
                if line_collision_free(new_node, self.goal_node, self.obstacles, self.clearance):
                    self.goal_node.parent = new_node
                    self.goal_node.cost = new_node.cost + distance(new_node, self.goal_node)
                    self.tree.append(self.goal_node)
                    print("Goal reached by RRT*!")
                    self.path_found = True
                    self.final_path = self.extract_path()
                    break

            if self.iterations_done % 100 == 0:
                print(f"[DEBUG] Iteration {self.iterations_done}, Tree size: {len(self.tree)}")

    def sample_random_point(self):
        goal_bias = 0.05
        if random.random() < goal_bias:
            return Node(self.goal[0], self.goal[1], self.goal[2])
        else:
            # Use safe bounds
            return Node(
                random.uniform(self.bounds["min_x"], self.bounds["max_x"]),
                random.uniform(self.bounds["min_y"], self.bounds["max_y"]),
                random.uniform(0, self.bounds["building_height"])
            )

    def extend_node(self, nearest, rand_point):
        dx = rand_point.x - nearest.x
        dy = rand_point.y - nearest.y
        dz = rand_point.z - nearest.z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist == 0 or math.isnan(dist):
            return None

        scale = self.step_size / dist
        new_node = Node(
            nearest.x + dx * scale,
            nearest.y + dy * scale,
            nearest.z + dz * scale
        )

        # Check collision for the new edge
        if not line_collision_free(nearest, new_node, self.obstacles, self.clearance):
            return None

        new_node.cost = nearest.cost + distance(nearest, new_node)
        new_node.parent = nearest
        return new_node

    def rewire(self, new_node):
        near = near_nodes(self.tree, new_node, self.rewire_radius)
        for node_n in near:
            potential_cost = new_node.cost + distance(new_node, node_n)
            if potential_cost < node_n.cost and line_collision_free(new_node, node_n, self.obstacles, self.clearance):
                node_n.parent = new_node
                node_n.cost = potential_cost

    def extract_path(self):
        if self.goal_node.parent is None:
            return None
        path = []
        current = self.goal_node
        while current is not None:
            path.append((current.x, current.y, current.z))
            current = current.parent
        path.reverse()
        return path

    def is_path_found(self):
        return self.path_found

    def get_path(self):
        return self.final_path

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
        if math.isnan(dist):
            continue
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
    print("[DEBUG] Path visualized._visualize_path_as_line")
    print("[DEBUG] Stuck in visualize_path_as_line.")
    return

def visualize_random_point(point, radius=0.1, color=[0, 0, 1, 0.5]):
    sphere_visual = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=radius,
        rgbaColor=color
    )
    sphere_body = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=sphere_visual,
        basePosition=point
    )
    return sphere_body
