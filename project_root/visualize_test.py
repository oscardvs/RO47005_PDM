import pybullet as p
import pybullet_data
import time
import random
import math
from environment.obstacle_definition import generate_obstacles, visualize_obstacles
#from planning.rrt import generate_path_rrt, interpolate_path, visualize_path_as_line, check_collision
from planning.rrt_star import generate_path_rrt_star, visualize_path_as_line, check_collision

# Connect to PyBullet with GUI enabled
physicsClient = p.connect(p.GUI, key=12345)

# Set the search path for PyBullet (for plane.urdf and other assets)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a ground plane
plane_id = p.loadURDF("plane.urdf")

# Make the ground slightly transparent
p.changeVisualShape(plane_id, -1, rgbaColor=[1, 1, 1, 0.7])

# Apply gravity
p.setGravity(0, 0, -9.8)

def get_yaw_quaternion(degrees):
    radians = degrees * (math.pi / 180.0)
    return p.getQuaternionFromEuler([0, 0, radians])

# PARAMETERS FOR CITY BUILDING
NUM_BUILDINGS = 10
ROOM_URDF_PATH = "environment/room.urdf"  
ROOM_HEIGHT = 2.0  # Approximate height of each room
CITY_AREA = 75.0   # 75x75 meters
HALF_AREA = CITY_AREA / 2.0

top_level_rooms = []      # Will store tuples of (room_id, [x,y,z], building_index)
buildings_info = []       # Will store tuples of (num_floors, [base_x, base_y, 0])

# Build a city:
for b in range(NUM_BUILDINGS):
    # Random position for the building base
    building_x = random.uniform(-HALF_AREA, HALF_AREA)
    building_y = random.uniform(-HALF_AREA, HALF_AREA)

    # For debugging, use fewer floors:
    num_floors = random.randint(5, 10)

    # Build floors for this building
    for floor in range(num_floors):
        floor_z = floor * ROOM_HEIGHT
        yaw_degrees = floor * 90
        room_orientation = get_yaw_quaternion(yaw_degrees)

        room_id = p.loadURDF(
            ROOM_URDF_PATH, 
            [building_x, building_y, floor_z], 
            room_orientation, 
            useFixedBase=True
        )

        # Make the room slightly transparent
        visual_shapes = p.getVisualShapeData(room_id)
        if visual_shapes:
            for shape in visual_shapes:
                shape_index = shape[1]
                p.changeVisualShape(room_id, shape_index, rgbaColor=[1, 1, 1, 0.4])

        # If this is the top level (last floor), store its ID and position along with building index
        if floor == num_floors - 1:
            top_level_rooms.append((room_id, [building_x, building_y, floor_z], b))

    # Store building info
    buildings_info.append((num_floors, [building_x, building_y, 0]))

# Randomly pick one building's top floor and highlight it in red
highlighted_room, building_base_pos, highlighted_building_index = random.choice(top_level_rooms)

# Highlight the selected top floor
visual_shapes = p.getVisualShapeData(highlighted_room)
if visual_shapes:
    for shape in visual_shapes:
        shape_index = shape[1]
        p.changeVisualShape(highlighted_room, shape_index, rgbaColor=[1, 0, 0, 0.7])

# Retrieve the chosen building's info
chosen_num_floors, chosen_building_base = buildings_info[highlighted_building_index]

# Spawn the pink ball at the ground floor of the selected building as the "drone" start
drone_start_pos = [chosen_building_base[0] + 1, chosen_building_base[1] - 1, 1]
drone_id = p.createMultiBody(
    baseMass=0,  # Static object
    baseVisualShapeIndex=p.createVisualShape(
        p.GEOM_SPHERE,
        radius=0.3,
        rgbaColor=[1, 0, 1, 1] 
    ),
    basePosition=drone_start_pos
)

# The goal is also a pink ball placed at the same X,Y as start but at the top floor
top_floor_z = chosen_num_floors * ROOM_HEIGHT -0.5
goal_pos = [drone_start_pos[0], drone_start_pos[1], top_floor_z ]

goal_ball_radius = 0.3
goal_collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=goal_ball_radius)
goal_visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=goal_ball_radius, rgbaColor=[1, 0, 1, 1])
goal_id = p.createMultiBody(baseMass=0, 
                            baseCollisionShapeIndex=goal_collision_shape, 
                            baseVisualShapeIndex=goal_visual_shape, 
                            basePosition=goal_pos)


# Retrieve and store absolute coordinates
drone_absolute_position, _ = p.getBasePositionAndOrientation(drone_id)
ball_absolute_position, _ = p.getBasePositionAndOrientation(goal_id)

print("[DEBUG] Drone Absolute Position:", drone_absolute_position)
print("[DEBUG] Goal Absolute Position:", ball_absolute_position)

# Adjust camera
p.resetDebugVisualizerCamera(cameraDistance=50, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])

p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)

#-------------------------------------------------------------
# Path Planning
# Generate obstacles for the chosen building (from ground floor to top floor)
print("[DEBUG] Generating obstacles...")
# Generate and visualize obstacles
obstacles = generate_obstacles(
    num_floors=chosen_num_floors,
    urdf_file=ROOM_URDF_PATH,
    floor_height=ROOM_HEIGHT,
    rotation_step=90,
    building_base=chosen_building_base
)

# Debug: Print obstacle details
print("[DEBUG] Generated Obstacles:")
for obs in obstacles:
    print(f"  Center: {obs['center']}, Size: {obs['size']}")

# Visualize the obstacles
visualize_obstacles(obstacles)

start = drone_absolute_position
goal = ball_absolute_position

# Ensure the start and goal positions are collision-free
if check_collision(start, obstacles):
    print("[ERROR] Start position is in collision. Please choose a valid start position.")
    p.disconnect()
    exit()

if check_collision(goal, obstacles):
    print("[ERROR] Goal position is in collision. Please choose a valid goal position.")
    p.disconnect()
    exit()

print("[DEBUG] Start and Goal positions are collision-free. Proceeding with path planning.")

####################DEBUG#########################
#from the visualization of the environment, I know all of the above is correct, points are beeing 
#generated (maybe the height still needs to be ajusted) and collision is checked correctly.




###############DEBUG####################3
#How are obstacles represented??
#we have to generate path using rrt_star. paramters we can use are;
#start, goal, obstacles, other variables defined here and needed inside 

# Start path planning with adjusted parameters




#4 floors you need at LEAST 6000 iterations to get a path (with 0.2 step size and 0.5 clearance)
#4 floors worked with 5000 iterations and 0.5 step size and 0.5 clearance - takes a lon time
#print("[DEBUG] Starting RRT Path Planning...")
#path = generate_path_rrt(
    #start, 
    #goal, 
    #obstacles, 
    #step_size=0.5, 
    #max_iters=5000, 
    #clearance=0.5, 
    #floor_height=ROOM_HEIGHT,
    #chosen_num_floors=chosen_num_floors
#)

path = generate_path_rrt_star(
    start=start,
    goal=goal,
    obstacles=obstacles,
    step_size=0.3,
    max_iters=50000,
    clearance=0.2,
    floor_height=ROOM_HEIGHT,
    chosen_num_floors=chosen_num_floors,
    rewire_radius=0.3
)
print("[DEBUG] Starting RRT* Path Planning...")

if path and len(path) > 1:
    print(f"[DEBUG] Path Computed! Total Waypoints: {len(path)}")
    # Optional: Interpolate path
    # path = interpolate_path(path, interp_step=0.1)
    # print(f"[DEBUG] Interpolated Path Total Waypoints: {len(path)}")
    visualize_path_as_line(path, color=[1, 0.5, 0], line_width=3)  # Orange line
else:
    print("[DEBUG] No Path Found or only one waypoint in path!")

# ------------------------------------------------------------
# Keep the simulation running
while True:
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

p.disconnect()
