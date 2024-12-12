import pybullet as p
import pybullet_data
import time

# Connect to PyBullet with GUI enabled
physicsClient = p.connect(p.GUI, key=12345)

# Set the search path for PyBullet (for plane.urdf and other assets)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a ground plane
plane_id = p.loadURDF("plane.urdf")

# Apply gravity
p.setGravity(0, 0, -9.8)


# Load the building environment URDF
env_start_pos = [0, 0, 0.2]  # Adjusted to account for hovering offset
env_start_orn = p.getQuaternionFromEuler([0, 0, 0])
env_id = p.loadURDF("flat_drone_env.urdf", env_start_pos, env_start_orn, useFixedBase=True)

# Set transparency for the building
visual_shapes = p.getVisualShapeData(env_id)
if visual_shapes:
    for shape in visual_shapes:
        shape_index = shape[1]  # Visual shape index
        p.changeVisualShape(env_id, shape_index, rgbaColor=[1, 1, 1, 0.5])  # RGBA: last value is alpha (transparency)
else:
    print("No visual shapes found for the building.")


# Load the drone model
drone_start_pos = [1, 1, 1]
drone_start_orn = p.getQuaternionFromEuler([0, 0, 0])
drone_id = p.loadURDF("/home/oskrt/Documents/RO47005_PDM/dev_test_urdf/gym_pybullet_drones/assets/racer.urdf", drone_start_pos, drone_start_orn, globalScaling=0.5)

# Adjust camera to focus on the environment
p.resetDebugVisualizerCamera(cameraDistance=10, cameraYaw=50, cameraPitch=-30, cameraTargetPosition=[0, 0, 0])


p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
# Enable mouse picking and keyboard shortcuts for viewpoint adjustment
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)  # Enable GUI controls
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)  # Enable rendering
p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 1)  # Allow mouse interaction
p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 1)  # Enable keyboard shortcuts


# Keep the simulation running
while True:
    p.stepSimulation()
    time.sleep(1.0 / 240.0)

# Disconnect PyBullet when exiting (this line won't execute unless the loop is terminated)
p.disconnect()
