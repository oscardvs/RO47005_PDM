import pybullet as p
import pybullet_data
import time
from gym_pybullet_drones.envs.BaseAviary import BaseAviary


# Initialize PyBullet
p.connect(p.GUI)  # Use p.DIRECT for headless mode
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Create the environment (ground plane)
plane_id = p.loadURDF("plane.urdf")

# Add the drone URDF (use the assets folder in gym-pybullet-drones)
drone_start_pos = [0, 0, 0.1]  # Laying on the ground
drone_start_orientation = p.getQuaternionFromEuler([0, 0, 0])  # No rotation
drone_urdf_path = "/gym_pybullet_drones/assets/cf2x.urdf"  # Update this path if necessary
drone_id = p.loadURDF(drone_urdf_path, drone_start_pos, drone_start_orientation)

# Add two blocks of different colors and sizes
# Block 1: Red, Large
block1_position = [1, 0, 0.5]  # Position (x, y, z)
block1_size = [0.5, 0.5, 0.5]  # Size (length, width, height)
visual_shape1 = p.createVisualShape(shapeType=p.GEOM_BOX,
                                    halfExtents=block1_size,
                                    rgbaColor=[1, 0, 0, 1])  # Red
collision_shape1 = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=block1_size)
block1_id = p.createMultiBody(baseMass=0,  # Static object
                              baseCollisionShapeIndex=collision_shape1,
                              baseVisualShapeIndex=visual_shape1,
                              basePosition=block1_position)

# Block 2: Blue, Small
block2_position = [-1, 0, 0.25]  # Position (x, y, z)
block2_size = [0.25, 0.25, 0.25]  # Size (length, width, height)
visual_shape2 = p.createVisualShape(shapeType=p.GEOM_BOX,
                                    halfExtents=block2_size,
                                    rgbaColor=[0, 0, 1, 1])  # Blue
collision_shape2 = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=block2_size)
block2_id = p.createMultiBody(baseMass=0,  # Static object
                              baseCollisionShapeIndex=collision_shape2,
                              baseVisualShapeIndex=visual_shape2,
                              basePosition=block2_position)

# Run the simulation
print("Starting simulation. Press Ctrl+C to exit.")
while True:
    p.stepSimulation()
    time.sleep(1 / 240)  # Simulation step

# Disconnect PyBullet
p.disconnect()

