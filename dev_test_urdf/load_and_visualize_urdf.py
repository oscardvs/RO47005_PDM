#token repo RO47005: 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

import pybullet as p
import pybullet_data
import time
import os

# Initialize PyBullet in GUI mode
p.connect(p.GUI)

# Set PyBullet's data path for URDFs
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load the drone URDF
drone_urdf_path = "/home/oskrt/gym-pybullet-drones/gym_pybullet_drones/assets/racer.urdf"  # Update with the correct path
if not os.path.exists(drone_urdf_path):
    raise FileNotFoundError(f"URDF file not found at {drone_urdf_path}")

# Load the URDF into the simulation
p.loadURDF(drone_urdf_path, basePosition=[0, 0, 0], useFixedBase=True)

# Set gravity to 0 for visualization only
p.setGravity(0, 0, 0)

# Keep the simulation open
while True:
    time.sleep(5)  # Adjust the refresh rate as needed

