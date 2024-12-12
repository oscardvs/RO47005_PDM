import pybullet as p
import time

# Connect to the existing PyBullet simulation (shared server)
physicsClient = p.connect(p.SHARED_MEMORY, key=12345)

if physicsClient < 0:
    print("Failed to connect to the simulation. Ensure 'main_simulation.py' is running.")
    exit()

# Get the drone's unique ID (adjust this if the ID is dynamic)
drone_id = 1  # Use the correct ID for your drone from main_simulation.py
drone_start_orn = p.getQuaternionFromEuler([0, 0, 0])

# Define start and end points for the trajectory
point_A = [0, -0.2, 2]  # Starting point
point_B = [5, 5, 2]     # Ending point
num_steps = 240         # Total steps for the motion (adjust speed)

# Move the drone from A to B
for step in range(num_steps):
    t = step / num_steps
    drone_position = [
        point_A[0] * (1 - t) + point_B[0] * t,
        point_A[1] * (1 - t) + point_B[1] * t,
        point_A[2] * (1 - t) + point_B[2] * t
    ]
    p.resetBasePositionAndOrientation(drone_id, drone_position, drone_start_orn)
    time.sleep(1.0 / 240.0)

# Disconnect the shared memory when done
p.disconnect()
