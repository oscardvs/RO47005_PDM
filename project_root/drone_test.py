import time
import numpy as np
from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.utils import sync
import pybullet as p
import pybullet_data
from planning.rrt_star_incremental import RRTStarPlanner, visualize_path_as_line, check_collision, interpolate_path

def main():
    #### Parameters ####
    DRONE_MODEL = DroneModel.CF2X
    PHYSICS = Physics.PYB
    GUI = True
    SIM_FREQ = 240
    CTRL_FREQ = 48  # Control frequency
    PATH_DURATION_SEC = 10
    HOVER_DURATION_SEC = 5

    #### Define Start Position and Path ####
    start_pos = [0, 0, 1.0]
    path = [
        [0, 0, 1.0],
        [1, 0, 1.5],
        [1, 1, 1.5],
        [0, 1, 1.0],
        [0, 0, 1.0]
    ]

    # Interpolate the path for smoother transitions
    path = interpolate_path(path, interp_step=0.1)

    #### Create Environment ####
    env = CtrlAviary(
        drone_model=DRONE_MODEL,
        num_drones=1,
        initial_xyzs=np.array([start_pos]),
        initial_rpys=np.array([[0, 0, 0]]),
        physics=PHYSICS,
        neighbourhood_radius=10,
        pyb_freq=SIM_FREQ,
        ctrl_freq=CTRL_FREQ,
        gui=GUI,
        record=False,
        obstacles=False,
        user_debug_gui=False
    )

    pid = DSLPIDControl(drone_model=DRONE_MODEL)  # Initialize PID controller
    pid.setPIDCoefficients(
        p_coeff_pos=np.array([0.7, 0.7, 0.7]),  # Increased Position Proportional Gains
        i_coeff_pos=np.array([0.0, 0.0, 0.0]),  # Position Integral Gains
        d_coeff_pos=np.array([0.1, 0.1, 0.2]),  # Increased Position Derivative Gains
        p_coeff_att=np.array([0.1, 0.1, 0.1]),  # Attitude Proportional Gains
        i_coeff_att=np.array([0.0, 0.0, 0.0]),  # Attitude Integral Gains
        d_coeff_att=np.array([0.05, 0.05, 0.05])  # Attitude Derivative Gains
    )

    #### Get PyBullet Client for Debug ####
    PYB_CLIENT = env.getPyBulletClient()
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=PYB_CLIENT)
    plane_id = p.loadURDF("plane.urdf", physicsClientId=PYB_CLIENT)
    p.changeVisualShape(plane_id, -1, rgbaColor=[1, 1, 1, 0.7], physicsClientId=PYB_CLIENT)
    p.setGravity(0, 0, -9.8, physicsClientId=PYB_CLIENT)

    #### Path Following Test ####
    print("[DEBUG] Starting path-following test...")
    action = np.zeros((1, 4))
    ctrl_every_n_steps = int(SIM_FREQ / CTRL_FREQ)
    current_target_index = 0
    path_follow_start = time.time()

    for t in range(int(PATH_DURATION_SEC * SIM_FREQ)):
        obs, _, _, _, _ = env.step(action)

        # Update the camera to follow the drone
        drone_pos = obs[0][:3]
        p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=drone_pos)

        # Apply control at the control frequency
        if t % ctrl_every_n_steps == 0:
            drone_state = obs[0]
            drone_pos = drone_state[:3]
            target_pos = np.array(path[current_target_index])
            dist_to_target = np.linalg.norm(drone_pos - target_pos)

            # Check if we reached the current waypoint
            if dist_to_target < 0.1 and current_target_index < len(path) - 1:
                current_target_index += 1
                target_pos = np.array(path[current_target_index])
                print(f"[DEBUG] Moving to next waypoint: {current_target_index}, New Target: {target_pos}")

            # Compute new action using PID
            new_action, _, _ = pid.computeControlFromState(
                control_timestep=1.0 / CTRL_FREQ,
                state=drone_state,
                target_pos=target_pos,
                target_rpy=[0, 0, 0]
            )

            # Clip actions to avoid excessive thrust
            MIN_RPM = 1000
            MAX_RPM = 20000
            action = np.clip([new_action], MIN_RPM, MAX_RPM)

            # Print debug info
            print(f"[DEBUG] t={t/SIM_FREQ:.2f}s Pos={drone_pos}, Target={target_pos}, Dist={dist_to_target:.2f}, Action={action}")

        if GUI:
            sync(t, path_follow_start, 1.0 / SIM_FREQ)

        # Check if the path is complete
        if current_target_index == len(path) - 1 and dist_to_target < 0.1:
            print("[DEBUG] Path completed!")
            break

    #### Hover at Final Position ####
    print("[DEBUG] Hovering at the final position...")
    target_pos = np.array(path[-1])  # Hover at the final position
    hover_start_time = time.time()

    for t in range(int(HOVER_DURATION_SEC * SIM_FREQ)):
        obs, _, _, _, _ = env.step(action)

        # Update the camera to follow the drone
        drone_pos = obs[0][:3]
        p.resetDebugVisualizerCamera(cameraDistance=5, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=drone_pos)

        # Apply control at the control frequency
        if t % ctrl_every_n_steps == 0:
            drone_state = obs[0]
            drone_pos = drone_state[:3]

            # Compute new action for hovering
            new_action, _, _ = pid.computeControlFromState(
                control_timestep=1.0 / CTRL_FREQ,
                state=drone_state,
                target_pos=target_pos,
                target_rpy=[0, 0, 0]
            )

            # Clip actions to avoid excessive thrust
            action = np.clip([new_action], MIN_RPM, MAX_RPM)

            # Print debug info
            dist_to_target = np.linalg.norm(drone_pos - target_pos)
            print(f"[DEBUG] t={t/SIM_FREQ:.2f}s Pos={drone_pos}, Hover Target={target_pos}, Dist={dist_to_target:.2f}, Action={action}")

        if GUI:
            sync(t, hover_start_time, 1.0 / SIM_FREQ)

    print("[DEBUG] Hover test completed.")
    env.close()

if __name__ == "__main__":
    main()
