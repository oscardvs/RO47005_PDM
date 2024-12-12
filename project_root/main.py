from config import Config
from environment.pybullet_interface import init_simulation, step_simulation
from environment.environment_setup import load_building
from environment.obstacle_definitions import get_obstacles
from project_root.planning.rrt import generate_path
from control.pid_controller import PIDController
from control.trajectory_follower import TrajectoryFollower
from drones.drone_model import Drone

def main():
    # Load configuration
    cfg = Config()

    # Initialize simulation environment
    physics_client = init_simulation(gui=cfg.GUI)

    # Load building and obstacles
    load_building("environment/model.urdf")
    obstacles = get_obstacles()

    # Initialize Drone
    drone = Drone(start_position=cfg.START_POS, start_orientation=cfg.START_ORI)

    # Plan path
    path = generate_path(start=cfg.START_POS, goal=cfg.GOAL_POS, obstacles=obstacles, 
                         step_size=cfg.RRT_STEP_SIZE, max_iters=cfg.RRT_MAX_ITERS, 
                         clearance=cfg.CLEARANCE)

    # Set up controller
    pid = PIDController(kp=cfg.PID_KP, ki=cfg.PID_KI, kd=cfg.PID_KD)
    follower = TrajectoryFollower(drone, pid, path, speed=cfg.DRONE_SPEED)

    # Control loop
    for _ in range(cfg.MAX_STEPS):
        cmd = follower.get_next_control_input()
        drone.apply_control(cmd)
        step_simulation()
        # Optional: log, visualize, or break if goal reached.

    # End of main
    print("Navigation complete.")

if __name__ == "__main__":
    main()
