class TrajectoryFollower:
    def __init__(self, drone, pid_controller, path, speed=0.5):
        self.drone = drone
        self.pid = pid_controller
        self.path = path
        self.speed = speed
        self.current_index = 0

    def get_next_control_input(self):
        # Get drone current position
        # Compute error to the next waypoint
        # Use PID to get control command
        # Move to next waypoint if close enough
        return control_cmd
