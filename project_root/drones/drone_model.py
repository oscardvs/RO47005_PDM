import pybullet as p

class Drone:
    def __init__(self, start_position, start_orientation):
        self.drone_id = p.loadURDF("path/to/drone.urdf", start_position, start_orientation)
    
    def apply_control(self, cmd):
        # Convert cmd (like desired vx, vy) to actual motor commands
        # p.setJointMotorControlArray(...)
        pass

    def get_position(self):
        pos, ori = p.getBasePositionAndOrientation(self.drone_id)
        return pos, ori
