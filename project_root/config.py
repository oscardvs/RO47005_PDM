class Config:
    GUI = True
    START_POS = [0, 0, 1.0]  # x, y, z
    START_ORI = [0, 0, 0, 1]
    GOAL_POS = [10, 5, 1.0]
    RRT_STEP_SIZE = 0.5
    RRT_MAX_ITERS = 1000
    CLEARANCE = 0.3

    PID_KP = 1.0
    PID_KI = 0.0
    PID_KD = 0.1

    DRONE_SPEED = 0.5
    MAX_STEPS = 10000
