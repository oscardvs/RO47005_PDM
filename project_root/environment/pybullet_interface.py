import pybullet as p
import pybullet_data

def init_simulation(gui=True):
    if gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    return p.getConnectionInfo()

def step_simulation():
    p.stepSimulation()
