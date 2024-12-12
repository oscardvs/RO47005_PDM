import pybullet as p

def load_building(urdf_path):
    building_id = p.loadURDF(urdf_path, basePosition=[0,0,0], useFixedBase=True)
    return building_id
