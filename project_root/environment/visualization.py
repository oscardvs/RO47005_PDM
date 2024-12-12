import pybullet as p
from planning.geometry_tools import path_to_line_points

def visualize_path(path, color=[1,0,0]):
    line_points = path_to_line_points(path)
    for i in range(len(line_points)-1):
        p.addUserDebugLine(line_points[i], line_points[i+1], color, 2)
