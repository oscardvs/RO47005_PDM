import math
import xml.etree.ElementTree as ET
import numpy as np
import pybullet as p

def euler_to_rotation_matrix(rpy):
    roll, pitch, yaw = rpy
    cx, cy, cz = math.cos(roll), math.cos(pitch), math.cos(yaw)
    sx, sy, sz = math.sin(roll), math.sin(pitch), math.sin(yaw)
    R = np.array([
        [cy*cz, cz*sx*sy - sz*cx, cz*cx*sy + sz*sx],
        [cy*sz, sz*sx*sy + cz*cx, sz*cx*sy - cz*sx],
        [-sy,   cy*sx,            cy*cx]
    ])
    return R

def parse_collision_boxes(urdf_file):
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    boxes = []
    for link in root.findall("link"):
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            box = geometry.find("box")
            if box is not None:
                size = list(map(float, box.attrib["size"].split()))
                origin_elem = collision.find("origin")
                if origin_elem is not None:
                    origin = list(map(float, origin_elem.attrib["xyz"].split()))
                    rpy = list(map(float, origin_elem.attrib["rpy"].split()))
                else:
                    origin = [0.0, 0.0, 0.0]
                    rpy = [0.0, 0.0, 0.0]
                boxes.append((size, origin, rpy))
    return boxes

def generate_obstacles(num_floors, urdf_file, floor_height, rotation_step=90, building_base=[0, 0, 0]):
    collision_boxes = parse_collision_boxes(urdf_file)
    all_obstacles = []

    for floor in range(num_floors):
        print("[DEBUG] Floor:", floor)
        floor_yaw = math.radians(rotation_step * floor)
        floor_rotation_rpy = [0.0, 0.0, floor_yaw]
        R_floor = euler_to_rotation_matrix(floor_rotation_rpy)
        print("[DEBUG] R_floor:\n", R_floor)
        floor_z = floor * floor_height

        for size, origin, rpy in collision_boxes:
            R_obs = euler_to_rotation_matrix(rpy)
            print("[DEBUG] R_obs:\n", R_obs)
            
            # size = [sx, sy, sz]
            sx, sy, sz = size
            hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0

            # Local corners of the box (centered at [0,0,0])
            local_corners = np.array([
                [-hx, -hy, -hz],
                [-hx, -hy,  hz],
                [-hx,  hy, -hz],
                [-hx,  hy,  hz],
                [ hx, -hy, -hz],
                [ hx, -hy,  hz],
                [ hx,  hy, -hz],
                [ hx,  hy,  hz],
            ])
            print("[DEBUG] Local_corners:\n", local_corners)

            # Apply obstacle rotation and translation: X_room = R_obs*X_local + origin
            room_corners = np.dot(local_corners, R_obs.T) + np.array(origin)
            print("[DEBUG] room_corners:\n", room_corners)

            # Apply floor rotation and building base translation
            # X_world = building_base + [0,0,floor_z] + R_floor * X_room
            world_corners = np.dot(room_corners, R_floor.T) + np.array([building_base[0], building_base[1], building_base[2] + floor_z])
            print("[DEBUG] world_corners:\n", world_corners)

            min_coords = world_corners.min(axis=0)
            max_coords = world_corners.max(axis=0)

            final_center = 0.5 * (min_coords + max_coords)
            final_size = max_coords - min_coords

            all_obstacles.append({
                "center": final_center.tolist(),
                "size": final_size.tolist()
            })

    return all_obstacles

def visualize_obstacles(obstacles):
    for obs in obstacles:
        center = obs["center"]
        size = obs["size"]

        # Optional: Color by height or size
        color = [1, 0, 0, 0.5] if size[2] > 0.5 else [0, 1, 0, 0.8]

        visual_shape = p.createVisualShape(
            shapeType=p.GEOM_BOX,
            halfExtents=[size[0] / 2, size[1] / 2, size[2] / 2],
            rgbaColor=color
        )

        p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=visual_shape,
            basePosition=center
        )

        # (Optional) Add debug lines for edges
        min_corner = [center[i] - size[i] / 2 for i in range(3)]
        max_corner = [center[i] + size[i] / 2 for i in range(3)]

        corners = [
            [min_corner[0], min_corner[1], min_corner[2]],
            [max_corner[0], min_corner[1], min_corner[2]],
            [max_corner[0], max_corner[1], min_corner[2]],
            [min_corner[0], max_corner[1], min_corner[2]],
            [min_corner[0], min_corner[1], max_corner[2]],
            [max_corner[0], min_corner[1], max_corner[2]],
            [max_corner[0], max_corner[1], max_corner[2]],
            [min_corner[0], max_corner[1], max_corner[2]],
        ]

        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),
            (4, 5), (5, 6), (6, 7), (7, 4),
            (0, 4), (1, 5), (2, 6), (3, 7),
        ]

        for edge in edges:
            p.addUserDebugLine(corners[edge[0]], corners[edge[1]], lineColorRGB=color[:3], lineWidth=1)