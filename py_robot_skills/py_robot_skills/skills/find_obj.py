import numpy as np
from scipy.spatial.transform import Rotation as R

# PREDEFINED_POS = {
#     "cabinet": (np.array([1.999, 2.831, 0.255]), np.array([0.000, 0.000, 0.089, 0.996])),
#     "table": (np.array([-1.444, 4.062, 0.255]), np.array([0.000, 0.000, -0.219, 0.976])),
#     "bed": (np.array([0.892, -1.344, -0.242]), np.array([0.000, 0.000, 0.024, 1.000])),
#     "home": (np.array([-3.603, -2.838, 0.255]), np.array([0.000, 0.000, 0.685, 0.728])),
#     "water": (np.array([-6.663, 2.464, 0.255]), np.array([0.000, 0.000, 0.956, 0.295])),
#     "sofa": (np.array([-1.904, 3.169, 0.255]), np.array([0.000, 0.000, -0.265, 0.964])),
# }
# PREDEFINED_POS = {
#     "储藏柜": (np.array([1.999, 2.831, 0.255]), np.array([0.000, 0.000, 0.089, 0.996])),
#     "桌子": (np.array([-1.444, 4.062, 0.255]), np.array([0.000, 0.000, -0.219, 0.976])),
#     "床": (np.array([0.892, -1.344, -0.242]), np.array([0.000, 0.000, 0.024, 1.000])),
#     "home": (np.array([-3.603, -2.838, 0.255]), np.array([0.000, 0.000, 0.685, 0.728])),
#     "水": (np.array([-6.663, 2.464, 0.255]), np.array([0.000, 0.000, 0.956, 0.295])),
#     "sofa": (np.array([-1.904, 3.169, 0.255]), np.array([0.000, 0.000, -0.265, 0.964])),
# }

'''
transfer semantic map to dict
'''
# def trans_map_to_dict(sematic_map):
OBJECT_DIC = {
    "水": "bottle",
    "床": "bed",
    "沙发": "couch",
    "柜子": "refrigerator",
    "椅子": "chair",
    "餐桌": "dining table",
    "货架": "bed"
}

'''
    calculate the nearest point of contour to line 
    point: point of contour
    line_start: center of object's contour
    line_end: current pose
    all 2D
'''
def point_to_line_distance(point, line_start, line_end):
    
    line_vec = line_end - line_start
    point_vec = point - line_start
    line_len = np.linalg.norm(line_vec)
    line_unitvec = line_vec / line_len
    point_vec_scaled = point_vec / line_len
    t = np.dot(line_unitvec, point_vec_scaled)
    nearest = line_start + t * line_vec
    distance = np.linalg.norm(point - nearest)
    return distance, nearest

'''
    select the nearest point in the line and offset safe_distance+robot_radius
    contour: contour points of target
    cur_pos: current position, 2D array
    return target_pos, target_rot
    3D array, 4D quaternion
'''
def select_nav_target(contour, cur_pos, robot_radius=0.4, safe_distance=0.2):
        center = np.mean(contour, axis=0)
        vector_to_current = cur_pos - center
        unit_vector = vector_to_current / np.linalg.norm(vector_to_current)
        min_distance = float('inf')
        nearest_point_on_contour = None
        for point in contour:
            distance, nearest = point_to_line_distance(point, center, cur_pos)
            if distance < min_distance:
                if np.dot((point - center), vector_to_current) > 0:
                    min_distance = distance
                    nearest_point_on_contour = point

        navigation_point = nearest_point_on_contour + unit_vector * (robot_radius + safe_distance)

        yaw = np.fmod(np.arctan2(unit_vector[1], unit_vector[0]) + np.pi, 2 * np.pi)
        quaternion = R.from_euler('xyz', [0, 0, yaw]).as_quat()

        target_pos = np.array([navigation_point[0], navigation_point[1], 0.0])  
        target_rot = quaternion
        
        return target_pos, target_rot
    
'''
    return [pos, rotation]
'''
def find_object_from_map(object_name, sematic_map, container_name=None, object_index=None, current_pose=None):
    if object_name in OBJECT_DIC:
        object_class = OBJECT_DIC[object_name]
        try:
            if object_class not in sematic_map:
                raise ValueError(f"Class {object_class} is not in the map")
            contours = sematic_map[object_class]
            if len(contours) == 0:
                raise ValueError(f"No contour data for class {object_class}")
        except ValueError as e:
            print(e)
            return (None, None)
        # target_contour = np.array(contours[object_index])
        target_contour = np.array(contours[0])
    else:
        if container_name is not None and container_name in OBJECT_DIC:
            object_class = OBJECT_DIC[container_name]
            try:
                if object_class not in sematic_map:
                    raise ValueError(f"Class {object_class} is not in the map")
                contours = sematic_map[object_class]
                if len(contours) == 0:
                    raise ValueError(f"No contour data for class {object_class}")
            except ValueError as e:
                print(e)
                return (None, None)
            # target_contour = np.array(contours[object_index])
            target_contour = np.array(contours[1])
        else: 
            return (None, None)
    target_pos, target_rot = select_nav_target(target_contour, current_pose, robot_radius=0.3, safe_distance=0.2)
    return (target_pos, target_rot)

'''
    return [pos, rotation]
'''
def find_object_from_shelf(object_name, sematic_map):
    try:
        if object_name not in sematic_map["货架"]:
            raise ValueError(f"Class {object_name} is not in the shelf")
        target_pos = sematic_map["货架"][object_name][0]
        target_rot = sematic_map["货架"][object_name][1]
    except ValueError as e:
            print(e)
            return (None, None)
    return (np.array(target_pos), np.array(target_rot))

'''
    raw code frame
'''
# def find_object_from_map(object_name, sematic_map, container_name=None):
#     # return [pos, rotation]
#     if object_name in PREDEFINED_POS:
#         pose = PREDEFINED_POS[object_name]
#         return pose
#     else:
#         if container_name is not None and container_name in PREDEFINED_POS:
#             pose = PREDEFINED_POS[container_name]
#             return pose
#         else: 
#             return (None, None)