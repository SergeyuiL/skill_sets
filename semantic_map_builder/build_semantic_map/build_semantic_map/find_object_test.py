import numpy as np
import json

def find_object_from_shelf(object_name, sematic_map):
    try:
        if object_name not in sematic_map["货架"]:
            raise ValueError(f"Class {object_name} is not in the shelf")
        object_tag = sematic_map["货架"][object_name]["tag"]
        object_layer = sematic_map["货架"][object_name]["layer"]
        target_pos = sematic_map["货架"][object_tag][0]
        target_rot = sematic_map["货架"][object_tag][1]
    except ValueError as e:
            print(e)
            return (None, None)
    return (np.array(target_pos), np.array(target_rot)), object_layer

def main():
    map_path = "/home/drl/dev_ws/src/skill_sets/py_robot_skills/maps/object_info.json"
    with open(map_path, 'r') as f:
            sematic_map = json.load(f)
    object_name = "可乐"
    target_pose, object_layer = find_object_from_shelf(object_name, sematic_map) 
    print(f"Find object name: {object_name}; Target pose: {target_pose}; Target object layer: {object_layer}") 
    
if __name__ == "__main__":
    main()