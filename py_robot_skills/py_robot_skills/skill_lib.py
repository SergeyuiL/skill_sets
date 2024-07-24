import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import json

import sys
sys.path.append('./skills')
from .skills.find_obj import find_object_from_map, find_object_from_shelf
from .base import Base

class RobotSkillSets(Node):
    def __init__(self) -> None:
        super().__init__("robot_skill")
        self.current_pose = None
        self.task_start_base_pose = None
        self.task_start_arm_pose = None
        self.state = {"task_name": None, "task_status": 0}
        self.base_handler = Base(goal_action_name="/navigate_to_pose", spin_action_name="/spin", forward_action_name="/drive_on_heading",
                                 base_frame="base_link", world_frame="map")
        self.arm_handler = None
        # self.func_map = {"find": self.find_object,
        #                  "go_home": self.go_home,
        #                  "place": self.place_object,
        #                  "grasp": self.grasp}
        self.func_map = {"找到物体": self.find_object,
                         "回家": self.go_home,
                         "放置物体": self.place_object,
                         "拿起物体": self.grasp}
        with open("/home/nvidia/walle_ws/src/py_robot_skills/maps/object_info.json", 'r') as f:
            self.sematic_map = json.load(f)
        # self.sematic_map = None

    # def find_object(self, object_name, container_name, object_index, relationship):
    def find_object(self, object_name, container_name, relationship):
        # relationship will not be used
        # self.task_start_base_pose = self._get_current_base_pose()
        self._update_status("find", 1)
        current_position = self.base_handler.position[0:2]
        if container_name == "货架":
            target_pose = find_object_from_shelf(object_name=object_name, sematic_map=self.sematic_map)
        else:
            target_pose = find_object_from_map(object_name=object_name, sematic_map=self.sematic_map, container_name=container_name, current_pose=current_position)
        if target_pose[0] is None:
            success = False
            info = "cannot find the object {}".format(object_name)
            self.get_logger().info(info)
            return success, info
        self.get_logger().info(f"Find object: {object_name}, Current position: {current_position}, Target position: {target_pose}")
        success, info = self.base_handler.move_to(target_pose[0], target_pose[1], frame_id="world")
        # success = True
        # info = ''
        return success, info
    
    def go_home(self, object_name, container_name, relationship):
        self._update_status("go_home", 1)
        # parameter will not be used
        target_pose = find_object_from_map(object_name="home", sematic_map=self.sematic_map, container_name=container_name)
        if target_pose[0] is None:
            success = False
            info = "cannot find the home"
            self.get_logger().info(info)
            return success, info
        success, info = self.base_handler.move_to(target_pose[0], target_pose[1], frame_id="world")
        return success, info

    def place_object(self, object_name, container_name, relationship):
        self._update_status("place_object", 1)
        valid_relationships = ["in", "over", "nearby"]
        # print(relationship)
        if relationship not in valid_relationships:
            success = False
            # print(relationship)
            info = "wrong relationship {}".format(relationship)
            return success, info
        success = True
        info = " "
        return success, info
    
    def grasp(self, object_name, container_name, relationship):
        self._update_status("grasp", 1)
        # relationship will not be used
        success = True
        info = " "
        return success, info
    
    def reset(self):
        "reset all actuator"
        self.target_start_pose = None

        self._update_status(None, 0)
    
    def idle_status(self):
        self._update_status(None, 0)
    

    def _get_current_base_pose(self):
        pose = (self.base_handler.position, self.base_handler.rotation)
        return pose
    
    def _get_current_arm_pose(self):
        pose = None
        return pose
    
    def _update_status(self, task_name, task_status):
        self.state['task_name'] = task_name
        self.state["task_status"] = task_status