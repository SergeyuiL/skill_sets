#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from skillsets_msg.srv import SetSlam
from .mapbuilder import MapBuilder
import numpy as np
import os
import json

class SemanticMapBuilderService(Node):

    def __init__(self):
        super().__init__('semantic_map_builder_service')
        self.srv = self.create_service(SetSlam, 'build_semantic_map', self.build_map_callback)
        self.trans_camera_base = [0.08278859935292791, -0.03032243564439939, 1.0932014910932797]
        self.quat_camera_base = [-0.48836894018639176, 0.48413701319615116, -0.5135400532533373, 0.5132092598729002]
        self.map2base_link_origin = np.array([[-0.347, 0.938, 0.000, 2.229],
                                    [-0.938, -0.347, -0.000, 9.121],
                                    [-0.000, 0.000, 1.000, 0.255],
                                    [0.000, 0.000, 0.000, 1.000]])
        self.is_map_origin = True

    def build_map_callback(self, request, response):
        if request.cmd != 'build':
            response.success = False
            response.message = "Invalid command. Only 'build' command is supported."
            return response

        map_dir = request.mapdir
        filename = request.filename
        
        if map_dir is None or filename is None:
            response.success = False
            response.message = "Invalid map_dir or filename. "
            return response
        
        object_info_path = os.path.join("/home/nvidia/skillsets_ws/src/skill_sets/py_robot_skills/maps", filename + ".json")

        try:
            self.get_logger().info("Start to build semantic map...")
            ssmap = MapBuilder(map_dir, self.trans_camera_base, self.quat_camera_base, self.map2base_link_origin, is_map_origin=self.is_map_origin)
            ssmap.data_load()
            ssmap.seg_rgb(confidence=0.5)
            final_object_info = ssmap.build_2dmap(max_depth=6000, DBSCAN_eps=0.1, DBSCAN_min_samples=190, voxel_size=0.05)
            with open(object_info_path, 'w') as f:
                json.dump(final_object_info, f, indent=4)

            response.success = True
            response.message = f"Map built and saved to {object_info_path}"
        except Exception as e:
            response.success = False
            response.message = f"Failed to build map: {e}"

        return response

def main(args=None):
    rclpy.init(args=args)
    service = SemanticMapBuilderService()
    service.get_logger().info("Service node is running...Waiting for calling...")
    rclpy.spin(service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
