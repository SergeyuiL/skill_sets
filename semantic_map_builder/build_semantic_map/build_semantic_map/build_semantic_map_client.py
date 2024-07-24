#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from skillsets_msg.srv import SetSlam
from std_msgs.msg import Header
from geometry_msgs.msg import Pose

class SemanticMapBuilderClient(Node):

    def __init__(self):
        super().__init__('semantic_map_builder_client')
        self.cli = self.create_client(SetSlam, 'build_semantic_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetSlam.Request()

    def send_request(self):
        current_time = self.get_clock().now().to_msg()

        self.req.header = Header()
        self.req.header.stamp = current_time
        self.req.header.frame_id = 'map'

        self.req.cmd = 'build'
        self.req.mapdir = '/home/sg/workspace/top-down-map/map'
        self.req.filename = 'map_save'
        self.req.accurate_initial_pose = False
        self.req.initial_pose = Pose()
        self.req.initial_pose.position.x = 0.0
        self.req.initial_pose.position.y = 0.0
        self.req.initial_pose.position.z = 0.0
        self.req.initial_pose.orientation.x = 0.0
        self.req.initial_pose.orientation.y = 0.0
        self.req.initial_pose.orientation.z = 0.0
        self.req.initial_pose.orientation.w = 1.0

        self.future = self.cli.call_async(self.req)
        
def main(args=None):
    rclpy.init(args=args)
    client = SemanticMapBuilderClient()
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info('Service call failed %r' % (e,))
            else:
                client.get_logger().info('Success: %s' % response.success)
                client.get_logger().info(f'{response.message}')
            break

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
