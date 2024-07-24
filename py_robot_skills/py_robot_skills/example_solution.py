import rclpy
from rclpy.node import Node
from skill_lib import RobotSkillSets
from base import Base

from rclpy.executors import MultiThreadedExecutor
import numpy as np

class Demo(Node):
    def __init__(self):
        super().__init__("demo_test")
        self.skill = RobotSkillSets()
        self.base = self.skill.base_handler

    def demo_test1(self):
        # self.skill.find_object("water", "shelf", "in")
        # self.skill.find_object("table", None, None)
        self.skill.go_home(None, None, None)
    
    def demo_test2(self):
        self.skill.find_object("sofa", None, None)
        self.skill.find_object("water", "shelf", None)
        self.skill.find_object("sofa", None, None)
        self.skill.find_object("cabinet", None, None)
        self.skill.find_object("table", None, None)
        self.skill.go_home(None, None, None)
        
    def demo_test3(self):
        test_pos = np.array([0.2, 0., 0.])
        test_rot = np.array([0., 0., 0., 1.])
        self.base.move_to(test_pos, test_rot, 'robot')
    
    def demo_spin(self):
        self.base.turn_around(0.2)
        self.get_logger().info("spin test successfully!")
    
    def demo_moveforward(self):
        self.base.move_forward(0.5, speed=0.5)
        self.get_logger().info("Moving forward successfully!")
        

def main(args=None):
    rclpy.init(args=args)
    node = Demo()
    # node.demo_test1()
    # node.demo_test2()
    # node.demo_spin()
    # node.demo_moveforward()
    node.demo_test3()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node.skill)
    executor.add_node(node.base)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        node.skill.destroy_node()
        node.base.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()