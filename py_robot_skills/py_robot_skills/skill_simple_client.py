import rclpy
from rclpy.node import Node
from rclpy.client import Client
from robot_skills.srv import Skill
from example_interfaces.srv import AddTwoInts

class SkillClient(Node):
    def __init__(self):
        super().__init__("skill_client")
        self.client = self.create_client(Skill, "skill")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        
        self.add_client = self.create_client(AddTwoInts, "add_two_ints")
    
    def send_request(self):
        request = Skill.Request()
        request.operation_name = "find"
        request.object = "apple"
        future = self.client.call_async(request)
        response = future.result()
        # print(future.done())
        print(response)
    
    def send_add_request(self):
        request = AddTwoInts.Request()
        request.a = 5
        request.b = 7
        future = self.add_client.call_async(request)
        response = future.result()
        print(response)

def main(args=None):
    rclpy.init(args=args)
    node = SkillClient()
    node.send_request()
    node.send_add_request()
    rclpy.shutdown()

if __name__ == "__main__":
    main()