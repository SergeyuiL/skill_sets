import rclpy
from rclpy.node import Node
from rclpy.service import Service
from robot_skills.srv import Skill
from example_interfaces.srv import AddTwoInts

class SkillServer(Node):
    def __init__(self):
        super().__init__("skill_simple_server")
        self.srv = self.create_service(Skill, "skill", self.exec_cb)
        self.add_srv = self.create_service(AddTwoInts, "add_two_ints", self.add_two_ints_cb)
    

    def exec_cb(self, request: Skill.Request, response: Skill.Response):
        func_name = request.operation_name
        obj_name = request.object
        self.get_logger().info("received {} {}".format(func_name, obj_name))
        # print()
        response.success = True
        response.error_info = "nothing"
        return response
    
    def add_two_ints_cb(self, request: AddTwoInts.Request, response: AddTwoInts.Response):
        response.sum = request.a + request.b
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SkillServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()