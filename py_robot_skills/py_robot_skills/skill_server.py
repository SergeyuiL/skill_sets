import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor
import threading
from rclpy.service import Service
# from robot_skills.srv import Skill
from skillsets_msg.srv import Skill
from .skill_lib import RobotSkillSets
# from robot_skills.msg import SkillStatus
from skillsets_msg.msg import SkillStatus

class SkillServer(Node):
    def __init__(self):
        super().__init__("skill_serve_new_node")
        self.srv = self.create_service(Skill, "wall_e_skill", self.exec_cb)
        self.status_pub = self.create_publisher(SkillStatus, "/wall_e_skill_status", 10)
        self.skill_set = RobotSkillSets()

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_status)
        
        # self.get_logger().info("Test:this is the new pkg in skillsets_ws")
    

    def exec_cb(self, request: Skill.Request, response: Skill.Response):
        func_name = request.operation_name
        obj_name = request.object
        con_name = request.container
        rel_ship = request.relationship
        # print(rel_ship)
        if func_name not in self.skill_set.func_map:
            response.success = False
            response.error_info = "wrong operation_name {}".format(func_name)
        else:
            success, info = self.skill_set.func_map[func_name](obj_name, con_name, rel_ship)
            response.success = success
            response.error_info = info
        
        self.skill_set.idle_status()
        return response
    
    def publish_status(self):
        status = SkillStatus()
        if self.skill_set.state["task_name"] is None:
            status.task_name = "Idle"
        else:
            status.task_name = self.skill_set["task_name"]
        status.task_status = self.skill_set.state["task_status"]
        self.status_pub.publish(status)

def spin_executor(executor):
    executor.spin()

def main(args=None):
    rclpy.init(args=args)
    server_node = SkillServer()  
    
    executor1 = MultiThreadedExecutor()
    executor1.add_node(server_node)
    executor1.add_node(server_node.skill_set)
    
    executor2 = SingleThreadedExecutor()
    executor2.add_node(server_node.skill_set.base_handler)

    thread1 = threading.Thread(target=spin_executor, args=(executor1,))
    thread2 = threading.Thread(target=spin_executor, args=(executor2,))
    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()

    executor1.shutdown()
    executor2.shutdown()
    server_node.destroy_node()
    server_node.skill_set.destroy_node()
    server_node.skill_set.base_handler.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()