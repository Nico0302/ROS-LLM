import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from llm_interfaces.srv import NavigateToWaypoint


class Nav2LLM(Node):
    def __init__(self):
        super().__init__("llm_nav2")
        self.nav = BasicNavigator()

        


def main():
    rclpy.init()
    node = Nav2LLM()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
