import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator

from geometry_msgs.msg import PoseWithCovarianceStamped

class Nav2LLM(Node):
    def __init__(self):
        super().__init__("llm_nav2")
        self.nav = BasicNavigator()
        
        self.nav_service = self.create_service(
            PoseWithCovarianceStamped,
            "go_to_waypoint",
            self.go_to_waypoint,
        )

    def go_to_waypoint(self, msg):

        assert isinstance(msg, PoseWithCovarianceStamped)

        self.nav.goToPose(msg)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            self.get_logger().info("Navigation to waypoint...")
            if feedback.navigation_duration > 600:
                self.nav.cancelTask()

def main():
    rclpy.init()
    node = Nav2LLM()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
