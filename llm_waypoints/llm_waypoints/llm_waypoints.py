from llm_interfaces.srv import Nav2Waypoint

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer

import json
from std_msgs.msg import String
from llm_waypoints.default_waypoints import *

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class Waypoints(Node):
    def __init__(self):
        super().__init__('llm_waypoints')

        self.nav = BasicNavigator()

        self.waypoints_publisher_ = self.create_publisher(
            String, 
            'waypoints', 
            10)

        self.navigate_to_waypoint_srv = self.create_service(
            Nav2Waypoint, 
            'navigate_to_waypoint', 
            self.navigate_to_waypoint)
        
        # For grabbing positional information
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.pose_listener_callback,
            10
        )
        
        # Default Waypoints
        self.waypoint_list = default_waypoints

        # Stores current position of the robot
        self.position = None

    def navigate_to_waypoint(self, request, response):
        '''
        Searches the waypoint list for the short name, if found, navigates to that position.
        '''
        destination = request.waypoint_shortname
        
        # Find the waypoint by shortname
        waypoint = None
        for point in self.waypoint_list:
            if point.short_name == destination:
                waypoint = point
                break

        # Call the nav2waypoint method in the waypoint
        self.get_logger().info("Calling navigation client...")

        self.nav.goToPose(waypoint.location)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            self.get_logger().info("Navigation to waypoint...")
            if feedback.navigation_duration > 600:
                self.nav.cancelTask()

    def publish_waypoints(self):
        '''
        Returns a JSON string of all the available waypoints
        '''
        msg = String()
        if self.position is not None:
            waypoint_dicts = [waypoint.to_dict(self.position) for waypoint in self.waypoint_list]
            msg.data = json.dumps(waypoint_dicts)
            self.waypoints_publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        else:
            self.get_logger().info("Position has not be published by amcl_pose. Set starting position in Nav2.")
 
    def pose_listener_callback(self, msg):
        self.get_logger().info(f"Received pose: {msg.pose.pose}")
        self.position = msg.pose.pose
        self.publish_waypoints()
        
def main():
    rclpy.init()
    node = Waypoints()
    rclpy.spin(node)
    print(node.get_waypoints())


    rclpy.shutdown()
        
if __name__ == '__main__':
    main()

