from llm_interfaces.srv import Nav2Waypoint
from llm_interfaces.srv import GetWaypoints

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer

import json
from llm_waypoints.waypoint import Waypoint
from llm_waypoints.default_waypoints import *

from geometry_msgs.msg import PoseWithCovarianceStamped
from llm_interfaces.srv import GoToWaypoint

class Waypoints(Node):
    def __init__(self):
        super().__init__('llm_waypoints')
        self.get_waypoints_srv = self.create_service(GetWaypoints, 'get_waypoints', self.get_waypoints)
        self.navigate_to_waypoint_srv = self.create_service(Nav2Waypoint, 'navigate_to_waypoint', self.navigate_to_waypoint)
        
        # For grabbing positional information
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.pose_listener_callback,
            1
        )

        # For navigating to the waypoint
        self.navigation_client = self.create_client(
            GoToWaypoint,
            "/go_to_waypoint"
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

        request = GoToPose.Request()
        request.pose = waypoint.location
        future = self.navigation_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Result: {future.result().sum}")
        else:
            self.get_logger().error('Service call failed')
        

    def get_waypoints(self, request, response):
        '''
        Returns a JSON string of all the available waypoints
        '''
        
        if self.postion is not None:
            waypoint_dicts = [waypoint.to_dict(self.position) for waypoint in self.waypoint_list]
            response.waypoint_options = json.dumps(waypoint_dicts)
            return response
            
    def pose_listener_callback(self, msg):
        self.get_logger().info(f"Received pose: {msg.pose.pose}")
        assert isinstance(msg, PoseWithCovarianceStamped)
        self.position = msg
        
def main():
    rclpy.init()
    node = Waypoints()
    rclpy.spin(node)
    print(node.get_waypoints())


    rclpy.shutdown()
        
if __name__ == '__main__':
    main()

