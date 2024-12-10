from llm_interfaces.srv import Nav2Waypoint
from llm_interfaces.srv import GetWaypoints

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer

import json
from llm_waypoints.waypoint import Waypoint

from geometry_msgs.msg import PoseWithCovarianceStamped

class Waypoints(Node):

    def __init__(self):
        super().__init__('llm_waypoints')
        self.get_waypoints_srv = self.create_service(GetWaypoints, 'get_waypoints', self.get_waypoints)
        self.navigate_to_waypoint_srv = self.create_service(Nav2Waypoint, 'navigate_to_waypoint', self.navigate_to_waypoint)
        
        # For grabbing positional information
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            f"amcl_pose",
            self.pose_listener_callback,
            10
        )

        # Default Waypoints
        self.waypoint_list = [
            Waypoint(short_name='garbage can', description='A garbage can in the hallway', location=(20, 20))
        ]

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
        waypoint.navigate_to_waypoint()

    def get_waypoints(self, request, response):
        '''
        Returns a JSON string of all the available waypoints
        '''

        # Get the 
        waypoint_dicts = [waypoint.to_dict(self.position) for waypoint in self.waypoint_list]

        response.waypoint_options = json.dumps(waypoint_dicts)

        return response

    def pose_listener_callback(self, msg):
        self.get_logger().info(f"Received pose: {msg.pose.pose}")

        self.position = msg.pose.pose
        
def main():
    rclpy.init()
    node = Waypoints()
    rclpy.spin(node)
    print(node.get_waypoints())


    rclpy.shutdown()
        
if __name__ == '__main__':
    main()

