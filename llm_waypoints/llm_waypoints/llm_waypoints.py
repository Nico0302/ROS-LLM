from llm_interfaces.srv import Nav2Waypoint
from llm_interfaces.srv import GetWaypoints

from turtlesim.msg import Pose

import rclpy
from rclpy.node import Node

import json
from llm_waypoints.waypoint import Waypoint

class Waypoints(Node):

    def __init__(self):
        super().__init__('llm_waypoints')
        self.get_waypoints_srv = self.create_service(
            GetWaypoints, 'get_waypoints', self.get_waypoints
            )
        self.navigate_to_waypoint_srv = self.create_service(
            Nav2Waypoint, 'navigate_to_waypoint', self.navigate_to_waypoint
            )
        
        # For grabbing positional information
        self.subscription = self.create_subscription(
            Pose, 
            '/turtle1/pose',
            self.pose_callback,
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
        self.position = self.get_position()
        print(self.position)

        # Get the 
        waypoint_dicts = [waypoint.to_dict(self.position) for waypoint in self.waypoint_list]

        response.waypoint_options = json.dumps(waypoint_dicts, indent=4)

        return response

    def pose_callback(self, msg):

        x = msg.x
        y = msg.y
        theta = msg.theta

        return x, y, theta
        
def main():
    rclpy.init()
    node = Waypoints()
    rclpy.spin(node)
    print(node.get_waypoints())


    rclpy.shutdown()
        
if __name__ == '__main__':
    main()

