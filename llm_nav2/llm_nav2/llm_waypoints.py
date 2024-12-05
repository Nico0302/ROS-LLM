from llm_interfaces.srv import Nav2Waypoint
from llm_interfaces.srv import GetWaypoints

import rclpy
from rclpy.node import Node

import json

import waypoint

class Waypoints(Node):

    def __init__(self):
        super().__init__('llm_waypoints')
        self.get_waypoints_srv = self.create_service(GetWaypoints, 'get_waypoints', self.get_waypoints)
        self.navigate_to_waypoint_srv = self.create_service(Nav2Waypoint, 'navigate_to_waypoint', self.navigate_to_waypoint)
        
        # Subscribe to the position topic
        self.position = None
        self.position_callback = None       # <-implement


        # Default Waypoints
        self.waypoint_list = [
            waypoint(short_name='garbage can', description='A garbage can in the hallway', location=(20, 20))
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

    def get_waypoints(self):
        '''
        Returns a JSON string of all the available waypoints
        '''
        self.position = self.position_callback()

        # Get the 
        waypoint_dicts = [waypoint.to_dict() for waypoint in self.waypoint_list]

        return json.dumps(waypoint_dicts, indent=4)

    

