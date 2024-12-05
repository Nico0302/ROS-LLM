from typing import Tuple
import json

class Waypoint:

    def __init__(self, short_name: str, description: str, location: Tuple[float, float]):
        self.short_name = short_name
        self.description = description
        self.location = location


    def navigate_to_waypoint(self):
        '''
        Call the nav2 command to navigate to self.location.
        '''
        pass

    def to_dict(self, position):

        distance = None # Calculate euclidean distance from location
        return {
            'short_name': self.short_name,
            'description': self.description,
            'location': self.location,
            'distance': distance
        }