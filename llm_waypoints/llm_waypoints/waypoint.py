from typing import Tuple
import json
import math

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion

class Waypoint:
    def __init__(self, short_name: str, description: str, location: PoseWithCovarianceStamped):
        self.short_name = short_name
        self.description = description
        self.location = location

    def to_dict(self, position: PoseWithCovarianceStamped):
        '''
        Converts the waypoint to a string for the LLM to consume.
        '''
        return {
            'short_name': self.short_name,
            'description': self.description,
            'distance': self.calc_distance(position)
        }
    
    def calc_distance(self, position):
        '''
        Calculates the euclideans distance from the waypoint to the current location of the robot.
        '''
        x = abs(position.pose.pose.position.x - self.location.pose.pose.position.x)
        y = abs(position.pose.pose.position.y - self.location.pose.pose.position.y)
        z = abs(position.pose.pose.position.z -  self.location.pose.pose.position.z)

        return round(math.sqrt(x*x + y*y + z*z), 2)
    
if __name__ == "__main__":

    starting_pose = PoseWithCovarianceStamped()
    point = Point()
    point.x = 2.
    point.y = 3.
    point.z = 4.
    starting_pose.pose.pose.position = point
    waypoint = Waypoint('test', 'test', starting_pose)

    end_pose = PoseWithCovarianceStamped()
    point = Point()
    point.x = 1.
    point.y = 1.
    point.z = 1.
    end_pose.pose.pose.position = point

    print(waypoint.to_dict(end_pose))