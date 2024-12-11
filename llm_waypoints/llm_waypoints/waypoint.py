# lucas butler
import math
from geometry_msgs.msg import Pose, Point

class Waypoint:
    def __init__(self, short_name: str, description: str, location: Pose):
        self.short_name = short_name
        self.description = description
        self.location = location

    def to_dict(self, position: Pose):
        '''
        Converts the waypoint to a string for the LLM to consume.
        '''
        return {
            'short_name': self.short_name,
            'description': self.description,
            'distance': self.calc_distance(position)
        }
    
    def calc_distance(self, position: Pose):
        '''
        Calculates the euclideans distance from the waypoint to the current location of the robot.
        '''
        x = abs(position.position.x - self.location.position.x)
        y = abs(position.position.y - self.location.position.y)
        z = abs(position.position.z - self.location.position.z)

        return round(math.sqrt(x*x + y*y + z*z), 2)
    

def test():
    starting_pose = Pose()
    point = Point()
    point.x = 2.
    point.y = 3.
    point.z = 4.
    starting_pose.position = point
    waypoint = Waypoint('test', 'test', starting_pose)

    end_pose = Pose()
    point = Point()
    point.x = 1.
    point.y = 1.
    point.z = 1.
    end_pose.position = point

    print(waypoint.to_dict(end_pose))


if __name__ == "__main__":

    test()