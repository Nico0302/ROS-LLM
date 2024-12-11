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
    
    def to_storage_dict(self):
        return {
            'short_name': self.short_name,
            'description': self.description,
            'location': self.pose_to_dict(self.location)  # Convert Pose to dictionary
        }
    
    def calc_distance(self, position: Pose):
        '''
        Calculates the euclideans distance from the waypoint to the current location of the robot.
        '''
        x = abs(position.position.x - self.location.position.x)
        y = abs(position.position.y - self.location.position.y)
        z = abs(position.position.z - self.location.position.z)

        return round(math.sqrt(x*x + y*y + z*z), 2)
    
    @staticmethod
    def pose_to_dict(pose: Pose):
        '''
        Convert a Pose message to a dictionary.
        '''
        return {
            'position': {
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z
            },
            'orientation': {
                'x': pose.orientation.x,
                'y': pose.orientation.y,
                'z': pose.orientation.z,
                'w': pose.orientation.w
            }
        }

    @staticmethod
    def from_dict(data, pose: Pose):
        '''
        Creates a Waypoint object from a dictionary.
        '''
        location = Pose()
        location.position.x = data['location']['position']['x']
        location.position.y = data['location']['position']['y']
        location.position.z = data['location']['position']['z']
        location.orientation.x = data['location']['orientation']['x']
        location.orientation.y = data['location']['orientation']['y']
        location.orientation.z = data['location']['orientation']['z']
        location.orientation.w = data['location']['orientation']['w']

        return Waypoint(data['short_name'], data['description'], location)
    

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