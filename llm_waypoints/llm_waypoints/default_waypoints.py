from geometry_msgs.msg import Pose, Point, Quaternion
from llm_waypoints.waypoint import Waypoint

def create_pose(x, y, z):
    pose = Pose()
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    pose.position = point

    return pose


default_waypoints = [
    Waypoint(
        short_name='garbage_can', 
        description='A garbage can in the hallway', 
        location=create_pose(3, 0, 4)
    ),
    Waypoint(
        short_name='table', 
        description='A table in the kitchen', 
        location=create_pose(5, 3, 1)
    ),
    Waypoint(
        short_name='lamp', 
        description='A lamp in the living room', 
        location=create_pose(7, 3, 4)
    ),

]