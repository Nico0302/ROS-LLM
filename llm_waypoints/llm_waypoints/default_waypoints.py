from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from llm_waypoints.waypoint import Waypoint

def create_pose(x, y, z):
    pose = PoseWithCovarianceStamped()
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    pose.pose.pose.position = point

    return pose

default_waypoints = [
    Waypoint(
        short_name='garbage can', 
        description='A garbage can in the hallway', 
        location=create_pose(1, 1, 1)
    ),
    Waypoint(
        short_name='table', 
        description='A table in the kitchen', 
        location=create_pose(1, 1, 1)
    ),
    Waypoint(
        short_name='lamp', 
        description='A lamp in the living room', 
        location=create_pose(1, 1, 1)
    ),

]