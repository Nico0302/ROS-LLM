# lucas butler
from llm_interfaces.srv import Nav2Waypoint
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from llm_interfaces.srv import CreateWaypoint
import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
from llm_waypoints.default_waypoints import *
from nav2_simple_commander.robot_navigator import BasicNavigator

class Waypoints(Node):
    def __init__(self):
        super().__init__('llm_waypoints')

        self.nav = BasicNavigator()

        # Shows available waypoints
        self.waypoints_publisher_ = self.create_publisher(
            String, 
            'waypoints', 
            10)

        # Navigate to a waypoint
        self.navigate_to_waypoint_srv = self.create_service(
            Nav2Waypoint, 
            'navigate_to_waypoint', 
            self.navigate_to_waypoint)
        
        # Create a new waypoint
        self.create_waypoint_srv = self.create_service(
            CreateWaypoint,
            'create_waypoint',
            self.create_waypoint
        )
        
        # For grabbing positional information
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.pose_listener_callback,
            10
        )
        
        # Default Waypoints
        self.waypoint_list = default_waypoints

        # Stores current position of the robot
        self.position = None

    def _find_waypoint(self, waypoint_shortname):

        for point in self.waypoint_list:
            if point.short_name == waypoint_shortname:
                return point
            
        return None

    def navigate_to_waypoint(self, request: Nav2Waypoint.Request, response: Nav2Waypoint.Response):
        '''
        Searches the waypoint list for the short_name, if found, navigates to that position.
        '''
        
        # Find the waypoint by shortname
        waypoint = self._find_waypoint(request.waypoint_shortname)

        if not waypoint:
            self.get_logger().info("No waypoint found")
            return response

        self.get_logger().info("Calling navigation client...")

        # Create the target pose and send it to nav2
        pose_stamped = PoseStamped()
        pose_stamped.pose = waypoint.location
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'map'    # Fixed global point, not changing
        self.nav.goToPose(pose_stamped)

        return response

    def publish_waypoints(self):
        '''
        Publishes the string of available waypoints and their distances relative to the robot
            to the /waypoints topic.
        '''
        msg = String()

        if self.position is not None:
            waypoint_dicts = [waypoint.to_dict(self.position) for waypoint in self.waypoint_list]
            msg.data = json.dumps(waypoint_dicts)
            self.waypoints_publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        else:
            self.get_logger().info("Position has not be published by amcl_pose. Set starting pose in Nav2.")
 
    def pose_listener_callback(self, msg: PoseWithCovarianceStamped):
        '''
        Recieves the position of the robot from nav2
        '''
        self.get_logger().info(f"Received pose: {msg.pose.pose}")
        self.position = msg.pose.pose   # Traverse PoseWithCovarienceStamped -> PoseWithCovariance -> Pose
        assert isinstance(self.position, Pose)
        self.publish_waypoints()        # Update /waypoints after a new position is posted

    def create_waypoint(self, request: CreateWaypoint.Request, response: CreateWaypoint.Response):
        '''
        Creates a new waypoint.
        '''
        if self.position is not None:
            self.get_logger().info(f"Creating new waypoint: {request.waypoint_shortname} {self.position}")
            pose = self.position   # type Pose
            new_waypoint = Waypoint(
                request.waypoint_shortname,
                request.description,
                pose
            )
            self.waypoint_list.append(new_waypoint)
            return response
        else:
            self.get_logger().info(f"Cannot generate new waypoint, position not set.")
            return response


def main():
    rclpy.init()
    node = Waypoints()
    rclpy.spin(node)


    rclpy.shutdown()
        
if __name__ == '__main__':
    main()

