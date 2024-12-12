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
import os

class Waypoints(Node):
    def __init__(self):
        super().__init__('llm_waypoints')

        self.waypoint_storage_file = os.path.join("./src/ROS-LLM/llm_waypoints/llm_waypoints", "waypoint_storage.json")
        self.get_logger().info(f"{self.waypoint_storage_file}")

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
            self.navigate_to_waypoint
        )
        
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

        self.waypoint_list = default_waypoints

        if os.path.exists(self.waypoint_storage_file):
            self.waypoint_list = self.load_waypoints_from_file()      # load the existing list of waypoints 
            self.get_logger().info(f"Found {len(self.waypoint_list)} waypoints.") 

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
            self.save_waypoints_to_file()               # Save the list of waypoints
            self.publish_waypoints()
            return response
        else:
            self.get_logger().info(f"Cannot generate new waypoint, position not set.")
            return response
        
    def save_waypoints_to_file(self):
        '''
        Serialize Waypoints and save them to a JSON file.
        '''
        waypoints_dict = [wp.to_storage_dict() for wp in self.waypoint_list]  # Assuming pose is the robot's current position
        with open(self.waypoint_storage_file, 'w') as file:
            json.dump(waypoints_dict, file, indent=4)

    def load_waypoints_from_file(self):
        '''
        Load Waypoints from a JSON file and deserialize them into Waypoint objects.
        '''
        with open(self.waypoint_storage_file, 'r') as file:
            waypoints_dict = json.load(file)
        
        waypoints = []
        for data in waypoints_dict:
            pose = Pose()
            waypoints.append(Waypoint.from_dict(data, pose))
        
        print(waypoints)
        return waypoints
    

def test_storage():
    # Initialize rclpy
    rclpy.init(args=None)

    # Create node instance
    node = Waypoints()

    pose = Pose()

    pose.position.x = 7.
    pose.position.y = 7.
    pose.position.z = 7.

    node.position = pose
    node.save_waypoints_to_file()

    # Now load from the file and check if the waypoint is correctly deserialized
    node.load_waypoints_from_file()
    for wp in node.waypoint_list:
        print(f"Loaded waypoint: {wp.short_name}, {wp.description}, Location: {wp.location.position.x}, {wp.location.position.y}, {wp.location.position.z}")

    # Spin the node to ensure any necessary callbacks (like pose updates) can run
    # Since this is just a test, we'll spin for a short period
    rclpy.spin_once(node, timeout_sec=1.0)

    waypoint_request = CreateWaypoint.Request()
    waypoint_request.waypoint_shortname = 'test'
    waypoint_request.description = "test"

    waypoint_response = CreateWaypoint.Response()

    node.create_waypoint(waypoint_request, waypoint_response)

    node.save_waypoints_to_file()
    node.load_waypoints_from_file()
    for wp in node.waypoint_list:
        print(f"Loaded waypoint: {wp.short_name}, {wp.description}, Location: {wp.location.position.x}, {wp.location.position.y}, {wp.location.position.z}")


    # Shutdown rclpy after the test is done
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    waypoints = Waypoints()
    rclpy.spin(waypoints)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
