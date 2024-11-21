import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rosidl_adapter import parser
import os

def pretty(d, indent=0):
   for key, value in d.items():
      print('\t' * indent + str(key))
      if isinstance(value, dict):
         pretty(value, indent+1)
      else:
         print('\t' * (indent+1) + str(value))

class Nav2LLM(Node):
    def __init__(self):
        super().__init__("nav2_llm")

    def get_topics_test(self):
        services = self.get_service_names_and_types_by_node("turtlebot3_joint_state", "/")
        for service in services:
            if (len(service) < 1):
                continue
            interfaces = service[1]
            if isinstance(interfaces, str):
                interfaces = [interfaces]
            for interface in interfaces:
                definiton = self.get_service_definiton(interface)
                print(definiton.request.annotations)
        return services
    
    def get_service_definiton(self, service_type_name):
        package_name, service_name = service_type_name.split('/', 1)
        package_dir = get_package_share_directory(package_name)

        # Construct the IDL file path based on the ROS 2 package structure
        srv_path = os.path.join(package_dir, f'{service_name}.srv')
        
        return parser.parse_service_file(package_name, srv_path)



def main():
    rclpy.init()
    nav2llm = Nav2LLM()
    try:
        print(nav2llm.get_topics_test())
    except KeyboardInterrupt:
        pass
    finally:
        nav2llm.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
