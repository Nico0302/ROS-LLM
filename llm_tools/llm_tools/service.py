import rclpy
import rclpy.exceptions
from rclpy.node import Node
from rosidl_runtime_py import set_message_fields
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
from llm_tools.tool import Tool
import importlib

class Service(Tool):
    def __init__(self, name, type):
        self.name = name
        self.type = type
    
    def call(self, node: Node, values: dict):
        node.get_logger().info(f'Calling service {self.name} of type {self.type}')
        try:
            parts = self.type.split('/')
            if len(parts) == 2:
                parts = [parts[0], 'srv', parts[1]]
            module = importlib.import_module('.'.join(parts[:-1]))
            srv_name = parts[-1]
            srv_module = getattr(module, srv_name)
        except (AttributeError, ModuleNotFoundError, ValueError):
            raise RuntimeError('The passed service type is invalid')
        try:
            srv_module.Request
            srv_module.Response
        except AttributeError:
            raise RuntimeError('The passed type is not a service')

        cli = node.create_client(srv_module, self.name)

        request = srv_module.Request()

        try:
            set_message_fields(request, values)
        except Exception as e:
            return 'Failed to populate field: {0}'.format(e)

        while not cli.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('service not available, waiting again...')

        return cli.call_async(request)
    
    @staticmethod
    def discover(node: Node, node_name: str, node_namespace: str):
        tools = []
        services = []
        try:
            services = node.get_service_names_and_types_by_node(node_name, node_namespace)
        except:
            pass
        for [service_name, types] in services:
            if not isinstance(types, list):
                types = [types]
            for service_type in types:
                if topic_or_service_is_hidden(service_name):
                    continue
                tools.append(Service(service_name, service_type))
        return tools