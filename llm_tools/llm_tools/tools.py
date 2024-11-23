from llm_tools.converters.base import BaseConverter
from llm_tools.service import Service
from llm_tools.converters.openai import OpenAIConverter
import rclpy
from rclpy.node import Node
from llm_interfaces.srv import CallTool
from llm_interfaces.srv import GetToolDescription
from llm_tools.tool import Tool
import json

class Tools(Node):

    tools: dict[str, Tool]

    converter: BaseConverter

    def __init__(self):
        super().__init__('llm_tools')
        self.tools = {}
        self.converter = OpenAIConverter()
        self.call_srv = self.create_service(CallTool, 'call_tool', self.call_tool)
        self.description_srv = self.create_service(GetToolDescription, 'get_tool_description', self.get_tool_description)

    def call_tool(self, request, response):
        [type, rest] = request.name.split(':', 1)
        if type == 'srv':
            [service_name, service_type] = rest.split(':', 1)
            response.parameters = json.dumps(self._call_service(service_type, service_name, request.parameters), indent=2)

        return response

    def get_tool_description(self, request, response):
        response.description = self._discover()
        return response

    def _discover(self, include_rules=list[str]):
        for node_name in self.get_node_names():
            services = Service.discover(self, node_name, '/')
            # apply rules
            if include_rules:
                services = [
                    service for service in services
                    if service.name in include_rules
                ]
            # add new services to the tools
            self.tools.update(
                self.converter.get_names(
                    services
                )
            )
        self.get_logger().info(f'Discovered {len(self.tools)} tools')
        return self.converter.get_definition(self.tools)
    
    def _get_function_name(self, service_name, service_type):
        return f"srv:{service_name}:{service_type}"



def main():
    rclpy.init()
    tools = Tools()
    # wait for node to initialize
    rclpy.spin_once(tools, timeout_sec=1)
    with open('tools.json', 'w') as f:
        json.dump(json.loads(tools._discover()), f, indent=4)

    tools.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()