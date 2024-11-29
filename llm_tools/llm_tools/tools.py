from llm_tools.service import Service
from llm_tools.filter import Filter, NameFilter
from llm_tools.converters import Converter, OpenAIConverter
import rclpy
from rclpy.node import Node
from llm_interfaces.srv import CallTool
from llm_interfaces.srv import GetToolDescriptions
import llm_tools.tool
import json

class Tools(Node):

    tools: dict[str, llm_tools.tool.Tool]

    def __init__(self):
        super().__init__('llm_tools')

        self.tools = {}

        self.declare_parameter('include_services', [])
        self.declare_parameter('exclude_services', [])

        self.call_srv = self.create_service(CallTool, 'call_tool', self.call_tool)
        self.description_srv = self.create_service(GetToolDescriptions, 'get_tool_descriptions', self.get_tool_descriptions)

    def call_tool(self, request, response):
        response.paramaters = self._call(request.name, json.loads(request.parameters))
        return response

    def get_tool_descriptions(self, request, response):
        include_services = self.get_parameter('include_services').get_parameter_value().string_array_value
        exclude_services = self.get_parameter('exclude_services').get_parameter_value().string_array_value

        filter = NameFilter(include_services, exclude_services) # type: ignore
        converter = OpenAIConverter()

        response.description = self._discover(
            filter, converter,
            node_namespace = request.node_namespace
        )
        return response

    def _discover(self, filter: Filter, converter: Converter, node_namespace = None):
        if not node_namespace:
            node_namespace = self.get_namespace()
        tools: list[llm_tools.tool.Tool] = []
        for node_name in self.get_node_names():
            tools += Service.discover(self, node_name, node_namespace)

        tools = filter.filter_tools(tools)

        for tool in tools:
            for name in tool.get_names():
                self.tools[name] = tool

        return converter.convert_tools(tools)
    
    def _call(self, name, parameters):
        return self.tools[name].call(self, parameters)



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