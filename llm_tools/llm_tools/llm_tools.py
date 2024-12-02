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
    """
    A ROS 2 node that provides a service to call tools and a service to get tool descriptions.
    """

    tools: dict[str, llm_tools.tool.Tool]

    def __init__(self):
        super().__init__('llm_tool_server')

        self.tools = {}

        self.declare_parameter('include_services', [])
        self.declare_parameter('exclude_services', [])

        self.call_srv = self.create_service(CallTool, 'call_tool', self.call_tool)
        self.description_srv = self.create_service(GetToolDescriptions, 'get_tool_descriptions', self.get_tool_descriptions)

    def call_tool(self, request: CallTool.Request, response: CallTool.Response):
        response.output = json.dumps(self._call(request.tool.name, json.loads(request.tool.parameters)), indent=2)
        return response

    def get_tool_descriptions(self, request: GetToolDescriptions.Request, response: GetToolDescriptions.Response):
        include_services = self.get_parameter('include_services').get_parameter_value().string_array_value
        exclude_services = self.get_parameter('exclude_services').get_parameter_value().string_array_value

        filter = NameFilter(include_services, exclude_services) # type: ignore
        converter = OpenAIConverter()

        response.tools = self._discover(
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
    
    def _call(self, name: str, parameters: dict):
        return self._wait_for_response(self.tools[name].call(self, parameters))
    
    def _wait_for_response(self, future):
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def run_call_tool():
    rclpy.init()
    tools = Tools()
    # wait for node to initialize
    rclpy.spin_once(tools, timeout_sec=1)
    tools.tools = {
        'llm_tools-get_parameters': Service('/llm_tools/get_parameters', 'rcl_interfaces/srv/GetParameters'),
    }
    print(tools._call('llm_tools-get_parameters', {"names": ["include_services", "exclude_services"]}))

    tools.destroy_node()
    rclpy.shutdown()

def run_get_tool_descriptions():
    rclpy.init()
    tools = Tools()
    # wait for node to initialize
    rclpy.spin_once(tools, timeout_sec=1)
    with open('tools.json', 'w') as f:
        json.dump(json.loads(tools._discover(NameFilter(['/*'], []), OpenAIConverter())), f, indent=4)

    tools.destroy_node()
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    tools = Tools()
    rclpy.spin(tools)
    tools.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()