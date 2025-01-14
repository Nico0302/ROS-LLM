from typing import Sequence
from llm_tools.topic import Topic
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

        self.declare_parameter('include_services', ["/"])
        self.declare_parameter('exclude_services', ["*parameter*"])
        self.declare_parameter('include_topics', ["/"])
        self.declare_parameter('exclude_topics', ["*parameter*"])

        self.call_srv = self.create_service(CallTool, 'call_tool', self.call_tool)
        self.description_srv = self.create_service(GetToolDescriptions, 'get_tool_descriptions', self.get_tool_descriptions)

    def call_tool(self, request: CallTool.Request, response: CallTool.Response):
        self.get_logger().info(f'Calling tool {request.tool.name} with arguments {request.tool.arguments}')
        call_result = None
        try:
            call_result = self._call(request.tool.name, json.loads(request.tool.arguments))
            self.get_logger().info(f'Call result: {call_result}')
        except Exception as e:
            self.get_logger().error(f'Error calling tool {request.tool.name}: {e}')
            call_result = {"error": str(e)}
        if call_result is not None:
            response.output = json.dumps(call_result, indent=2)
        return response

    def get_tool_descriptions(self, request: GetToolDescriptions.Request, response: GetToolDescriptions.Response):
        include_services = self.get_parameter('include_services').get_parameter_value().string_array_value
        exclude_services = self.get_parameter('exclude_services').get_parameter_value().string_array_value
        include_topics = self.get_parameter('include_topics').get_parameter_value().string_array_value
        exclude_topics = self.get_parameter('exclude_topics').get_parameter_value().string_array_value

        filter = NameFilter(include_services, exclude_services, include_topics, exclude_topics) # type: ignore
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
            if node_name == self.get_name():
                continue
            tools += Service.discover(self, node_name, node_namespace)
            tools += Topic.discover(self, node_name, node_namespace)

        tools = filter.filter_tools(tools)

        for tool in tools:
            tool.setup(self)
            for name in tool.get_names():
                self.tools[name] = tool

        return converter.convert_tools(tools)
    
    def _call(self, name: str, parameters: dict):
        result = self.tools[name].call(self, parameters)
        # YOLO
        return None
        if result is not None:
            return self._wait_for_response(result)

    
    def _wait_for_response(self, future):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1)
            self.get_logger().info(f'Waiting for response: {future.done()}')
            if future.done():
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

def run_get_tool_descriptions(Args=None):
    rclpy.init()
    tools = Tools()
    # wait for node to initialize
    rclpy.spin_once(tools, timeout_sec=1)
    with open('tools.json', 'w') as f:
        json.dump(json.loads(tools._discover(NameFilter(['/'], [], ["/cmd_vel"], []), OpenAIConverter(), node_namespace="/")), f, indent=4)

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
