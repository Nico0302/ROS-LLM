import importlib
from llm_tools.tool import Tool
from rclpy.node import Node
import rclpy.publisher
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
from rosidl_runtime_py import set_message_fields

class Topic(Tool):
    """
    A subscribable topic to be injected into the LLM context.
    """

    name: str
    type: str

    publisher: rclpy.publisher.Publisher | None

    def __init__(self, name: str, type: str) -> None:
        self.name = name
        self.type = type
        self.publisher = None

    def setup(self, node: Node) -> None:
        self._create_publisher(node)

    def _create_publisher(self, node: Node):
        self.publisher = node.create_publisher(self._get_msg_module(), self.name, 10)

    def get_names(self):
        return [self.name[1:60].replace("/", "-").replace(" ", "_")]
    
    def accept(self, visitor):
        return visitor.visit_topic(self)
    
    def call(self, node, values):
        if self.publisher is None:
            raise RuntimeError('The publisher is not initialized')
        if "values" not in values:
            raise RuntimeError('The values key is missing')
        msg_module = self._get_msg_module()
        msg = msg_module()
        try:
            set_message_fields(msg, values["values"])
        except Exception as e:
            raise RuntimeError('Failed to populate field: {0}'.format(e))
        times = values.get("times", 1)
        
        count = 0

        def timer_callback():
            if self.publisher is None:
                return
            nonlocal count
            count += 1
            self.publisher.publish(msg)

        timer_callback()
        if times != 1:
            timer = node.create_timer(1, timer_callback)
            while times == 0 or count < times:
                rclpy.spin_once(node)
            node.destroy_timer(timer)

        return None
        
    
    @staticmethod
    def discover(node, node_name, node_namespace):
        tools = []
        topics = []
        try:
            topics = node.get_topic_names_and_types()
        except Exception as e:
            print('Failed to get publisher names and types by node: {0}'.format(e))
            return []

        for [topic_name, types] in topics:
            if not isinstance(types, list):
                types = [types]
            for topic_type in types:
                if topic_or_service_is_hidden(topic_name):
                    continue
                tools.append(Topic(topic_name, topic_type))
        return tools
    
    def _get_msg_module(self):
        msg_module = None
        try:
            parts = self.type.split('/')
            if len(parts) == 2:
                parts = [parts[0], 'msg', parts[1]]
            module = importlib.import_module('.'.join(parts[:-1]))
            msg_name = parts[-1]
            msg_module = getattr(module, msg_name)
        except (AttributeError, ModuleNotFoundError, ValueError):
            raise RuntimeError('The passed message type is invalid')
        return msg_module
    
    def __eq__(self, value: object) -> bool:
        if not isinstance(value, Topic):
            return False
        return self.name == value.name and self.type == value.type