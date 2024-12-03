from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
import rclpy.wait_for_message

class TopicContext:
    """
    Manages all subscribed topics which get injected into the LLM context.
    """

    node: Node

    async_topics: list[str]
    sync_topics: list[str]

    def __init__(self, node: Node) -> None:
        self.node = node
        self._setup_parameters()

    def _setup_parameters(self) -> None:
        self.node.declare_parameter('async_topics', [], ParameterDescriptor(
            description='Topics which will not halt the LLM call.'
        ))
        self.node.declare_parameter('sync_topics', [], ParameterDescriptor(
            description='Topics which will halt the LLM call until new data is available. (Should be used if the topic value depends on the user input.)'
        ))

    def get_subscribed_topics(self, callback) -> None:
        """
        Returns all subscribed topic values.
        Will wait for new data from all wait_for_topics entries.
        """