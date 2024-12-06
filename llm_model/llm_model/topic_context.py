from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from llm_model.topic import Topic
import rclpy.wait_for_message

class TopicContext:
    """
    Manages all subscribed topics which get injected into the LLM context.
    """

    node: Node

    topics: list[Topic]

    def __init__(self, node: Node) -> None:
        self.node = node
        self._setup_parameters(
        )

    def _setup_parameters(self) -> None:
        self.node.declare_parameter('passiv_topics', ["/example_topic:string"], ParameterDescriptor(
            description='Topics which will not halt the LLM call.'
        ))
        self.node.declare_parameter('activ_topics', ["/example_topic:string"], ParameterDescriptor(
            description='Topics which will halt the LLM call until new data is available. (Should be used if the topic value depends on the user input.)'
        ))

    def subscribe_topics(self) -> None:
        """
        Subscribes to all topics.
        """
        passiv_topics = self.node.get_parameter('passiv_topics').get_parameter_value().string_array_value
        activ_topics = self.node.get_parameter('activ_topics').get_parameter_value().string_array_value

        self.topics = []
        for topic in passiv_topics:
            if 'example_topic' not in topic:
                self.topics.append(Topic.from_string(topic, active=False))
        for topic in activ_topics:
            if 'example_topic' not in topic:
                self.topics.append(Topic.from_string(topic, active=True))

        for topic in self.topics:
            topic.subscribe(self.node)

    def get_subscribed_topics(self) -> list[Topic]:
        """
        Returns all subscribed topic values.
        Will wait for new data from all wait_for_topics entries.
        """
        if not hasattr(self, 'topics'):
            self.subscribe_topics()
        for topic in self.topics:
            topic.request_value()
        while not all([topic.is_future_complete() for topic in self.topics]):
            rclpy.spin_once(self.node)

        return self.topics