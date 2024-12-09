from functools import reduce
import importlib
from rclpy.node import Node
from rclpy.task import Future
from rosidl_runtime_py import message_to_ordereddict
import yaml

class Topic:
    """
    A subscribable topic to be injected into the LLM context.
    """

    name: str
    type: str
    path: list[str]
    active: bool

    value: str|None
    dirty: bool

    def __init__(self, name: str, type: str, path: list[str] = [], active=False) -> None:
        self.name = name
        self.type = type
        self.path = path
        self.active = active
        self.value = None
    
    @staticmethod
    def from_string(value: str, **kwargs) -> "Topic":
        [topic_name, topic_type] = value.split(':')
        [topic_name, *path] = topic_name.split('.')
        return Topic(topic_name, topic_type, path, **kwargs)

    def subscribe(self, node: Node) -> None:
        """
        Subscribes to the topic to receive data.
        """
        msg_module = self._get_msg_module()
        self.subscription = node.create_subscription(
            msg_module,
            self.name,
            self._listener_callback,
            10
        )

    def request_value(self) -> None:
        """
        Requests a new value from the topic.
        """
        self.dirty = False
    
    def is_future_complete(self) -> bool:
        """
        Returns whether the future is complete.
        """
        return self.dirty or not self.active
    
    def get_value(self) -> str|None|dict:
        """
        Returns the value of the topic.
        """
        if self.value is None:
            return None
        value = message_to_ordereddict(self.value)
        if len(self.path) > 0:
            value = reduce(lambda dct, key: dct[key], self.path, value) # type: ignore
        return value

    def __str__(self) -> str:
        return f"{self.name}:\n{yaml.dump(self.get_value())}"

    def _listener_callback(self, msg) -> None:
        """
        Callback for the subscription.
        """
        self.value = msg
        self.dirty = True

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