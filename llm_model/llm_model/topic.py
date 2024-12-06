import importlib
from rclpy.node import Node
from rclpy.task import Future
import json

class Topic:
    """
    A subscribable topic to be injected into the LLM context.
    """

    name: str
    type: str
    active: bool

    value: str
    dirty: bool

    def __init__(self, name: str, type: str, active=False) -> None:
        self.name = name
        self.type = type
        self.active = active
        

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
    
    def future_complete(self) -> bool:
        """
        Returns whether the future is complete.
        """
        return self.dirty or not self.active

    def __str__(self) -> str:
        return f"{self.name}:\n{json.dumps(self.value)}"

    def _listener_callback(self, msg) -> None:
        """
        Callback for the subscription.
        """
        self.value = msg.data
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