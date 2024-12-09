from typing import Any
from rclpy.task import Future
from rclpy.node import Node
from abc import ABC, abstractmethod

class ToolVisitor(ABC):
    @abstractmethod
    def visit_service(self, service) -> Any:
        pass

    @abstractmethod
    def visit_topic(self, topic) -> Any:
        pass

class Tool(ABC):
    def setup(self, node: Node) -> None:
        """
        Setup the tool for use in the context of the given node.
        (Not every tool needs to implement this method)
        """
        pass

    @abstractmethod
    def get_names(self) -> list[str]:
        """
        Get the LLM friendly names of the tool.
        """
        pass

    @abstractmethod
    def call(self, node: Node, values: dict) -> Future | None:
        """
        Call the tool with the given values.
        """
        pass

    @staticmethod
    @abstractmethod
    def discover(node: Node, node_name: str, node_namespace: str) -> list['Tool']:
       pass

    @abstractmethod
    def accept(self, visitor: ToolVisitor) -> Any:
        pass