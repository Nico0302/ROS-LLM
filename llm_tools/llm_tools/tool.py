from typing import Any
from rclpy.task import Future
from rclpy.node import Node
from abc import ABC, abstractmethod

import llm_tools.service

class ToolVisitor(ABC):
    @abstractmethod
    def visit_service(self, service: llm_tools.service.Service) -> Any:
        pass

class Tool(ABC):
    @abstractmethod
    def get_names(self) -> list[str]:
        pass

    @abstractmethod
    def call(self, node: Node, values: dict) -> Future:
        pass

    @staticmethod
    @abstractmethod
    def discover(node: Node, node_name: str, node_namespace: str) -> list['Tool']:
       pass

    @abstractmethod
    def accept(self, visitor: ToolVisitor) -> Any:
        pass