from rosidl_runtime_py import get_interface_path
from llm_tools.tool import Tool, ToolVisitor
from abc import ABC, abstractmethod

class Converter(ToolVisitor, ABC):

    @abstractmethod
    def convert_tools(self, tools: list[Tool]) -> str:
        """
        Converts a list of tools into a dialect conform representation.
        """
        pass

    def _get_interface_path(self, interface_name: str):
        [package, _, _] = interface_name.rsplit("/")
        return [package, get_interface_path(interface_name)]