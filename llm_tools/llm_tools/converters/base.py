from rosidl_runtime_py import get_interface_path
from llm_tools.tool import Tool

class BaseConverter:
    def get_names(self, tools: list[Tool]) -> dict[str, Tool]:
        """
        Generates dialect conform names for a list of tools.
        """
        raise NotImplementedError

    def get_definition(self, tools: dict[str, Tool]) -> str:
        """
        Converts a tool list into a tool definition for a specifc dialect.
        """
        raise NotImplementedError

    def _get_interface_path(self, interface_name: str):
        [package, _, _] = interface_name.rsplit("/")
        return [package, get_interface_path(interface_name)]