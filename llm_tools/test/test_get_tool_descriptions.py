import json
from llm_tools.filter import NameFilter
from llm_tools.converters import OpenAIConverter
from llm_tools.llm_tools import Tools
import rclpy

EXPECTED_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "get_tool_descriptions",
            "parameters": {
                "type": "object",
                "properties": {
                    "node_namespace": {
                        "type": "string",
                        "description": "(optional) Defaults to the tool node namepsace"
                    }
                }
            },
            "description": "List all available tools in the Open AI tool description format"
        }
    },
]

def test_get_tool_descriptions():
    rclpy.init()
    tools = Tools()
    # wait for node to initialize
    rclpy.spin_once(tools, timeout_sec=1)

    assert json.loads(tools._discover(NameFilter(['*/get_tool_descriptions'], []), OpenAIConverter())) == EXPECTED_TOOLS

    tools.destroy_node()
    rclpy.shutdown()