

from llm_tools.service import Service
from llm_tools.llm_tools import Tools
import rclpy


def test_call_tool():
    rclpy.init()
    tools = Tools()
    # wait for node to initialize
    rclpy.spin_once(tools, timeout_sec=1)
    tools.tools = {
        'llm_tools-get_parameters': Service('/llm_tools/get_parameters', 'rcl_interfaces/srv/GetParameters'),
    }
    assert len(tools._call('llm_tools-get_parameters', {"names": ["include_services", "exclude_services"]}).values) == 2 # type: ignore

    tools.destroy_node()
    rclpy.shutdown()