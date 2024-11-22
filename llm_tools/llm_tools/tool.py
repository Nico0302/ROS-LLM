from rclpy.node import Node

class Tool:
    def call(self, node: Node, values: dict):
        raise NotImplementedError

    @staticmethod
    def discover(node: Node, node_name: str, node_namespace: str):
        raise NotImplementedError