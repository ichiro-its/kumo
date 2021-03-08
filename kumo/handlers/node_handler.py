import rclpy
from rclpy.node import Node
from typing import Dict


class NodeHandler():

    def __init__(self):

        self.nodes: Dict[str, Node] = {}
        self.counter: int = 0

    def __del__(self):
        for id, node in self.nodes:
            try:
                node.destroy_node()
            except Exception as e:
                print('Failed to destroy node with id %d,' % id, e)


    def get_by_id(self, id: str) -> Node:
        if id not in self.nodes:
            raise Exception('Cannot find Node with id', id)

        return self.nodes[id]

    def spin(self) -> None:
        for id, node in self.nodes.items():
            try:
                rclpy.spin_once(node, timeout_sec=0.01)
            except Exception as e:
                print('Failed to spin node with id %d, %s' % (id, e))

    def create(self, name: str) -> (str, Node):
        node_id = str(self.counter)
        self.counter += 1

        node = Node(name)
        self.nodes[node_id] = node

        return node_id, node
