# Copyright (c) 2021 Ichiro ITS
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

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
