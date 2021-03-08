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

from ..message_handler import MessageHandler
from ..node_handler import NodeHandler


class CreateNodeHandler(MessageHandler):

    def __init__(self, node_handler: NodeHandler):
        super().__init__('CREATE_NODE')

        self.node_handler: NodeHandler = node_handler

    def handle(self, id: str, request: dict) -> None:
        super().handle(id, request)

        try:
            node_name: str = str(request['node_name'])
            node_id, _ = self.node_handler.create(node_name)

            self.response(id, {'node_id': node_id})

        except Exception as e:
            print('Failed to handle create node!', e)
            self.response(id, {'error': str(e)})