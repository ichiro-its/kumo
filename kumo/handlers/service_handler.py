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

from rclpy.logging import get_logger
from rclpy.node import Node, MsgType, SrvType

from kumo.handlers.base_handler import BaseHandler
from kumo.message import Message, MessageType


class ServiceHandler(BaseHandler):

    def __init__(self, node: Node, service_type: SrvType, service_name: str):
        super().__init__()

        self.service = node.create_service(
            service_type, service_name, self.callback)

        self.logger = get_logger('service_%s' % self.id)

    def destroy(self) -> bool:
        if super().destroy():
            self.logger.warn('Destroying Service...')
            self.service.destroy()

    async def handle_message(self, message: Message) -> None:
        if message.type == MessageType.DESTROY_SERVICE:
            try:
                return self.handle_destroy_service(message)

            except Exception as e:
                self.logger.error('Failed to destroy Service! %s' % str(e))
                self.send_error_respond(message, e)

        await super().handle_message(message)

    def handle_destroy_service(self, message: Message) -> None:
        if message.content.get('service_id') == self.id:
            self.destroy()
            self.send_respond(message, {'service_id': self.id})

    def callback(self, request: MsgType, response: SrvType) -> None:
        pass
