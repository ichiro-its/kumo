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

from kumo_json import dict_to_msg
from rclpy.logging import get_logger
from rclpy.node import Node, MsgType

from kumo.handlers.base_handler import BaseHandler, Connection
from kumo.message import Message, MessageType


class PublisherHandler(BaseHandler):

    def __init__(self, connection: Connection, node: Node, message_type: MsgType, topic_name: str):
        super().__init__(connection)

        self.publisher = node.create_publisher(message_type, topic_name, 10)

        self.logger = get_logger('publisher_%s' % self.id)

    def destroy(self) -> bool:
        if super().destroy():
            self.logger.warn('Destroying publisher...')
            self.publisher.destroy()

    async def handle_message(self, message: Message) -> None:
        if message.type == MessageType.DESTROY_PUBLISHER:
            try:
                return await self.handle_destroy_publisher(message)

            except Exception as e:
                self.logger.error('Failed to destroy Publisher! %s' % str(e))
                await self.send_error_response(message, e)

        elif message.type == MessageType.PUBLISHER_MESSAGE:
            try:
                return await self.handle_publisher_message(message)

            except Exception as e:
                self.logger.error('Failed to publish Publisher message! %s' % str(e))
                await self.send_error_response(message, e)

        await super().handle_message(message)

    async def handle_destroy_publisher(self, message: Message) -> None:
        if message.content.get('publisher_id') == self.id:
            self.destroy()
            await self.send_response(message, {'publisher_id': self.id})

    async def handle_publisher_message(self, message: Message) -> None:
        if message.content.get('publisher_id') == self.id:
            msg_dict: dict = message.content.get('message')
            msg = dict_to_msg(msg_dict, self.publisher.msg_type())

            self.logger.debug('Publishing Publisher message: %s' % str(msg))
            self.publisher.publish(msg)

            await self.send_response(message, {'publisher_id': self.id})
