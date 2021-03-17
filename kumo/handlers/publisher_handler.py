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
from rclpy.node import Node, MsgType

from kumo.handlers.base_handler import BaseHandler
from kumo.message import Message, MessageType


class PublisherHandler(BaseHandler):

    def __init__(self, node: Node, message_type: MsgType, topic_name: str):
        super().__init__()

        self.publisher = node.create_publisher(message_type, topic_name, 10)

        self.logger = get_logger('publisher_%s' % self.id)

    def destroy(self) -> bool:
        if super().destroy():
            self.logger.warn('Destroying publisher...')
            self.publisher.destroy()

    async def handle_message(self, message: Message) -> None:
        if message.type == MessageType.DESTROY_PUBLISHER:
            try:
                return self.handle_destroy_publisher(message)

            except Exception as e:
                self.logger.error('Failed to destroy Publisher! %s' % str(e))
                self.send_error_respond(message, e)

        elif message.type == MessageType.PUBLISHER_MESSAGE:
            try:
                return self.handle_publisher_message(message)

            except Exception as e:
                self.logger.error('Failed to publish Publisher message! %s' % str(e))
                self.send_error_respond(message, e)

        await super().handle_message(message)

    def handle_destroy_publisher(self, message: Message) -> None:
        if message.content.get('publisher_id') == self.id:
            self.destroy()
            self.send_respond(message, {'publisher_id': self.id})

    def handle_publisher_message(self, message: Message) -> None:
        if message.content.get('publisher_id') == self.id:
            fields = self.publisher.msg_type.get_fields_and_field_types()
            msg_dict: dict = message.content.get('message')

            msg = self.publisher.msg_type()
            for field in fields:
                if hasattr(msg, field):
                    setattr(msg, field, msg_dict.get(field))

            self.logger.debug('Publishing Publisher message: %s' % str(msg))
            self.publisher.publish(msg)

            self.send_respond(message, {'publisher_id': self.id})
