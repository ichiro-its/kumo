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


class SubscriptionHandler(BaseHandler):

    def __init__(self, node: Node, message_type: MsgType, topic_name: str):
        super().__init__()

        self.subscription = node.create_subscription(
            message_type, topic_name, self.callback, 10)

        self.logger = get_logger('node_handler')

    def destroy(self) -> None:
        if super().destroy():
            self.logger.warn('Destroying Subscription %s...' % self.id)
            self.subscription.destroy()

    async def handle_message(self, message: Message) -> None:
        if message.type == MessageType.DESTROY_SUBSCRIPTION:
            try:
                return self.handle_destroy_subscription(message)

            except Exception as e:
                self.logger.error('Failed to destroy Subscription %s! %s'
                                  % (self.id, str(e)))
                self.send_error_respond(message, e)

        await super().handle_message(message)

    def handle_destroy_subscription(self, message: Message) -> None:
        if message.content.get('subscription_id') == self.id:
            self.destroy()
            self.send_respond(message, {'subscription_id': self.id})

    def callback(self, message: MsgType) -> None:
        try:
            fields = message.get_fields_and_field_types()

            message_dict = {}
            for field in fields:
                if hasattr(message, field):
                    message_dict[field] = getattr(message, field)

            self.logger.debug('Subscription %s message sent! %s'
                              % (self.id, str(message_dict)))
            self.send_request(MessageType.SUBSCRIPTION_MESSAGE, {
                'subscription_id': self.id,
                'message': message_dict})

        except Exception as e:
            self.logger.error('Failed to handle Subscription %s callback! %s'
                              % (self.id, str(e)))
