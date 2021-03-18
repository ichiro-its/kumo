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

from kumo.handlers.base_handler import BaseHandler, Connection
from kumo.message import Message, MessageType, msg_to_dict


class SubscriptionHandler(BaseHandler):

    def __init__(self, connection: Connection, node: Node, message_type: MsgType, topic_name: str):
        super().__init__(connection)

        self.subscription = node.create_subscription(
            message_type, topic_name, self.callback, 10)

        self.logger = get_logger('subscription_%s' % self.id)

    def destroy(self) -> bool:
        if super().destroy():
            self.logger.warn('Destroying Subscription...')
            self.subscription.destroy()

    async def handle_message(self, message: Message) -> None:
        if message.type == MessageType.DESTROY_SUBSCRIPTION:
            try:
                return await self.handle_destroy_subscription(message)

            except Exception as e:
                self.logger.error('Failed to destroy Subscription! %s' % str(e))
                await self.send_error_response(message, e)

        await super().handle_message(message)

    async def handle_destroy_subscription(self, message: Message) -> None:
        if message.content.get('subscription_id') == self.id:
            self.destroy()
            await self.send_response(message, {'subscription_id': self.id})

    async def callback(self, msg: MsgType) -> None:
        try:
            msg_dict = msg_to_dict(msg)

            self.logger.debug('Sending Subscription message: %s' % str(msg_dict))

            await self.send_request(MessageType.SUBSCRIPTION_MESSAGE, {
                'subscription_id': self.id,
                'message': msg_dict})

        except Exception as e:
            self.logger.error('Failed to handle Subscription callback! %s' % str(e))
