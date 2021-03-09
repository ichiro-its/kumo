from typing import Dict
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

from kumo.handlers.base_handler import BaseHandler
from kumo.handlers.subscription_handler import SubscriptionHandler
from kumo.message import Message, MessageType


class NodeHandler(BaseHandler):

    def __init__(self, name: str):
        super().__init__()

        self.node = rclpy.create_node(name)

        self.logger = get_logger('node_handler')

    def destroy(self) -> None:
        if super().destroy():
            self.logger.warn('Destroying Node %s...' % self.id)
            self.node.destroy_node()

    async def process(self) -> None:
        try:
            rclpy.spin_once(self.node, timeout_sec=0.01)

        except Exception as e:
            self.logger.error('Failed to spin Node %s! %s' % (id, e))

        await super().process()

    async def handle_message(self, message: Message) -> None:
        if message.type == MessageType.CREATE_SUBSCRIPTION:
            try:
                return self.handle_create_subscription(message)

            except Exception as e:
                self.logger.error('Failed to create a Subscription! %s' % str(e))
                self.send_error_respond(message, e)


        await super().handle_message(message)

    def handle_create_subscription(self, message: Message) -> None:
        if message.content.get('node_id') == self.id:
            handler = SubscriptionHandler(
                self.node, get_message(message.content.get('message_type')),
                message.content.get('topic_name'))

            self.attach(handler)

            self.logger.info('Subscription %s created!' % handler.id)
            self.send_respond(message, {'subscription_id': handler.id})
