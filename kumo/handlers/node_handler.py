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
from rclpy.logging import get_logger
from rosidl_runtime_py.utilities import get_message, get_service

from kumo.handlers.base_handler import BaseHandler, Connection
from kumo.handlers.client_handler import ClientHandler
from kumo.handlers.publisher_handler import PublisherHandler
from kumo.handlers.service_handler import ServiceHandler
from kumo.handlers.subscription_handler import SubscriptionHandler
from kumo.message import Message, MessageType


class NodeHandler(BaseHandler):

    def __init__(self, connection: Connection, name: str):
        super().__init__(connection)

        self.node = rclpy.create_node(name)

        self.logger = get_logger('node_%s' % self.id)

    def destroy(self) -> bool:
        if super().destroy():
            self.logger.warn('Destroying Node...')
            self.node.destroy_node()

    async def process(self) -> None:
        await super().process()

        try:
            rclpy.spin_once(self.node, timeout_sec=0)

        except Exception as e:
            self.logger.error('Failed to spin Node! %s' % str(e))

    async def handle_message(self, message: Message) -> None:

        if message.type == MessageType.DESTROY_NODE:
            try:
                return await self.handle_destroy_node(message)

            except Exception as e:
                self.logger.error('Failed to destroy Node! %s' % str(e))
                await self.send_error_response(message, e)

        elif message.type == MessageType.CREATE_PUBLISHER:
            try:
                return await self.handle_create_publisher(message)

            except Exception as e:
                self.logger.error('Failed to create a Publisher! %s' % str(e))
                await self.send_error_response(message, e)

        elif message.type == MessageType.CREATE_SUBSCRIPTION:
            try:
                return await self.handle_create_subscription(message)

            except Exception as e:
                self.logger.error('Failed to create a Subscription! %s' % str(e))
                await self.send_error_response(message, e)

        elif message.type == MessageType.CREATE_CLIENT:
            try:
                return await self.handle_create_client(message)

            except Exception as e:
                self.logger.error('Failed to create a Client! %s' % str(e))
                await self.send_error_response(message, e)

        elif message.type == MessageType.CREATE_SERVICE:
            try:
                return await self.handle_create_service(message)

            except Exception as e:
                self.logger.error('Failed to create a Service! %s' % str(e))
                await self.send_error_response(message, e)

        await super().handle_message(message)

    async def handle_destroy_node(self, message: Message) -> None:
        if message.content.get('node_id') == self.id:
            self.destroy()
            await self.send_response(message, {'node_id': self.id})

    async def handle_create_publisher(self, message: Message) -> None:
        if message.content.get('node_id') == self.id:
            publisher = PublisherHandler(
                self.connection, self.node,
                get_message(message.content.get('message_type')),
                message.content.get('topic_name'))

            self.attach(publisher)

            self.logger.info('Publisher %s created!' % publisher.id)
            await self.send_response(message, {'publisher_id': publisher.id})

    async def handle_create_subscription(self, message: Message) -> None:
        if message.content.get('node_id') == self.id:
            subscription = SubscriptionHandler(
                self.connection, self.node,
                get_message(message.content.get('message_type')),
                message.content.get('topic_name'))

            self.attach(subscription)

            self.logger.info('Subscription %s created!' % subscription.id)
            await self.send_response(message, {'subscription_id': subscription.id})

    async def handle_create_client(self, message: Message) -> None:
        if message.content.get('node_id') == self.id:
            client = ClientHandler(
                self.connection, self.node,
                get_service(message.content.get('service_type')),
                message.content.get('service_name'))

            self.attach(client)

            self.logger.info('Client %s created!' % client.id)
            await self.send_response(message, {'client_id': client.id})

    async def handle_create_service(self, message: Message) -> None:
        if message.content.get('node_id') == self.id:
            service = ServiceHandler(
                self.connection, self.node,
                get_service(message.content.get('service_type')),
                message.content.get('service_name'))

            self.attach(service)

            self.logger.info('Service %s created!' % service.id)
            await self.send_response(message, {'service_id': service.id})
