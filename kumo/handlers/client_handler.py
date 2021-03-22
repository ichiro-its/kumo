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

from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.logging import get_logger
from rclpy.node import Node, SrvType, SrvTypeResponse
from typing import List

from kumo.handlers.base_handler import BaseHandler, Connection
from kumo.message import dict_to_msg, Message, MessageType, msg_to_dict


class ClientHandler(BaseHandler):

    def __init__(self, connection: Connection, node: Node,
                 service_type: SrvType, service_name: str):

        super().__init__(connection)

        self.client = node.create_client(service_type, service_name)
        self.requests: List[Message] = []

        self.logger = get_logger('client_%s' % self.id)

    def destroy(self) -> bool:
        if super().destroy():
            self.logger.warn('Destroying Client...')
            self.client.destroy()

    async def process(self) -> None:
        await super().process()

        ress: List[SrvTypeResponse] = []
        while True:
            with self.client.handle as capsule:
                _, res = rclpy_implementation.rclpy_take_response(
                    capsule, self.client.srv_type.Response)

                if res is None:
                    break

                ress.append(res)

        for request, res in zip(self.requests, ress):
            request: Message = request
            res: SrvTypeResponse = res

            await self.send_response(request, {
                'client_id': self.id,
                'response': msg_to_dict(res)})

            self.requests.remove(request)

    async def handle_message(self, message: Message) -> None:
        if message.type == MessageType.DESTROY_CLIENT:
            try:
                return await self.handle_destroy_client(message)

            except Exception as e:
                self.logger.error('Failed to destroy Client! %s' % str(e))
                await self.send_error_response(message, e)

        elif message.type == MessageType.CLIENT_REQUEST:
            try:
                return await self.handle_client_request(message)

            except Exception as e:
                self.logger.error('Failed to request Client service! %s' % str(e))
                await self.send_error_response(message, e)

        await super().handle_message(message)

    async def handle_destroy_client(self, message: Message) -> None:
        if message.content.get('client_id') == self.id:
            self.destroy()
            await self.send_response(message, {'client_id': self.id})

    async def handle_client_request(self, message: Message) -> None:
        if message.content.get('client_id') == self.id:
            req_dict: dict = message.content.get('request')
            req = dict_to_msg(req_dict, self.client.srv_type.Request())

            self.logger.debug('Requesting client service: %s' % str(req))
            self.client.call_async(req)

            self.requests.append(message)
