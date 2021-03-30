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

from asyncio import Future
from kumo_json import dict_to_msg, msg_to_dict
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.logging import get_logger
from rclpy.node import Node, SrvType, SrvTypeRequest
from typing import List

from kumo.handlers.base_handler import BaseHandler, Connection
from kumo.message import Message, MessageType


class ServiceHandler(BaseHandler):

    def __init__(self, connection: Connection, node: Node,
                 service_type: SrvType, service_name: str):

        super().__init__(connection)

        self.service = node.create_service(
            service_type, service_name, None)

        self.request_headers: List[(Message, any)] = []

        self.logger = get_logger('service_%s' % self.id)

    def destroy(self) -> bool:
        if super().destroy():
            self.logger.warn('Destroying Service...')
            self.service.destroy()

    async def process(self) -> None:
        await super().process()

        while True:
            with self.service.handle as capsule:
                req_header = rclpy_implementation.rclpy_take_request(
                    capsule, self.service.srv_type.Request)

                if req_header is None:
                    break

                req, header = req_header
                req: SrvTypeRequest = req

                request = await self.send_request(MessageType.SERVICE_RESPONSE, {
                    'service_id': self.id,
                    'request': msg_to_dict(req)})

                self.request_headers.append((request, header))

    async def handle_message(self, message: Message) -> None:
        if message.type == MessageType.DESTROY_SERVICE:
            try:
                return await self.handle_destroy_service(message)

            except Exception as e:
                self.logger.error('Failed to destroy Service! %s' % str(e))
                await self.send_error_response(message, e)

        elif message.type == MessageType.SERVICE_RESPONSE:
            try:
                return await self.handle_service_response(message)

            except Exception as e:
                self.logger.error('Failed to handle Service response! %s' % str(e))
                await self.send_error_response(message, e)

        await super().handle_message(message)

    async def handle_destroy_service(self, message: Message) -> None:
        if message.content.get('service_id') == self.id:
            self.destroy()
            await self.send_response(message, {'service_id': self.id})

    async def handle_service_response(self, message: Message) -> None:
        new_request_headers: List[(Message, Future)] = []
        for (request, header) in self.request_headers:
            request: Message = request

            if message.type == request.type and message.id == request.id:
                res_dict = message.content.get('response')
                res = dict_to_msg(res_dict, self.service.srv_type.Response())
                self.service.send_response(res, header)
            else:
                new_request_headers.append((request, header))

        self.request_headers = new_request_headers
