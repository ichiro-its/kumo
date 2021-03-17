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
from rclpy.logging import get_logger
from rclpy.node import Node, MsgType, SrvType
from typing import List

from kumo.handlers.base_handler import BaseHandler
from kumo.message import Message, MessageType


class ClientHandler(BaseHandler):

    def __init__(self, node: Node, service_type: SrvType, service_name: str):
        super().__init__()

        self.client = node.create_client(service_type, service_name)
        self.message_futures: List[(Message, Future)] = []

        self.logger = get_logger('client_handler')

    def destroy(self) -> bool:
        if super().destroy():
            self.logger.warn('Destroying client %s...' % self.id)
            self.client.destroy()

    async def process(self) -> None:
        new_message_futures: List[(Message, Future)] = []
        for (message, future) in self.message_futures:
            message: Message = message
            future: Future = future

            if future.done():
                res: MsgType = future.result()
                fields = res.get_fields_and_field_types()

                res_dict = {}
                for field in fields:
                    if hasattr(res, field):
                        res_dict[field] = getattr(res, field)

                self.logger.debug('Responding client %s service: %s'
                                  % (self.id, str(res_dict)))

                self.send_respond(message, {
                    'client_id': self.id,
                    'response': res_dict})

            else:
                new_message_futures.append((message, future))

        self.message_futures = new_message_futures

        await super().process()

    async def handle_message(self, message: Message) -> None:
        if message.type == MessageType.DESTROY_CLIENT:
            try:
                return self.handle_destroy_client(message)

            except Exception as e:
                self.logger.error('Failed to destroy client %s! %s'
                                  % (self.id, str(e)))

                self.send_error_respond(message, e)

        elif message.type == MessageType.CLIENT_REQUEST:
            try:
                return self.handle_client_request(message)

            except Exception as e:
                self.logger.error('Failed to request client %s service! %s'
                                  % (self.id, str(e)))

                self.send_error_respond(message, e)

        await super().handle_message(message)

    def handle_destroy_client(self, message: Message) -> None:
        if message.content.get('client_id') == self.id:
            self.destroy()
            self.send_respond(message, {'client_id': self.id})

    def handle_client_request(self, message: Message) -> None:
        if message.content.get('client_id') == self.id:
            fields = self.client.srv_type.Request.get_fields_and_field_types()
            req_dict: dict = message.content.get('request')

            req = self.client.srv_type.Request()
            for field in fields:
                if hasattr(req, field):
                    setattr(req, field, req_dict.get(field))

            self.logger.debug('Requesting client %s service: %s' % (self.id, str(req)))
            future = self.client.call_async(req)

            self.message_futures.append((message, future))
