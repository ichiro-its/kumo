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

import asyncio
from rclpy.logging import get_logger

from kumo.handlers.base_handler import BaseHandler, Connection
from kumo.handlers.node_handler import NodeHandler
from kumo.message import Message, MessageType


class SessionHandler(BaseHandler):

    def __init__(self, connection: Connection):
        super().__init__(connection)

        self.logger = get_logger('session_%s' % self.id)

    async def process(self) -> None:
        await super().process()

        while len(self.messages) > 0:
            await self.handle_message(self.messages.pop())

        while True:
            try:
                message = await asyncio.wait_for(self.recover(), 0.01)
                await self.handle_message(message)

            except asyncio.TimeoutError:
                break

    async def handle_message(self, message: Message) -> None:
        if message.type == MessageType.CREATE_NODE:
            try:
                return await self.handle_create_node(message)

            except Exception as e:
                self.logger.error('Failed to create a Node! %s' % str(e))
                await self.send_error_response(message, e)

        await super().handle_message(message)

    async def handle_create_node(self, message: Message) -> None:
        handler = NodeHandler(self.connection, message.content.get('node_name'))
        self.attach(handler)

        self.logger.info('Node %s created!' % handler.id)
        await self.send_response(message, {'node_id': handler.id})
