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
import websockets

from kumo.handlers.context_handler import ContextHandler
from kumo.message import Message, parse_message

WebSocket = websockets.WebSocketServerProtocol


class Session:

    id_counter: int = 0

    def __init__(self, websocket: WebSocket):
        self.id = str(Session.id_counter)
        Session.id_counter += 1

        self.websocket = websocket
        self.logger = get_logger('session')

        self.context = ContextHandler()

    def cleanup(self) -> None:
        self.context.destroy()

    async def listen(self) -> None:
        self.logger.info('Session %s started!' % self.id)

        while True:
            try:
                await self.context.process()

                while len(self.context.messages) > 0:
                    message: Message = self.context.messages.pop()
                    await self.websocket.send(message.toString())

                while True:
                    try:
                        await self.handle_message()

                    except asyncio.TimeoutError:
                        break

            except websockets.ConnectionClosed as e:
                self.logger.warn('Session %s closed! %s' % (self.id, str(e)))
                return self.cleanup()

            except KeyboardInterrupt as e:
                self.cleanup()
                raise e

            except Exception as e:
                self.logger.error('Something happened on Session %s! %s'
                                  % (self.id, str(e)))

    async def handle_message(self) -> None:
        message_string = await asyncio.wait_for(self.websocket.recv(), 0.01)

        try:
            message = parse_message(message_string)

            try:
                await self.context.handle_message(message)

            except Exception as e:
                self.logger.error('Failed to handle request! %s' % str(e))

        except Exception as e:
            self.logger.error('Failed to parse request! %s' % str(e))
