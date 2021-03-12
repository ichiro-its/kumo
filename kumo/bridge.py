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
import rclpy
import websockets

from kumo.session import Session

WebSocket = websockets.WebSocketServerProtocol


class Bridge:

    def __init__(self, port: int, host: str):
        self.port = port
        self.host = host

        self.logger = rclpy.logging.get_logger('bridge')

    async def listen(self, websocket: WebSocket, path: str) -> None:
        await Session(websocket).listen()

    def run(self) -> None:
        rclpy.init()

        try:
            self.logger.info('Starting bridge on %s with port %d...'
                             % (self.host, self.port))

            websocket = websockets.serve(self.listen, self.host, self.port)

            asyncio.get_event_loop().run_until_complete(websocket)
            asyncio.get_event_loop().run_forever()

        except KeyboardInterrupt as e:
            self.logger.error('Keyboard interrupt! %s' % str(e))

        rclpy.shutdown()


def main():
    Bridge(8080, None).run()


if __name__ == '__main__':
    main()
