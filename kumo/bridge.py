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

import argparse
import asyncio
import rclpy
import websockets
from typing import List

from kumo.session import Connection, Session


class Bridge:

    def __init__(self, port: int, hosts: List[str]):
        self.port = port
        self.hosts = hosts

        self.logger = rclpy.logging.get_logger('bridge')

    async def listen(self, connection: Connection, path: str) -> None:
        await Session(connection).listen()

    def run(self) -> None:
        rclpy.init()

        try:
            self.logger.info('Starting bridge server on port %d...' % (self.port))

            websocket = websockets.serve(self.listen, self.hosts, self.port)

            asyncio.get_event_loop().run_until_complete(websocket)
            asyncio.get_event_loop().run_forever()

        except KeyboardInterrupt as e:
            self.logger.error('Keyboard interrupt! %s' % str(e))

        except Exception as e:
            self.logger.error('Failed to start bridge! %s' % str(e))

        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(prog='ros2 run kumo bridge')

    parser.add_argument('--port', type=int, default=8080,
                        help='specify the port number of the bridge server')

    parser.add_argument('--hosts', nargs='+',
                        help='specify host addresses to be bind by the bridge server')

    arg = parser.parse_args()

    Bridge(arg.port, arg.hosts).run()


if __name__ == '__main__':
    main()
