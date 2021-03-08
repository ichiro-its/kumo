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

import json
from typing import List, Callable


class MessageHandler():

    def __init__(self, type: str):

        self.type: str = type
        self.id_counter: int = 0

        self.messages: List[str] = []

    async def send(self, callback: Callable) -> None:
        while len(self.messages) > 0:
            await callback(self.messages.pop())

    def request(self, content: dict) -> None:
        content['type'] = self.type

        content['id'] = str(self.id_counter)
        self.id_counter += 1

        self.messages.append(json.dumps(content))

    def response(self, id: str, content: dict) -> None:
        content['type'] = self.type
        content['id'] = id

        self.messages.append(json.dumps(content))

    def handle(self,  id: str, content: dict) -> None:
        pass
